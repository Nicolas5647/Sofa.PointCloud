/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2025 CNRS, INRIA, USTL, UJF, CNRS, MGH               *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/pointcloud/fwd.h>
#include <sofa/pointcloud/components/PointCloudVisualModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <Eigen/Dense>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/pointcloud/components/utils.h>
#include <sofa/helper/ScopedAdvancedTimer.h>


namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudVisualModel>(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A point cloud visual model.")
                             .add< sofa::pointcloud::components::PointCloudVisualModel >());
}

}

namespace sofa::pointcloud::components {

    PointCloudVisualModel::PointCloudVisualModel() :
        l_geometries(initLink("geometries", "link to the topology container"))
        , l_selector(initLink("selector", "link to the selector"))
        , d_indices(initData(&d_indices, "indices", " the indices in the geometry to display"))
        , d_frames(initData(&d_frames, "frames", " set of frame controlling the geometry"))
        , d_initFrames(initData(&d_initFrames, "initFrames", " initiale frames location/orientation, if empty, use frames as reference"))
        , d_frameIndices(initData(&d_frameIndices, "frameIndices", " the indice mapping each splats to a frame"))
        , d_uniformScale(initData(&d_uniformScale, 1.0f, "uniformScale", " scale factor to apply to whole shape"))
        , d_isStaticModel(initData(&d_isStaticModel, false, "isStatic", "true if the data is fully static" ))
        , d_doInit(initData(&d_doInit, false, "doInit", "true if the data is fully static" )) {
        d_frames.setValue({defaulttype::Rigid3Types::Coord{{0.0,0.0,0.0},{0.0,0.0,0.0,1.0}}});
    }

    PointCloudVisualModel::~PointCloudVisualModel() {
        delete data;
    }

    void PointCloudVisualModel::init() {
        if( isComponentStateValid() )
            return;
        Inherit1::init();


        if(l_geometries.size() == 0) {
            msg_error() << "Missing the geometry to render. To remove this message, set the link named 'geometry' so it point to a valid PointCloudContainer ";
            d_componentState = sofa::core::objectmodel::ComponentState::Invalid;
            return;
        }

        data = new GaussianData{};
        for (auto& visual : l_geometries) {
            if (visual->isComponentStateValid()) {
                append(*data, *visual->data);
            }
        }

        size_t totalIndicesSize = 0;
        size_t totalFramesSize = 0;
        for (auto geometry : l_geometries) {
            if (geometry) {
                totalIndicesSize += geometry->d_indices.getValue().size();
                if (geometry->data) {
                    totalFramesSize += geometry->data->size();
                }
            }
        }

        if (totalIndicesSize > 0) {
            auto indicesAccessor = sofa::helper::getWriteOnlyAccessor(d_indices);
            indicesAccessor.resize(totalIndicesSize);
            
            size_t currentOffset = 0;
            for (auto geometry : l_geometries) {
                auto newIndices = geometry->d_indices.getValue();
                if (!newIndices.empty()) {
                    std::copy(newIndices.begin(), newIndices.end(), indicesAccessor.begin() + currentOffset);
                    currentOffset += newIndices.size();
                }
            }
        }

        if (totalFramesSize > 0) {
            auto framesAccessor = sofa::helper::getWriteOnlyAccessor(d_frameIndices);
            framesAccessor.resize(totalFramesSize, 0);
        }

        initTransform();

        for (size_t i = 0; i < l_geometries.size(); ++i) {
            addUpdateCallback("update", {&l_geometries[i]->d_componentState}, [this, i](const sofa::core::DataTracker&){
                if(!l_geometries[i]->isComponentStateValid()) {
                    msg_warning() << "The geometry associated with this visual model is in an invalid state";
                    return sofa::core::objectmodel::ComponentState::Invalid;
                }

                auto frames = sofa::helper::getWriteOnlyAccessor(d_frameIndices);
                if(frames.size()!=l_geometries[i]->data->size()) {
                    frames.resize(l_geometries[i]->data->size(), 0);
                }

                initTransform();

                return sofa::core::objectmodel::ComponentState::Valid;
            }, {&d_frameIndices});
        }



        d_componentState = sofa::core::objectmodel::ComponentState::Valid;
    }

    void PointCloudVisualModel::initTransform() {
        std::cout << "INIT TRANSFORM" << std::endl;
        if(d_initFrames.isSet()) {
            std::cout << "INIT TRANSFORM => Initialize using frames, so future frames will be interpreted as delta compared to this reference" << std::endl;
            d_initFrames.updateIfDirty();
            d_initFrames.setParent(nullptr);
        } else{
            auto initFrames = helper::getWriteOnlyAccessor(d_initFrames);
            auto frames = helper::getReadAccessor(d_frames);
            initFrames.clear();
            for(auto& frame : frames) {
                initFrames.push_back(defaulttype::Rigid3Types::Coord{});
            }
        }

        auto initFrames = helper::getReadAccessor(d_initFrames);

        referenceFrames.clear();
        localToGlobalFrames.clear();
        for(auto& frame : initFrames) {
            localToGlobalFrames.push_back(frame-frame);
            referenceFrames.push_back(frame);
        };
    }

    void PointCloudVisualModel::doUpdateVisual(const sofa::core::visual::VisualParams* vparams) {
        SOFA_UNUSED(vparams);
        d_frameIndices.updateIfDirty();
    }

    bool PointCloudVisualModel::updateLayout(BaseCamera* camera, float aspect) {
        SCOPED_TIMER("PointCloudVisualModel::updateLayout");
        if (!camera || !l_selector) return false;

        auto indicesToRender = helper::getWriteOnlyAccessor(d_indices);
        return l_selector->updateSelection(camera, aspect, *indicesToRender);
    }


    bool PointCloudVisualModel::updateSh(GaussianData* renderingData, int offset) {
        if (!l_selector) return false;

        return l_selector->updateSh(renderingData, offset);
    }

    void PointCloudVisualModel::draw(const sofa::core::visual::VisualParams* vparams) {
        if (l_selector.get()) l_selector.get()->draw(vparams);
    }

}
