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
#pragma once

#include <sofa/pointcloud/config.h>
#include <sofa/pointcloud/components/PointCloudContainer.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/gl/GLSLShader.h>
#include <sofa/helper/SelectableItem.h>
#include <sofa/pointcloud/components/selectors/PointCloudSelector.h>

namespace sofa::pointcloud::components
{
using sofa::core::objectmodel::BaseObject;
using sofa::component::visual::BaseCamera;

// References:
//  - https://huggingface.co/blog/gaussian-splatting
class PointCloudVisualModel : public sofa::core::visual::VisualModel {
private:
    template<class T>
    using Link = core::objectmodel::SingleLink<PointCloudVisualModel, T, core::objectmodel::BaseLink::FLAG_STOREPATH>;
    template<class T>
    using MultiLink = core::objectmodel::MultiLink<PointCloudVisualModel, T, core::objectmodel::BaseLink::FLAG_STOREPATH>;

public:
    SOFA_CLASS(PointCloudVisualModel, BaseObject);

    PointCloudVisualModel();
    ~PointCloudVisualModel() override;

    MultiLink<PointCloudContainer> l_geometries;
    Link<PointCloudSelector> l_selector;

    GaussianData* data{nullptr};

    Data<type::vector<int>> d_indices;
    Data<type::vector<defaulttype::Rigid3Types::Coord>> d_frames;
    Data<type::vector<defaulttype::Rigid3Types::Coord>> d_initFrames;
    Data<type::vector<int>> d_frameIndices;
    Data<float> d_uniformScale;
    Data<bool> d_isStaticModel;
    Data<bool> d_doInit;

    void init() override;
    void doUpdateVisual(const sofa::core::visual::VisualParams* vparams) final;
    void draw(const sofa::core::visual::VisualParams* vparams) override;

public:
    void initTransform();
    type::vector<defaulttype::Rigid3Types::Coord> referenceFrames;
    type::vector<defaulttype::Rigid3Types::Coord> localToGlobalFrames;

    bool updateLayout(BaseCamera* camera, float aspect);
    bool updateSh(GaussianData* renderingData, int offset);

};
}
