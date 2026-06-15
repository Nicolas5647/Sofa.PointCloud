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
#include <sofa/pointcloud/components/PointCloudRenderer.h>
#include <sofa/pointcloud/components/PointCloudVisualModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/pointcloud/components/utils.h>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <Eigen/Dense>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/pointcloud/components/PointCloudRendererBackend.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <iostream>
#include <vector>

namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudRenderer>(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A point cloud renderer.")
                                 .add< sofa::pointcloud::components::PointCloudRenderer >());
}

}

namespace sofa::pointcloud::components
{

PointCloudRenderer::PointCloudRenderer() :
    l_targetNode(initLink("targetNode", "Link to the node to start rendering from"))
    , l_camera(initLink("camera", "link to the camera used to render"))
    , d_renderMode(initData(&d_renderMode, "color_sh_0", "renderMode", "Visualization method, select the spherical harmonics, gaussian or depth values"))
    , d_withCuda(initData(&d_withCuda, true, "withCuda", "Use the CUDA based rendering of point cloud for high performance, default=true"))
{
    helper::OptionsGroup methodOptions{"COLOR_SH_0",
                                       "COLOR_SH_1",
                                       "COLOR_SH_2",
                                       "COLOR_SH_3",
                                       "DEPTH",
                                       "GAUSS_BALL",
                                       "SURFEL"};
    methodOptions.setSelectedItem(0);
    d_renderMode.setValue(methodOptions);
}


PointCloudRenderer::~PointCloudRenderer() {}

void PointCloudRenderer::init()
{
    Inherit1::init();
    if(!l_targetNode)
    {
        l_targetNode.set(dynamic_cast<sofa::simulation::Node*>(getContext()));
    }
}

void PointCloudRenderer::doInitVisual(const sofa::core::visual::VisualParams* vparams)
{
    if (!sofa::gl::GLSLShader::InitGLSL())
    {
        msg_info() << "InitGLSL failed" ;
        return;
    }

    auto vtx = helper::system::DataRepository.getFile("shaders/gaussian_splatting.vert");
    auto fg = helper::system::DataRepository.getFile("shaders/gaussian_splatting.frag");
    auto geom = helper::system::DataRepository.getFile("shaders/gaussian_splatting.geom");

    shader.SetVertexShaderFileName(vtx);
    shader.SetFragmentShaderFileName(fg);
    shader.SetGeometryShaderFileName(geom);
    shader.InitShaders();

    glGenVertexArrays(1, &_vao);
    glGenBuffers(1, &_vbo);
    glBindVertexArray(_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);

    glGenBuffers(7, _ssbo_splat);
    for(auto i = 0;i<6;i++)
    {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[i]);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, i, _ssbo_splat[i]);
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    // Initialize the interop-buffer
    interop_positions = PointCloudRendererBackend::createBuffer<float>(_ssbo_splat[SplatProperty::POSITION]);
    interop_depths = PointCloudRendererBackend::createBuffer<float>(_ssbo_splat[SplatProperty::DEPTHS]);
    interop_indices = PointCloudRendererBackend::createBuffer<int>(_ssbo_splat[SplatProperty::INDEX]);

    if(!PointCloudRendererBackend::hasCuda()){
        d_withCuda.setValue(false);
        msg_warning() << "CUDA is not detected, falling back to pure (low performance) opengl renderer.";
    }else{
        msg_info() << "CUDA detected.";
    }
}

void PointCloudRenderer::doUpdateVisual(const sofa::core::visual::VisualParams* vparams)
{
    SOFA_UNUSED(vparams);
}


void PointCloudRenderer::transform(float scale,
                                   const std::vector<defaulttype::Rigid3Types::Coord>& globalToLocalFrames,
                                   const std::vector<defaulttype::Rigid3Types::Coord>& initFrames,
                                   const std::vector<defaulttype::Rigid3Types::Coord>& currentFrames,
                                   const std::vector<std::vector<int>>& frameIndices,
                                   const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& srcPositions,
                                   Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& dstPositions,
                                   const Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor>& srcOrientations,
                                   Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor>& dstOrientations,
                                   Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& scales, int offset)
{
    for(size_t frameIndex=0;frameIndex<frameIndices.size();++frameIndex)
    {
        auto frameCenter = currentFrames[frameIndex].getCenter(); // + globalToLocalFrames[frameIndex].getCenter();
        auto frameInitCenter = initFrames[frameIndex].getCenter(); // + globalToLocalFrames[frameIndex].getCenter();
        auto frameOrientation = currentFrames[frameIndex].getOrientation(); // + globalToLocalFrames[frameIndex].getOrientation();

        auto T = Eigen::Translation<float,3>(frameCenter.x(), frameCenter.y(), frameCenter.z());
        auto Tinv = Eigen::Translation<float,3>(-frameInitCenter.x(), -frameInitCenter.y(), -frameInitCenter.z());
        auto R = Eigen::Quaternion<float>(frameOrientation[3], frameOrientation[0], frameOrientation[1], frameOrientation[2]);
        auto S = Eigen::UniformScaling<float>(scale);

        auto transform = T * R;
        for(size_t i=0;i<frameIndices[frameIndex].size(); ++i)
        {
            auto vtxIndex = frameIndices[frameIndex][i];

            Eigen::Vector3f worldPosition = srcPositions.row(vtxIndex).transpose();
            dstPositions.row(vtxIndex+offset) = transform * (Tinv * worldPosition);

            auto tmp = srcOrientations.row(vtxIndex);
            Eigen::Quaternionf worldOrientation = R * Eigen::Quaternionf{tmp(0), tmp(1), tmp(2), tmp(3)};

            dstOrientations.row(vtxIndex+offset)(0) = (worldOrientation).w();
            dstOrientations.row(vtxIndex+offset)(1) = (worldOrientation).x();
            dstOrientations.row(vtxIndex+offset)(2) = (worldOrientation).y();
            dstOrientations.row(vtxIndex+offset)(3) = (worldOrientation).z();

            //scales.row(vtxIndex) = scales.row(vtxIndex)*scale;
        }
    }
}


void PointCloudRenderer::doDrawVisual(const sofa::core::visual::VisualParams* vparams)
{
    SCOPED_TIMER("PointCloud::doDrawVisual");

    auto viewport = vparams->viewport();
    
    Eigen::Matrix4f projmat;
    Eigen::Matrix4f viewmat;
    Eigen::Matrix4d dprojmat;
    Eigen::Matrix4d dviewmat;

    // If there is no camera, then we cannot draw the scene.
    if(!l_camera)
        return;

    // Aggregate the geometries by traversing the scene tree
    auto visualModels = l_targetNode->getTreeObjects<sofa::pointcloud::components::PointCloudVisualModel>();
    msg_info() << "Found " << visualModels.size() << " gaussian splats visual models.";

    bool indiceChanged = false;
    bool updateSh = false;
    float aspect;
    std::vector<std::tuple<int,int>> updatesBufferParts;
    {

        SCOPED_TIMER("PointCloud::doDrawVisual::sceneParsing");
        for(auto visual : visualModels)
        {
            visual->updateVisual(vparams);
            if(!visual->isComponentStateValid()){
                continue;
            }

            if(!visual->data){
                continue;
            }

            aspect = static_cast<float>(viewport[2]) / viewport[3];
            bool updateIndices = visual->updateLayout(l_camera, aspect);
            visual->draw(vparams);


            auto scale = visual->d_uniformScale.getValue();
            auto& referenceFrames = visual->d_initFrames.getValue();
            auto& localToGlobalFrames = visual->localToGlobalFrames;

            auto frames = helper::getReadAccessor(visual->d_frames);
            auto frameIndices = helper::getReadAccessor(visual->d_frameIndices);

            if(!dataCache.contains(visual)){
                int offset = renderingData.xyz.rows();
                int beginIndex = renderingData.size();
                int size = visual->data->xyz.rows()*(3+4+3+1+visual->data->sh_dim());
                bool enable = visual->d_enable.getValue();
                int firstIndex;
                int lastIndex;

                append(renderingData.xyz, visual->data->xyz);
                append(renderingData.sh, visual->data->sh);
                append(renderingData.opacity, visual->data->opacity);
                append(renderingData.scale, visual->data->scale);
                append(renderingData.rot, visual->data->rot);

                if (enable) {
                    auto visual_indices = helper::getReadAccessor(visual->d_indices);
                    for(int i=0;i<(int)visual_indices.size();i++)
                    {
                        indices.push_back(offset + visual_indices[i]);
                    }
                    firstIndex = indices.size() - visual_indices.size();
                    lastIndex = indices.size();
                }

                std::vector<std::vector<int>> frameMap{frames.size()};
                for(size_t i=0;i<frameIndices.size();++i)
                {
                    frameMap[frameIndices[i]].push_back(i);
                }

                // Now we need to apply the transformation
                this->transform(scale,
                                localToGlobalFrames, referenceFrames,
                                frames, frameMap,
                                visual->data->xyz, renderingData.xyz,
                                visual->data->rot, renderingData.rot,
                                renderingData.scale, offset);

                msg_info() << "Batching a new data set " << visual->getPathName() << " with frames " << frames.size() << msgendl
                           << "         data set offset & size " << beginIndex << ", " << size;
                updatesBufferParts.push_back({offset,size});
                dataCache[visual] = std::make_tuple(beginIndex, size, firstIndex, lastIndex, enable);
                indiceChanged = true;
                continue;
            } else if (std::get<4>(dataCache[visual]) != visual->d_enable.getValue()) {
                SCOPED_TIMER("PointCloud::doDrawVisual::updateEnable");
                auto& cachedData = dataCache[visual];
                std::get<4>(cachedData) = visual->d_enable.getValue();
                auto [offset, size, firstIndex, lastIndex, enable] = cachedData;
                int visual_size = visual->d_indices.getValue().size();
                if (enable) {
                    for(int i=offset; i < offset + visual_size; ++i)
                    {
                        indices.push_back(i);
                    }
                    std::get<2>(cachedData) = indices.size() - visual_size;
                    std::get<3>(cachedData) = indices.size();
                } else {
                    indices.erase(indices.begin() + firstIndex, indices.begin() + lastIndex);
                }
                indiceChanged = true;
            }
            
            auto [offset, size, firstIndex, lastIndex, enable] = dataCache[visual];
            updateSh = visual->updateSh(&renderingData, offset);
            if(visual->d_isStaticModel.getValue() || !enable)
                continue;

            if (updateIndices) {
                SCOPED_TIMER("PointCloud::doDrawVisual::updateIndices");

                auto localIndices = helper::getReadAccessor(visual->d_indices);
                const size_t newSize = localIndices.size();
                const size_t oldSize = lastIndex - firstIndex;
                const ptrdiff_t diff = static_cast<ptrdiff_t>(newSize) - static_cast<ptrdiff_t>(oldSize);

                if (diff == 0) {
                    for (size_t i = 0; i < newSize; ++i) {
                        indices[firstIndex + i] = offset + localIndices[i];
                    }
                } 
                else if (diff < 0) {
                    for (size_t i = 0; i < newSize; ++i) {
                        indices[firstIndex + i] = offset + localIndices[i];
                    }
                    indices.erase(indices.begin() + firstIndex + newSize, indices.begin() + lastIndex);
                } 
                else {
                    for (size_t i = 0; i < oldSize; ++i) {
                        indices[firstIndex + i] = offset + localIndices[i];
                    }
                    
                    indices.insert(indices.begin() + lastIndex, diff, 0);
                    for (size_t i = oldSize; i < newSize; ++i) {
                        indices[firstIndex + i] = offset + localIndices[i];
                    }
                }

                if (diff != 0) {
                    for (auto& v : visualModels) {
                        if (v == visual || std::get<3>(dataCache[v]) < firstIndex) continue;
                        
                        if (diff > 0) {
                            std::get<2>(dataCache[v]) += static_cast<size_t>(diff);
                            std::get<3>(dataCache[v]) += static_cast<size_t>(diff);
                        } else {
                            std::get<2>(dataCache[v]) -= static_cast<size_t>(-diff);
                            std::get<3>(dataCache[v]) -= static_cast<size_t>(-diff);
                        }
                    }
                }

                std::get<2>(dataCache[visual]) = firstIndex;
                std::get<3>(dataCache[visual]) = firstIndex + newSize;

                indiceChanged = true;
            }

            std::vector<std::vector<int>> frameMap{frames.size()};
            for(size_t i=0;i<frameIndices.size();++i)
            {
                frameMap[frameIndices[i]].push_back(i);
            }

            {
                SCOPED_TIMER("PointCloud::doDrawVisual::sceneTransform");
                // Now we need to apply the transformation
                this->transform(scale,
                                localToGlobalFrames,
                                referenceFrames, frames,
                                frameMap,
                                visual->data->xyz, renderingData.xyz,
                                visual->data->rot, renderingData.rot,
                                renderingData.scale, offset);
            }
            msg_info() << "Updating point clouds from " << visual->getPathName() << " with frames " << frames.size() << msgendl
                       << "         data set offset & size " << offset << ", " << size;
            updatesBufferParts.push_back({offset,size});
        }
    }
    msg_info() << "  total number of splats to render: " << renderingData.size() << " splats ";

    // In case there is not rendering data, then just exit
    if(renderingData.size()==0)
        return;

    // Here we have geometries to draw and a camera that look at it.
    // We first send the camera parameters to the gl rendering backend, then the geometries.
    auto c = l_camera->getPosition();
    Eigen::Vector3f cam_pos {(float)c[0],(float)c[1],(float)c[2]};

    vparams->getModelViewMatrix(dviewmat.data());
    vparams->getProjectionMatrix(dprojmat.data());

    projmat = dprojmat.cast<float>();
    viewmat = dviewmat.cast<float>();

    float fov = l_camera->getFieldOfView();
    float tanHalfFov = tan((fov / 180.0 * M_PI) / 2.0f);
    Eigen::Vector2f tanxy (aspect * tanHalfFov, tanHalfFov);
    float focal = static_cast<float>(viewport[3]) / (tan((fov / 180.0f * M_PI) / 2.0f) * 2.0f);

    auto mode = d_renderMode.getValue().getSelectedId();

    clipPlanes = extractFrustumPlanes(projmat, viewmat);

    defaulttype::Rigid3Types::Coord type;
    {
        SCOPED_TIMER("PointCloud::doDrawVisual::renderingSetupUp");
        vparams->drawTool()->pushMatrix();
        float glTransform[16];
        type.writeOpenGlMatrix ( glTransform );
        Eigen::Matrix4f transform {glTransform };

        shader.TurnOn();
        int max_sh_dim = 48;

        shader.SetMatrix4(shader.GetVariable("projmat"), 1, GL_FALSE, projmat.data());
        shader.SetMatrix4(shader.GetVariable("viewmat"), 1, GL_FALSE, viewmat.data());
        shader.SetFloatVector3(shader.GetVariable("cam_pos"), 1, cam_pos.data());
        shader.SetFloatVector2(shader.GetVariable("tanxy"), 1, tanxy.data());
        shader.SetFloat(shader.GetVariable("focal"), focal);
        shader.SetInt(shader.GetVariable("max_sh_dim"), max_sh_dim);
        shader.SetInt(shader.GetVariable("render_mod"), mode);
    }

    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    static bool firstTime = true;
    if(firstTime)
    {
        SCOPED_TIMER("PointCloud::doDrawVisual::shaderSetup");
        firstTime = false;
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::SCALE]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, renderingData.scale.rows() * sizeof(float) * 3, renderingData.scale.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, _ssbo_splat[SplatProperty::SCALE]);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::OPACITY]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, renderingData.opacity.rows() * sizeof(float), renderingData.opacity.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, _ssbo_splat[SplatProperty::OPACITY]);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        auto buffer = renderingData.flat_sh();
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::SPHERICAL_HARMONICS]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, buffer.size()*sizeof(float), buffer.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, _ssbo_splat[SplatProperty::SPHERICAL_HARMONICS]);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::POSITION]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, renderingData.xyz.rows() * sizeof(float) * 3, renderingData.xyz.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, _ssbo_splat[SplatProperty::POSITION]);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::ROTATION]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, renderingData.rot.rows() * sizeof(float) * 4, renderingData.rot.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, _ssbo_splat[SplatProperty::ROTATION]);

        depths.resize(renderingData.xyz.rows());
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::DEPTHS]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, depths.size() * sizeof(float), depths.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, _ssbo_splat[SplatProperty::DEPTHS]);
    }

    if(updatesBufferParts.size())
    {
        SCOPED_TIMER("PointCloud::doDrawVisual::bufferUpdate POSITION");
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::POSITION]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, renderingData.xyz.rows() * sizeof(float) * 3, renderingData.xyz.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, _ssbo_splat[SplatProperty::POSITION]);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::ROTATION]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, renderingData.rot.rows() * sizeof(float) * 4, renderingData.rot.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, _ssbo_splat[SplatProperty::ROTATION]);
    }

    if (updateSh) {
        SCOPED_TIMER("PointCloud::doDrawVisual::bufferUpdate SPHERICAL HARMONICS");

        auto buffer = renderingData.flat_sh();
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::SPHERICAL_HARMONICS]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, buffer.size()*sizeof(float), buffer.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, _ssbo_splat[SplatProperty::SPHERICAL_HARMONICS]);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    if(indiceChanged) {
        SCOPED_TIMER("PointCloud::doDrawVisual::bufferUpdate INDEX");
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::INDEX]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, indices.size() * sizeof(int), indices.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _ssbo_splat[SplatProperty::INDEX]);
    }

    int splatsToDraw = 0;
    {
        SCOPED_TIMER("PointCloud::doDrawVisual::splatSorting");

        if(d_withCuda.getValue())
        {
            splatsToDraw = PointCloudRendererBackend::transform_and_sort_cuda(
                clipPlanes,
                viewmat, interop_positions, interop_depths, interop_indices, renderingData.size());
        }else
        {
            std::vector<int> selectedIndices;
            {

                SCOPED_TIMER("PointCloud::doDrawVisual::planeClipping");
                // Plane clipping
                for(auto indice : indices )
                {
                    if(isSphereInsideFrustum(clipPlanes, renderingData.xyz.row(indice), 0.05))
                    {
                        selectedIndices.push_back(indice);
                    }
                }
            }
            PointCloudRendererBackend::transform_and_sort_cpu(viewmat, renderingData.xyz,
                                                              depths, selectedIndices);

            glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat[SplatProperty::INDEX]);
            glBufferData(GL_SHADER_STORAGE_BUFFER,  selectedIndices.size() * sizeof(int),  selectedIndices.data(), GL_DYNAMIC_DRAW);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _ssbo_splat[SplatProperty::INDEX]);
            splatsToDraw=selectedIndices.size();
        }
    }

    {
        SCOPED_TIMER("PointCloud::doDrawVisual::splatRendering");
        glBindBuffer(GL_ARRAY_BUFFER, _vbo);
        glBindVertexArray(_vao);

        glDrawArraysInstanced(GL_POINTS, 0, 1, splatsToDraw);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
    msg_info() << "Rendering "<< splatsToDraw << "/" << renderingData.size() << " splats.";

    shader.TurnOff();

    glEnable(GL_CULL_FACE);

    vparams->drawTool()->popMatrix();
}


}
