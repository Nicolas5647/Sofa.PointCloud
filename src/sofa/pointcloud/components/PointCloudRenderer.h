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
#include <sofa/pointcloud/components/PointCloudVisualModel.h>
#include <sofa/pointcloud/components/PointCloudContainer.h>
#include <sofa/pointcloud/components/PointCloudRendererBackend.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/simulation/Node.h>
#include <sofa/gl/GLSLShader.h>
#include <sofa/helper/SelectableItem.h>
#include <ostream>
#include <future>

namespace sofa::pointcloud::components
{
using sofa::core::objectmodel::BaseObject;
using sofa::component::visual::BaseCamera;

// References:
//  - https://huggingface.co/blog/gaussian-splatting
class PointCloudRenderer : public sofa::core::visual::VisualModel {
private:
    template<class T>
    using Link = core::objectmodel::SingleLink<PointCloudRenderer, T, core::objectmodel::BaseLink::FLAG_STOREPATH>;

public:
    SOFA_CLASS(PointCloudRenderer, BaseObject);

    PointCloudRenderer();
    ~PointCloudRenderer() override;

    Link<sofa::simulation::Node> l_targetNode;
    Link<BaseCamera> l_camera;

    Data<helper::OptionsGroup> d_renderMode;
    Data<bool> d_withCuda;

    void init() override;

    void doInitVisual(const sofa::core::visual::VisualParams* vparams) final;
    void doDrawVisual(const sofa::core::visual::VisualParams* vparams) final;
    void doUpdateVisual(const sofa::core::visual::VisualParams* vparams) final;
    
private:
    GLuint              _vao;
    GLuint              _vbo;

    enum SplatProperty
    {
        INDEX = 0,
        POSITION = 1,
        ROTATION = 2 ,
        SCALE = 3,
        OPACITY = 4,
        SPHERICAL_HARMONICS = 5,
        DEPTHS = 6
    };

    GLuint              _ssbo_splat[7];

    std::vector<float>  depths;
    gl::GLSLShader      shader;
    GaussianData        renderingData;

    BaseGLBuffer *interop_positions;
    BaseGLBuffer *interop_depths;
    BaseGLBuffer *interop_indices;

    std::future<void> a1;
    std::vector<int> indices;
    std::map<unsigned int, std::vector<int>> directionalIndices;

    std::map<PointCloudVisualModel*, std::tuple<int, int, int, int, bool>> dataCache;

    std::array<Plane,6 > clipPlanes;

    void transform(float uniformScale,
                   const std::vector<defaulttype::Rigid3Types::Coord>& initFrames,
                   const std::vector<defaulttype::Rigid3Types::Coord>& frames,
                   const std::vector<defaulttype::Rigid3Types::Coord>& frames2,
                   const std::vector<std::vector<int>>& frameIndices,
                   const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& srcPositions,
                   Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& dstPositions,
                   const Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor> &srcOrientations,
                   Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor> &dstOrientations,
                   Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> &scales, int offset);

    void sort(const Eigen::Matrix4f& P,
              const Eigen::MatrixXf& positions,
              std::vector<float>& depths,
              std::vector<int> &depth_indices);

    void transform_and_sort(const Eigen::Matrix4f& P,
                            BaseGLBuffer* position,
                            BaseGLBuffer* depth,
                            BaseGLBuffer* indices, int N);
};
}
