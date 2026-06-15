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

#include <sofa/pointcloud/components/PointCloudTransform.h>
#include <sofa/core/ObjectFactory.h>

#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudTransform>(sofa::core::ObjectFactory* factory){
    factory->registerObjects(sofa::core::ObjectRegistrationData("Transform a point cloud in a new one.")
                             .add< sofa::pointcloud::components::PointCloudTransform >());
}

}

namespace sofa::pointcloud::components
{

PointCloudTransform::PointCloudTransform() :
     l_input(initLink("input", "a point cloud container"))
    ,l_output(initLink("output", "a second cloud container"))
    ,d_frames(initData(&d_frames, "frames", "a frame to control the geometry"))
    ,d_frameIndices(initData(&d_frameIndices, "frameIndices", "ddd"))
    ,d_scales(initData(&d_scales, {1}, "scales", "a scaling factor"))
{
    d_frames.setValue({defaulttype::Rigid3Types::Coord{{0.0,0.0,0.0},{0.0,0.0,0.0,1.0}}});
}

PointCloudTransform::~PointCloudTransform()
{
}

void PointCloudTransform::init()
{

    std::cout << "INIT " << std::endl;
    Inherit1::init();
    update();
    d_componentState = core::objectmodel::ComponentState::Valid;

    addUpdateCallback("update", {&d_frames, &d_scales}, [this](const sofa::core::DataTracker&){

        std::cout << "CALLBACK " << std::endl;
        update();
        return sofa::core::objectmodel::ComponentState::Valid;
    }, {&l_output->d_componentState});
}

void PointCloudTransform::clear(GaussianData& data)
{
    data.xyz.resize(0, data.xyz.cols());
    data.sh.resize(0, data.sh.cols());
    data.opacity.resize(0, data.opacity.cols());
    data.rot.resize(0, data.rot.cols());
    data.scale.resize(0, data.scale.cols());
}

void PointCloudTransform::append(Eigen::MatrixXf& dest, const Eigen::MatrixXf& src)
{
    assert(dest.cols() == src.cols());

    int oldRows = dest.rows();
    dest.conservativeResize(dest.rows() + src.rows(), src.cols());
    dest.block(oldRows, 0, src.rows(), src.cols()) = src;
}

template<unsigned int Size>
void PointCloudTransform::append(Eigen::Matrix<float, Eigen::Dynamic, Size, Eigen::RowMajor>& dest,
                                 const Eigen::Matrix<float, Eigen::Dynamic, Size, Eigen::RowMajor>& src)
{
    assert(dest.cols() == src.cols());

    int oldRows = dest.rows();
    dest.conservativeResize(dest.rows() + src.rows(), src.cols());
    dest.block(oldRows, 0, src.rows(), src.cols()) = src;
}

void PointCloudTransform::append(Eigen::Matrix<float, Eigen::Dynamic, 1>& dest,
                                 const Eigen::Matrix<float, Eigen::Dynamic, 1>& src)
{    
    assert(dest.cols() == src.cols());


    int oldRows = dest.rows();
    dest.conservativeResize(dest.rows() + src.rows(), src.cols());
    dest.block(oldRows, 0, src.rows(), src.cols()) = src;
}


void PointCloudTransform::append(GaussianData& dest, const GaussianData& src)
{
    append<3>(dest.xyz, src.xyz);
    append(dest.sh, src.sh);
    append(dest.opacity, src.opacity);
    append<3>(dest.scale, src.scale);
    append<4>(dest.rot, src.rot);
}

void PointCloudTransform::update()
{
    if(l_input->isComponentStateInvalid())
    {
        return;
    }

    if(l_output->isComponentStateInvalid())
    {
        return;
    }


    auto source = l_input->data;
    auto destination = l_output->data;
    clear(*(l_output->data));
    append(*(l_output->data), *(l_input->data));

    // Compute the rigid transform from frames.
    auto frames = d_frames.getValue();
    auto scales = d_scales.getValue();
    auto frameIndices = helper::getWriteAccessor(d_frameIndices);
    if(frameIndices.size()!=source->size())
    {
        frameIndices.resize(source->size(), 0);
    }

    std::vector<Eigen::Translation<float,3>::AffineTransformType> t_transforms{frames.size()};
    std::vector<Eigen::Quaternion<float>> t_rotations {frames.size()};
    std::vector<Eigen::UniformScaling<float>> t_scales {frames.size()};

    for(size_t frameIndex=0;frameIndex<frames.size();++frameIndex)
    {
        auto center = frames[frameIndex].getCenter();
        auto orientation = frames[frameIndex].getOrientation();
        auto scale = scales[frameIndex];
        auto T = Eigen::Translation<float,3>(center.x(), center.y(), center.z());
        auto R = Eigen::Quaternion<float>(orientation[3], orientation[0], orientation[1], orientation[2]);
        auto S = Eigen::UniformScaling<float>(scale);
        auto RST = T*R*S;
        t_transforms[frameIndex] = RST;
        t_rotations[frameIndex] = R;
        t_scales[frameIndex] = S;
    }

    for(size_t vtxIndex=0;vtxIndex<source->xyz.rows(); ++vtxIndex)
    {
        auto TRS = t_transforms[frameIndices[vtxIndex]];
        auto R = t_rotations[frameIndices[vtxIndex]];
        auto S = t_scales[frameIndices[vtxIndex]];

        Eigen::Vector3f worldPosition = ((source->xyz.row(vtxIndex).transpose()));
        destination->xyz.row(vtxIndex) = TRS * worldPosition;

        auto tmp = source->rot.row(vtxIndex);
        Eigen::Quaternionf worldOrientation = R * Eigen::Quaternionf{tmp(0), tmp(1), tmp(2), tmp(3)};
        destination->rot.row(vtxIndex)(0) = (worldOrientation).w();
        destination->rot.row(vtxIndex)(1) = (worldOrientation).x();
        destination->rot.row(vtxIndex)(2) = (worldOrientation).y();
        destination->rot.row(vtxIndex)(3) = (worldOrientation).z();

        destination->scale.row(vtxIndex) = source->scale.row(vtxIndex)*S;
    }

    l_output->updateDataFields();
}


}
