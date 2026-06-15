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
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <fstream>
#include <sofa/pointcloud/components/liteviz-dataloader.h>

namespace sofa::core::objectmodel
{


/// Specialization for reading strings
template<>
bool Data<Eigen::MatrixXf>::read( const std::string& str );

template<>
std::string Data<Eigen::MatrixXf>::getValueString() const;

}

namespace sofa::pointcloud::components
{
using sofa::core::objectmodel::DataFileName;
using sofa::core::objectmodel::BaseObject;

class PointCloudContainer : public BaseObject {
public:
    SOFA_CLASS(PointCloudContainer, BaseObject);

    PointCloudContainer();
    ~PointCloudContainer();

    DataFileName d_filename;
    Data<bool> d_printLodding;

    void init() override;
    void updateBBox();

    bool load(const std::string& filename, int max_sh_degree = 3);
    void loadNaive();

    size_t size();

    void updateDataFields();

    GaussianData*    data{nullptr};

    Data<Eigen::MatrixXf> d_positions;
    Data<Eigen::MatrixXf> d_orientations;
    Data<Eigen::MatrixXf> d_scales;
    Data<Eigen::MatrixXf> d_opacities;
    Data<Eigen::MatrixXf> d_sphericalHarmonics;

    Data<type::vector<int>> d_indices;

    size_t size();
    
 private:
};


}
