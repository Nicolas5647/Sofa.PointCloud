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
#include <GL/gl.h>
#include <Eigen/Dense>
#include <sofa/core/ObjectFactory.h>

class BaseGLBuffer
{
public:
    //virtual void init(int ssboID) = 0;
    virtual void cleanup() = 0;
    virtual void map() = 0;
    virtual void unmap() = 0;

    virtual void getValueAsFloats(std::vector<float>& dest) = 0;
    virtual void getValueAsInts(std::vector<int>& dest) = 0;
};

struct Plane {
    Eigen::Vector3f normal; // (a,b,c)
    float d;                // d

    Plane(): normal(Eigen::Vector3f::Zero()), d(0) {}
    Plane(const Eigen::Vector3f& normal, float d) : normal(normal), d(d) {}

    static Eigen::Vector3f toEigenF(const sofa::type::Vec3& v) {
        return Eigen::Vector3f(static_cast<float>(v[0]), 
                                static_cast<float>(v[1]), 
                                static_cast<float>(v[2]));
    }

    Plane(const sofa::type::Vec3& n, const sofa::type::Vec3& p) {
        normal = toEigenF(n).normalized();
        d = normal.dot(toEigenF(p)); 
    }

    bool operator==(const Plane& other) const {
        return normal == other.normal && d == other.d;
    } 
};



class PointCloudRendererBackend
{
public:
    static bool hasCuda();

    template<class T> static BaseGLBuffer* createBuffer(GLuint ssboID);

    static int transform_and_sort_cuda(
            const std::array<Plane,6>& clipPlanes,
            const Eigen::Matrix4f&, BaseGLBuffer* positions,
                                        BaseGLBuffer* h_keys, BaseGLBuffer* h_values, int N);

    static void transform_and_sort_cpu(const Eigen::Matrix4f& P,
                                  const Eigen::MatrixXf& positions,
                                  std::vector<float>& depths,
                                  std::vector<int>& depth_indices);
};

