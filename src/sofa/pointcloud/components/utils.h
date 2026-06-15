#pragma once
#include <sofa/pointcloud/config.h>
#include <sofa/pointcloud/components/PointCloudRendererBackend.h>
#include <sofa/pointcloud/components/liteviz-dataloader.h>
#include <sofa/core/ObjectFactory.h>

#include <fstream>
#include <filesystem>

namespace sofa::pointcloud::components {

    namespace fs = std::filesystem;
    std::string readFile(fs::path path);
    void clear(GaussianData& data);
    void append(Eigen::MatrixXf& dest, const Eigen::MatrixXf& src);

    template<int Size>
    void append(Eigen::Matrix<float, Eigen::Dynamic, Size, Eigen::RowMajor>& dest,
                const Eigen::Matrix<float, Eigen::Dynamic, Size, Eigen::RowMajor>& src);

    void append(Eigen::Matrix<float, Eigen::Dynamic, 1>& dest,
                const Eigen::Matrix<float, Eigen::Dynamic, 1>& src);


    void append(GaussianData& dest, const GaussianData& src);
    std::vector<int> range(int maxSize);

    // Retourne 6 plans : left, right, bottom, top, near, far
    std::array<Plane, 6> extractFrustumPlanes(
        const Eigen::Matrix4f& projMat,
        const Eigen::Matrix4f& viewModelMat);

    bool isSphereInsideFrustum(const std::array<Plane,6>& planes,
                            const Eigen::Vector3f& center,
                            float radius);

    void generatePlaneMesh(const Plane& plane, std::vector<sofa::type::Vec3>& outVertices);

    void erase(Eigen::MatrixXf& dest, int startRow, int rowCount);
    template<int Size>
    void erase(Eigen::Matrix<float, Eigen::Dynamic, Size, Eigen::RowMajor>& dest, 
               int startRow, int rowCount);
    void erase(Eigen::Matrix<float, Eigen::Dynamic, 1>& dest, 
               int startRow, int rowCount);
    void erase(GaussianData& dest, int startRow, int rowCount);

}