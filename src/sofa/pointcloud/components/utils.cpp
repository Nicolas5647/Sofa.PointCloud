#include <sofa/pointcloud/fwd.h>
#include <sofa/pointcloud/components/utils.h>

namespace sofa::pointcloud::components {
    namespace fs = std::filesystem;
    std::string readFile(fs::path path)
    {
        // Open the stream to 'lock' the file.
        std::ifstream f(path, std::ios::in | std::ios::binary);

        // Obtain the size of the file.
        const auto sz = fs::file_size(path);

        // Create a buffer.
        std::string result(sz, '\0');

        // Read the whole file into the buffer.
        f.read(result.data(), sz);

        return result;
    }


    void clear(GaussianData& data)
    {
        data.xyz.resize(0, data.xyz.cols());
        data.sh.resize(0, data.sh.cols());
        data.opacity.resize(0, data.opacity.cols());
        data.rot.resize(0, data.rot.cols());
        data.scale.resize(0, data.scale.cols());
    }


    void append(Eigen::MatrixXf& dest, const Eigen::MatrixXf& src)
    {
        assert(dest.cols() == src.cols());

        int oldRows = dest.rows();
        dest.conservativeResize(dest.rows() + src.rows(), src.cols());
        dest.block(oldRows, 0, src.rows(), src.cols()) = src;
    }

    template<int Size>
    void append(Eigen::Matrix<float, Eigen::Dynamic, Size, Eigen::RowMajor>& dest,
                const Eigen::Matrix<float, Eigen::Dynamic, Size, Eigen::RowMajor>& src)
    {
        assert(dest.cols() == src.cols());

        int oldRows = dest.rows();
        dest.conservativeResize(dest.rows() + src.rows(), src.cols());
        dest.block(oldRows, 0, src.rows(), src.cols()) = src;
    }

    void append(Eigen::Matrix<float, Eigen::Dynamic, 1>& dest,
                const Eigen::Matrix<float, Eigen::Dynamic, 1>& src)
    {
        assert(dest.cols() == src.cols());

        int oldRows = dest.rows();
        dest.conservativeResize(dest.rows() + src.rows(), src.cols());
        dest.block(oldRows, 0, src.rows(), src.cols()) = src;
    }


    void append(GaussianData& dest, const GaussianData& src)
    {
        append(dest.xyz, src.xyz);
        append(dest.sh, src.sh);
        append(dest.opacity, src.opacity);
        append(dest.scale, src.scale);
        append(dest.rot, src.rot);
    }

    std::vector<int> range(int maxSize)
    {
        std::vector<int> tmp {maxSize};
        for(auto i=0;i<maxSize;++i) tmp.emplace_back(i);
        return tmp;
    }


    // Retourne 6 plans : left, right, bottom, top, near, far
    std::array<Plane, 6> extractFrustumPlanes(
        const Eigen::Matrix4f& projMat,
        const Eigen::Matrix4f& viewModelMat)
    {
        std::array<Plane, 6> planes;

        Eigen::Matrix4f clipMat = projMat * viewModelMat; // colonne-major convention Eigen

        // Pour simplifier
        auto row = [&](int i){ return clipMat.row(i); };

        // Left
        planes[0].normal = (row(3) + row(0)).head<3>();
        planes[0].d = (row(3) + row(0))(3);
        // Right
        planes[1].normal = (row(3) - row(0)).head<3>();
        planes[1].d = (row(3) - row(0))(3);
        // Bottom
        planes[2].normal = (row(3) + row(1)).head<3>();
        planes[2].d = (row(3) + row(1))(3);
        // Top
        planes[3].normal = (row(3) - row(1)).head<3>();
        planes[3].d = (row(3) - row(1))(3);
        // Near
        planes[4].normal = (row(3) + row(2)).head<3>();
        planes[4].d = (row(3) + row(2))(3);
        // Far
        planes[5].normal = (row(3) - row(2)).head<3>();
        planes[5].d = (row(3) - row(2))(3);

        // Normalisation
        for(auto& p : planes){
            float len = p.normal.norm();
            p.normal /= len;
            p.d /= len;
        }

        return planes;
    }

    bool isSphereInsideFrustum(const std::array<Plane,6>& planes,
                            const Eigen::Vector3f& center,
                            float radius)
    {
        for(const auto& p : planes){
            if (p.normal.dot(center) + p.d + radius < 0.0f)
                return false; // complètement en dehors
        }
        return true; // intersecte ou à l’intérieur
    }

    void generatePlaneMesh(const Plane& plane, std::vector<sofa::type::Vec3>& outVertices)
    {
        Eigen::Vector3f pointOnPlane = plane.normal * (-plane.d / plane.normal.dot(plane.normal));
        sofa::type::Vec3 pt = sofa::type::Vec3(pointOnPlane.data());

        // Trouve deux vecteurs orthogonaux au plan
        Eigen::Vector3f u = plane.normal.cross(Eigen::Vector3f(0,1,0));
        if(u.norm() < 0.1f) u = plane.normal.cross(Eigen::Vector3f(1,0,0));
        u = u.normalized();
        Eigen::Vector3f v = plane.normal.cross(u).normalized();

        float size = 10.0f; // taille du quad

        // 4 coins
        outVertices.push_back(pt + sofa::type::Vec3(Eigen::Vector3f(size*(u+v)).data()));
        outVertices.push_back(pt + sofa::type::Vec3(Eigen::Vector3f(size*(u-v)).data()));
        outVertices.push_back(pt + sofa::type::Vec3(Eigen::Vector3f(size*(-u-v)).data()));
        outVertices.push_back(pt + sofa::type::Vec3(Eigen::Vector3f(size*(-u+v)).data()));
    }

    void erase(Eigen::MatrixXf& dest, int startRow, int rowCount)
    {
        if (rowCount <= 0) return;
        assert(startRow + rowCount <= dest.rows());

        int rowsToMove = dest.rows() - (startRow + rowCount);
        if (rowsToMove > 0) {
            dest.block(startRow, 0, rowsToMove, dest.cols()) = 
                dest.block(startRow + rowCount, 0, rowsToMove, dest.cols());
        }
        dest.conservativeResize(dest.rows() - rowCount, dest.cols());
    }

    template<int Size>
    void erase(Eigen::Matrix<float, Eigen::Dynamic, Size, Eigen::RowMajor>& dest, 
               int startRow, int rowCount)
    {
        if (rowCount <= 0) return;
        assert(startRow + rowCount <= dest.rows());

        int rowsToMove = dest.rows() - (startRow + rowCount);
        if (rowsToMove > 0) {
            dest.block(startRow, 0, rowsToMove, dest.cols()) = 
                dest.block(startRow + rowCount, 0, rowsToMove, dest.cols());
        }
        dest.conservativeResize(dest.rows() - rowCount, dest.cols());
    }

    void erase(Eigen::Matrix<float, Eigen::Dynamic, 1>& dest, 
               int startRow, int rowCount)
    {
        if (rowCount <= 0) return;
        assert(startRow + rowCount <= dest.rows());

        int rowsToMove = dest.rows() - (startRow + rowCount);
        if (rowsToMove > 0) {
            dest.block(startRow, 0, rowsToMove, dest.cols()) = 
                dest.block(startRow + rowCount, 0, rowsToMove, dest.cols());
        }
        dest.conservativeResize(dest.rows() - rowCount, dest.cols());
    }

    void erase(GaussianData& dest, int startRow, int rowCount)
    {
        erase(dest.xyz, startRow, rowCount);
        erase(dest.sh, startRow, rowCount);
        erase(dest.opacity, startRow, rowCount);
        erase(dest.scale, startRow, rowCount);
        erase(dest.rot, startRow, rowCount);
    }

}