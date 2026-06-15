#include <sofa/pointcloud/components/PointCloudRendererBackend.h>
#include <tbb/tbb.h>


template<> BaseGLBuffer* PointCloudRendererBackend::createBuffer<int>(GLuint ssboID){
    
};
template<> BaseGLBuffer* PointCloudRendererBackend::createBuffer<float>(GLuint ssboID){
    
};
int PointCloudRendererBackend::transform_and_sort_cuda(
            const std::array<Plane,6>& clipPlanes,
            const Eigen::Matrix4f&, BaseGLBuffer* positions,
                                        BaseGLBuffer* h_keys, BaseGLBuffer* h_values, int N){}
bool PointCloudRendererBackend::hasCuda(){ return false; }


void PointCloudRendererBackend::transform_and_sort_cpu(const Eigen::Matrix4f& P,
                                                       const Eigen::MatrixXf& positions,
                                                       std::vector<float>& depths,
                                                       std::vector<int>& indices)
{
    const Eigen::RowVector3f proj_row = P.row(2).head<3>();

    for(auto index : indices)
    {
        Eigen::Vector3f center = (positions.row(index).transpose());
        depths[index] = proj_row.dot(center);
    }

    tbb::parallel_sort(indices.begin(), indices.end(),
                       [&](int i, int j) {
        return depths[i] < depths[j];
    });
}
