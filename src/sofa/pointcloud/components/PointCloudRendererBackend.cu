#include <sofa/pointcloud/components/PointCloudRendererBackend.h>
#include <tbb/tbb.h>

#ifdef SOFA_POINTCLOUD_USE_CUDA
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/remove.h>
#include <cuda_gl_interop.h>

#define CUDA_CHECK(call) do { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
    fprintf(stderr, "CUDA Error %s:%d: %s\n", __FILE__, __LINE__, cudaGetErrorString(err)); \
    } \
    } while(0)

template<class T>
class CudaGLBuffer : public BaseGLBuffer
{
public:
    GLuint glSSBO {0};
    cudaGraphicsResource* cudaBuffer{nullptr};

    CudaGLBuffer(GLuint ssboID)
    {
        glSSBO = ssboID;
    }

    void cleanup() override
    {
        CUDA_CHECK(cudaGraphicsUnregisterResource(cudaBuffer));
    }

    void map() override
    {
        if(cudaBuffer==nullptr)
            CUDA_CHECK(cudaGraphicsGLRegisterBuffer(&cudaBuffer, glSSBO, cudaGraphicsRegisterFlagsWriteDiscard));
        CUDA_CHECK(cudaGraphicsMapResources(1, &cudaBuffer));
    }

    void unmap() override
    {
        CUDA_CHECK(cudaGraphicsUnmapResources(1, &cudaBuffer));
    }

    // Returns the ssbo id & size in byte
    std::tuple<T*, size_t> getSSBOAndSize()
    {
        T* ssboPtr = nullptr;
        size_t ssboSize = 0;
        CUDA_CHECK(cudaGraphicsResourceGetMappedPointer((void**)&ssboPtr, &ssboSize, cudaBuffer));

        return {ssboPtr, ssboSize};
    }

    void getValueAsFloats(std::vector<float>& values)
    {
        auto [ssbo, size] = getSSBOAndSize();
        values.resize(size/sizeof(float));
        CUDA_CHECK(cudaMemcpy(values.data(), ssbo, size, cudaMemcpyDeviceToHost));
    }

    void getValueAsInts(std::vector<int>& values)
    {
        auto [ssbo, size] = getSSBOAndSize();
        values.resize(size/sizeof(int));
        CUDA_CHECK(cudaMemcpy(values.data(), ssbo, size, cudaMemcpyDeviceToHost));
    }
};

template<> BaseGLBuffer* PointCloudRendererBackend::createBuffer<int>(GLuint ssboID)
{
    return new CudaGLBuffer<int>(ssboID);
}

template<> BaseGLBuffer* PointCloudRendererBackend::createBuffer<float>(GLuint ssboID)
{
    return new CudaGLBuffer<float>(ssboID);
}

struct vec3 {
    float x, y, z;

    __device__ inline float dot(const vec3& b) const { return x*b.x + y*b.y + z*b.z; }
};
struct mat4 { float m[16]; }; // row-major
struct plane {
    vec3 normal;
    float d;
};

void project_to_z(const std::array<Plane, 6>& clipPlanes,
                  const Eigen::Matrix4f&,
                  const vec3* positions,
                  float* depthPtr, int* indicesPtr, const int N);
void sort_float_int(CudaGLBuffer<float>& values, CudaGLBuffer<int>& indices);

struct remove_if_second_is_minus_one
{
    __host__ __device__
    bool operator()(const thrust::tuple<const float&, const int&>& t) const
    {
        return thrust::get<1>(t) == -1;
    }
};

int PointCloudRendererBackend::transform_and_sort_cuda(
        const std::array<Plane, 6>& clipPlanes,
        const Eigen::Matrix4f& mvp,
        BaseGLBuffer* positions_, BaseGLBuffer* depths_, BaseGLBuffer* indices_, int N)
{

    auto positions = dynamic_cast<CudaGLBuffer<float>*>(positions_);
    auto depths = dynamic_cast<CudaGLBuffer<float>*>(depths_);
    auto indices = dynamic_cast<CudaGLBuffer<int>*>(indices_);

    assert(positions!=nullptr);
    assert(depths!=nullptr);
    assert(indices!=nullptr);

    glFinish();

    positions->map();
    indices->map();
    depths->map();

    auto [indices_ptr, indices_size] = indices->getSSBOAndSize();
    auto [depths_ptr, depths_size] = depths->getSSBOAndSize();
    auto [positions_ptr, positions_size] = positions->getSSBOAndSize();

    assert(positions_size/(3*sizeof(float)) == N);
    assert(depths_size/sizeof(float) == N);
    assert(indices_size/sizeof(int) == N);

    thrust::device_ptr<int> indices_t{indices_ptr};
    thrust::device_ptr<float> positions_t{positions_ptr};
    thrust::device_ptr<float> depths_t{depths_ptr};

    // Starts a kernel to quickly exclude some splats and compute depth on the remaining.
    project_to_z(clipPlanes,
                 mvp,
                 (vec3*)positions_ptr,
                 depths_ptr, indices_ptr, N);


    auto zipped_begin = thrust::make_zip_iterator(thrust::make_tuple(depths_t, indices_t));
    auto zipped_end = zipped_begin + N;

    auto new_end = thrust::remove_if(
        thrust::device,
        zipped_begin,
        zipped_end,
        remove_if_second_is_minus_one()
    );

    size_t splatToSort = new_end - zipped_begin;

    if(splatToSort >= 2)
    {
        // Tri : keys → triées, values → réarrangées dans le même ordre
        thrust::sort_by_key(depths_t, depths_t+splatToSort, indices_t);
    }

    positions->unmap();
    depths->unmap();
    indices->unmap();

    return splatToSort;
}



__device__ float compute_depth(const vec3& pos, const float* mvp)
{
    const mat4* proj = (const mat4*)mvp;

    // Transforme le vertex en vec4
    float x = pos.x;
    float y = pos.y;
    float z = pos.z;

    // ligne 2 de P, colonnes 0..2
    float proj0 = proj->m[0*4 + 2];
    float proj1 = proj->m[1*4 + 2];
    float proj2 = proj->m[2*4 + 2];

    // produit depth scalaire
    return (proj0 * x + proj1 * y + proj2 * z);
}

__device__ bool isSphereInsideFrustum(
                           const plane* planes,
                           const vec3& center,
                           float radius)
{
    for(int i=0;i<6;++i)
    {
        if (planes[i].normal.dot(center) + planes[i].d + radius < 0.0f)
            return false; // complètement en dehors
    }
    return true; // intersecte ou à l’intérieur
}

__global__ void compute_depth_kernel(
        const plane* planes,
        const vec3* positions,
        float* depths,
        int* indices,
        const int N,
        const float* proj
        )
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= N) return;
    auto p = positions[idx];
    if(isSphereInsideFrustum(planes, vec3{p.x,p.y,p.z}, 0.05)){
        depths[idx] = compute_depth(positions[idx], proj);
        indices[idx] = idx;
    }else{
        depths[idx] = -4.0;
        indices[idx] = -1;
    }
}

void project_to_z(const std::array<Plane,6>& clipPlanes,
                  const Eigen::Matrix4f& mvp,
                  const vec3* posPtr,
                  float* depthPtr,
                  int* indicesPtr,
                  int N)
{
    // Copy projection matrix
    float* d_proj;
    cudaMalloc(&d_proj, sizeof(float) * 16);
    cudaMemcpy(d_proj, mvp.data(), sizeof(float) * 16, cudaMemcpyHostToDevice);

    // Copy clip planes
    plane* planes;
    cudaMalloc(&planes, sizeof(plane) * 6);
    cudaMemcpy(planes, clipPlanes.data(), sizeof(plane) * 6, cudaMemcpyHostToDevice);

    // 2. Lancer le kernel
    int blockSize = 256;
    int gridSize = (N + blockSize - 1) / blockSize;
    compute_depth_kernel<<<gridSize, blockSize>>>(planes,
                                                  (const vec3*)posPtr, (float*)depthPtr,
                                                  (int*)indicesPtr,
                                                  N, (const float*)d_proj);

    cudaDeviceSynchronize();
    cudaFree(d_proj);
    cudaFree(planes);
}

bool PointCloudRendererBackend::hasCuda()
{
    int deviceCount = 0;
    cudaGetDeviceCount(&deviceCount);
    return deviceCount > 0;
}
#endif

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
