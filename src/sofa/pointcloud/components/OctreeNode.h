#pragma once

#include <sofa/pointcloud/config.h>
#include <sofa/pointcloud/components/PointCloudRendererBackend.h>
#include <sofa/pointcloud/components/liteviz-dataloader.h>
#include <sofa/component/visual/BaseCamera.h>
#include <vector>

using sofa::component::visual::BaseCamera;
struct CameraView {
    sofa::type::Vec3 position;
    sofa::type::Quat<double> orientation;
    double fov;
    double near;
    double far;
    float aspect;

    std::array<Plane, 6> clipPlanes;

    CameraView() : position(0,0,0), orientation(1,0,0,0), fov(0), near(0), far(0), aspect(0) {}
    CameraView(sofa::type::Vec3f position, 
        sofa::type::Quat<double> orientation, 
        double fov, 
        double near, 
        double far, 
        float  aspect):
            position(position), 
            orientation(orientation), 
            fov(fov), 
            near(near), 
            far(far),
            aspect(aspect) {
        double fovRad = fov * M_PI / 180.0;
        double tanHalfFov = std::tan(fovRad * 0.5);
        double hTanHalfFov = tanHalfFov * aspect;
        
        sofa::type::Vec3 forward = orientation.rotate(sofa::type::Vec3(0, 0, -1));
        sofa::type::Vec3 up      = orientation.rotate(sofa::type::Vec3(0, 1, 0));
        sofa::type::Vec3 right   = orientation.rotate(sofa::type::Vec3(1, 0, 0));
        forward.normalize();
        up.normalize();
        right.normalize();

        // Near and Far
        clipPlanes[0] = Plane(forward,  position + forward * near); 
        clipPlanes[1] = Plane(-forward, position + forward * far);

        // Top 
        sofa::type::Vec3 topNormal = forward * tanHalfFov - up;
        clipPlanes[2] = Plane(topNormal, position);

        // Bottom
        sofa::type::Vec3 bottomNormal = forward * tanHalfFov + up;
        clipPlanes[3] = Plane(bottomNormal, position);

        // Left
        sofa::type::Vec3 leftNormal = forward * hTanHalfFov + right;
        clipPlanes[4] = Plane(leftNormal, position);

        // Right
        sofa::type::Vec3 rightNormal = forward * hTanHalfFov - right;
        clipPlanes[5] = Plane(rightNormal, position);
    }
    ~CameraView() {}

    bool contains(const sofa::type::Vec3f& position) const;
};

struct Cube {
    sofa::type::Vec3f min;
    sofa::type::Vec3f max;

    Cube() : min(0,0,0), max(0,0,0) {} 
    Cube(const sofa::type::Vec3f& min, const sofa::type::Vec3f& max) : min(min), max(max) {}

    bool contains(const sofa::type::Vec3f& position) const;

    bool intersects(const Cube& other) const;

    bool intersects(const CameraView& camera) const;

    void draw(const sofa::core::visual::VisualParams* vparams, sofa::type::RGBAColor color); 

    inline friend std::istream& operator >> ( std::istream& in, Cube& s ){
        in >> s.min >> s.max;
        return in;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const Cube& s ){
        out << s.min << " " << s.max;
        return out;
    }
};

template <typename T>
struct OcTreeNodeBase {
    GaussianData* data;
    Cube* cube;
    int maxSplats;

    bool isSubdivided = false;
    T* upNorthWest = nullptr;
    T* upNorthEast = nullptr;
    T* upSouthWest = nullptr;
    T* upSouthEast = nullptr;

    T* downNorthWest = nullptr;
    T* downNorthEast = nullptr;
    T* downSouthWest = nullptr;
    T* downSouthEast = nullptr;

    OcTreeNodeBase(GaussianData* data, Cube* cube, int maxSplats) : data(data), cube(cube), maxSplats(maxSplats) {}

    virtual ~OcTreeNodeBase() {
        delete upNorthWest;   delete upNorthEast;   delete upSouthWest;   delete upSouthEast;
        delete downNorthWest; delete downNorthEast; delete downSouthWest; delete downSouthEast;
    }
    void draw(const sofa::core::visual::VisualParams* vparams, sofa::type::RGBAColor color) {
        cube->draw(vparams, color);

        if (isSubdivided) {
            if (upNorthWest) upNorthWest->draw(vparams, color);
            if (upNorthEast) upNorthEast->draw(vparams, color);
            if (upSouthWest) upSouthWest->draw(vparams, color);
            if (upSouthEast) upSouthEast->draw(vparams, color);
            if (downNorthWest) downNorthWest->draw(vparams, color);
            if (downNorthEast) downNorthEast->draw(vparams, color);
            if (downSouthWest) downSouthWest->draw(vparams, color);
            if (downSouthEast) downSouthEast->draw(vparams, color);
        }
    }

    void drawIntersec(const sofa::core::visual::VisualParams* vparams, sofa::type::RGBAColor color, CameraView& camera) {
        if (!cube->intersects(camera))
            return;
        cube->draw(vparams, color);

        if (isSubdivided) {
            if (upNorthWest) upNorthWest->drawIntersec(vparams, color, camera);
            if (upNorthEast) upNorthEast->drawIntersec(vparams, color, camera);
            if (upSouthWest) upSouthWest->drawIntersec(vparams, color, camera);
            if (upSouthEast) upSouthEast->drawIntersec(vparams, color, camera);
            if (downNorthWest) downNorthWest->drawIntersec(vparams, color, camera);
            if (downNorthEast) downNorthEast->drawIntersec(vparams, color, camera);
            if (downSouthWest) downSouthWest->drawIntersec(vparams, color, camera);
            if (downSouthEast) downSouthEast->drawIntersec(vparams, color, camera);
        }
    }

};

struct OcTreeNode : public OcTreeNodeBase<OcTreeNode> {
    OcTreeNode(GaussianData* data, Cube* cube, int maxSplats) 
    : OcTreeNodeBase<OcTreeNode>(data, cube, maxSplats) {}

    std::vector<int> splatIndices;

    bool insertSplat(int splatIndex);
    void subdivide();
    void query(CameraView& camera, std::vector<int>& results);

    size_t size() {
        if (isSubdivided) {
            return upNorthWest->size() + upNorthEast->size() + upSouthWest->size() + upSouthEast->size()
                + downNorthWest->size() + downNorthEast->size() + downSouthWest->size() + downSouthEast->size();
        }
        return splatIndices.size();
    }
};

struct OcTreeNodeLOD : public OcTreeNodeBase<OcTreeNodeLOD> {
    std::map<int, sofa::type::vector<int>> lodIndices;
    const std::vector<float>* distances;
    
    OcTreeNodeLOD(GaussianData* data, Cube* cube, int maxSplats, const std::vector<float>* distances) : OcTreeNodeBase<OcTreeNodeLOD>(data, cube, maxSplats), distances(distances) {}


    bool insertSplat(int splatIndex, int lod);
    void subdivide();
    void query(CameraView& camera, std::vector<int>& results);

    int getLod(const CameraView& camera) const;
    size_t size() {
        if (isSubdivided) {
            return upNorthWest->size() + upNorthEast->size() + upSouthWest->size() + upSouthEast->size()
                + downNorthWest->size() + downNorthEast->size() + downSouthWest->size() + downSouthEast->size();
        }
        size_t size = 0;
        for (auto it = lodIndices.begin(); it != lodIndices.end(); ++it) {
            size += it->second.size();
        }
        return size;
    }
};

