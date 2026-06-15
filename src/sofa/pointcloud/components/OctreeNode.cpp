#include <sofa/pointcloud/fwd.h>
#include <sofa/pointcloud/components/OctreeNode.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/ScopedAdvancedTimer.h>

// Cube
bool Cube::contains(const sofa::type::Vec3f& position) const {
    return min.x() <= position.x() && max.x() >= position.x()
        && min.y() <= position.y() && max.y() >= position.y()
        && min.z() <= position.z() && max.z() >= position.z();
}

bool Cube::intersects(const Cube& other) const {
    return min.x() <= other.max.x() && max.x() >= other.min.x()
        && min.y() <= other.max.y() && max.y() >= other.min.y()
        && min.z() <= other.max.z() && max.z() >= other.min.z();
}

bool Cube::intersects(const CameraView& camera) const {
    for (int i = 0; i < 6; ++i) {
        const Plane& p = camera.clipPlanes[i];
        sofa::type::Vec3f pVertex;
        pVertex[0] = (p.normal[0] >= 0) ? max[0] : min[0];
        pVertex[1] = (p.normal[1] >= 0) ? max[1] : min[1];
        pVertex[2] = (p.normal[2] >= 0) ? max[2] : min[2];
        if (sofa::type::dot(p.normal, pVertex)  < p.d) {
            return false;
        }
    }
    return true;
}

void Cube::draw(const sofa::core::visual::VisualParams* vparams, sofa::type::RGBAColor color) {
    auto* drawer = vparams->drawTool();

    float xmin = min.x();
    float ymin = min.y();
    float zmin = min.z();
    float xmax = max.x();
    float ymax = max.y();
    float zmax = max.z();

    drawer->drawLine(sofa::type::Vec3f(xmin, ymin, zmin), sofa::type::Vec3f(xmax, ymin, zmin), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymin, zmin), sofa::type::Vec3f(xmax, ymax, zmin), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymax, zmin), sofa::type::Vec3f(xmin, ymax, zmin), color);
    drawer->drawLine(sofa::type::Vec3f(xmin, ymax, zmin), sofa::type::Vec3f(xmin, ymin, zmin), color);

    drawer->drawLine(sofa::type::Vec3f(xmin, ymin, zmax), sofa::type::Vec3f(xmax, ymin, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymin, zmax), sofa::type::Vec3f(xmax, ymax, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymax, zmax), sofa::type::Vec3f(xmin, ymax, zmax), color);        
    drawer->drawLine(sofa::type::Vec3f(xmin, ymax, zmax), sofa::type::Vec3f(xmin, ymin, zmax), color);

    drawer->drawLine(sofa::type::Vec3f(xmin, ymin, zmin), sofa::type::Vec3f(xmin, ymin, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymin, zmin), sofa::type::Vec3f(xmax, ymin, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymax, zmin), sofa::type::Vec3f(xmax, ymax, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmin, ymax, zmin), sofa::type::Vec3f(xmin, ymax, zmax), color);  

    drawer->drawLine(sofa::type::Vec3f(xmin, ymin, zmin), sofa::type::Vec3f(xmin, ymax, zmin), color);
    drawer->drawLine(sofa::type::Vec3f(xmin, ymin, zmax), sofa::type::Vec3f(xmin, ymax, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymin, zmin), sofa::type::Vec3f(xmax, ymax, zmin), color);        
    drawer->drawLine(sofa::type::Vec3f(xmax, ymin, zmax), sofa::type::Vec3f(xmax, ymax, zmax), color);
    
}

// CameraView
bool CameraView::contains(const sofa::type::Vec3f& position) const {
    for (const auto& plane : clipPlanes) {
        if (sofa::type::dot(plane.normal, position) < plane.d) {
            return false;
        }
    }
    return true;
}

// OcTreeNode
bool OcTreeNode::insertSplat(int splatIndex){
    SCOPED_TIMER("OcTreeNode::insertSplat");
    sofa::type::Vec3f position(data->xyz(splatIndex, 0), data->xyz(splatIndex, 1), data->xyz(splatIndex, 2));
    if (!cube->contains(position))
        return false;
    
    if (!isSubdivided && (int)splatIndices.size() < maxSplats) {
        splatIndices.push_back(splatIndex);
        return true;
    }

    if (!isSubdivided) 
        subdivide();

    if (upNorthWest->insertSplat(splatIndex))
        return true;
    if (upNorthEast->insertSplat(splatIndex))
        return true;
    if (upSouthWest->insertSplat(splatIndex))
        return true;
    if (upSouthEast->insertSplat(splatIndex))
        return true;

    if (downNorthWest->insertSplat(splatIndex))
        return true;
    if (downNorthEast->insertSplat(splatIndex))
        return true;
    if (downSouthWest->insertSplat(splatIndex))
        return true;
    if (downSouthEast->insertSplat(splatIndex))
        return true;
    return false;
}

void OcTreeNode::subdivide(){
    SCOPED_TIMER("OcTreeNode::subdivide");
    isSubdivided = true;
    auto x_half = (cube->min.x() + cube->max.x()) / 2;
    auto y_half = (cube->min.y() + cube->max.y()) / 2;
    auto z_half = (cube->min.z() + cube->max.z()) / 2;

    auto min = sofa::type::Vec3f(cube->min.x(), y_half, z_half);
    auto max = sofa::type::Vec3f(x_half, cube->max.y(), cube->max.z());
    upNorthWest = new OcTreeNode(data, new Cube(min, max), maxSplats);

    min = sofa::type::Vec3f(x_half, y_half, z_half);
    max = sofa::type::Vec3f(cube->max.x(), cube->max.y(), cube->max.z());
    upNorthEast = new OcTreeNode(data, new Cube(min, max), maxSplats);

    min = sofa::type::Vec3f(cube->min.x(), cube->min.y(), z_half);
    max = sofa::type::Vec3f(x_half, y_half, cube->max.z());
    upSouthWest = new OcTreeNode(data, new Cube(min, max), maxSplats);

    min = sofa::type::Vec3f(x_half, cube->min.y(), z_half);
    max = sofa::type::Vec3f(cube->max.x(), y_half, cube->max.z());
    upSouthEast = new OcTreeNode(data, new Cube(min, max), maxSplats);

    // Down
    min = sofa::type::Vec3f(cube->min.x(), y_half, cube->min.z());
    max = sofa::type::Vec3f(x_half, cube->max.y(), z_half);
    downNorthWest = new OcTreeNode(data, new Cube(min, max), maxSplats);

    min = sofa::type::Vec3f(x_half, y_half, cube->min.z());
    max = sofa::type::Vec3f(cube->max.x(), cube->max.y(), z_half);
    downNorthEast = new OcTreeNode(data, new Cube(min, max), maxSplats);

    min = sofa::type::Vec3f(cube->min.x(), cube->min.y(), cube->min.z());
    max = sofa::type::Vec3f(x_half, y_half, z_half);
    downSouthWest = new OcTreeNode(data, new Cube(min, max), maxSplats);

    min = sofa::type::Vec3f(x_half, cube->min.y(), cube->min.z());
    max = sofa::type::Vec3f(cube->max.x(), y_half, z_half);
    downSouthEast = new OcTreeNode(data, new Cube(min, max), maxSplats);

    for (auto it = splatIndices.begin(); it != splatIndices.end(); ++it) {
        int index = *it;
        insertSplat(index);
    }
    splatIndices.clear();
}

void OcTreeNode::query(CameraView& camera, std::vector<int>& results){
    if (data == nullptr) return;

    if (!cube->intersects(camera))
        return;

    if  (isSubdivided) {
        upNorthWest->query(camera, results);
        upNorthEast->query(camera, results);
        upSouthWest->query(camera, results);
        upSouthEast->query(camera, results);
        
        downNorthWest->query(camera, results);
        downNorthEast->query(camera, results);
        downSouthWest->query(camera, results);
        downSouthEast->query(camera, results);
        return;
    }

    results.insert(results.end(), splatIndices.begin(), splatIndices.end());
    /*
    for (int index : splatIndices) {
        sofa::type::Vec3f position(data->xyz(index, 0), data->xyz(index, 1), data->xyz(index, 2));
        if (camera.contains(position)) {
            results.push_back(index);
        }
    }*/

}


// ocTreeNodeLOD
bool OcTreeNodeLOD::insertSplat(int splatIndex, int lod){
    SCOPED_TIMER("OcTreeNodeLOD::insertSplat");
    sofa::type::Vec3f position(data->xyz(splatIndex, 0), data->xyz(splatIndex, 1), data->xyz(splatIndex, 2));
    if (!cube->contains(position))
        return false;
    
    if (!isSubdivided && (int)lodIndices[lod].size() < maxSplats) {
        lodIndices[lod].push_back(splatIndex);
        return true;
    }

    if (!isSubdivided) 
        subdivide();

    if (upNorthWest->insertSplat(splatIndex, lod))
        return true;
    if (upNorthEast->insertSplat(splatIndex, lod))
        return true;
    if (upSouthWest->insertSplat(splatIndex, lod))
        return true;
    if (upSouthEast->insertSplat(splatIndex, lod))
        return true;

    if (downNorthWest->insertSplat(splatIndex, lod))
        return true;
    if (downNorthEast->insertSplat(splatIndex, lod))
        return true;
    if (downSouthWest->insertSplat(splatIndex, lod))
        return true;
    if (downSouthEast->insertSplat(splatIndex, lod))
        return true;
    return false;
}

void OcTreeNodeLOD::subdivide(){
    SCOPED_TIMER("OcTreeNodeLOD::subdivide");
    isSubdivided = true;
    auto x_half = (cube->min.x() + cube->max.x()) / 2;
    auto y_half = (cube->min.y() + cube->max.y()) / 2;
    auto z_half = (cube->min.z() + cube->max.z()) / 2;

    auto min = sofa::type::Vec3f(cube->min.x(), y_half, z_half);
    auto max = sofa::type::Vec3f(x_half, cube->max.y(), cube->max.z());
    upNorthWest = new OcTreeNodeLOD(data, new Cube(min, max), maxSplats, distances);

    min = sofa::type::Vec3f(x_half, y_half, z_half);
    max = sofa::type::Vec3f(cube->max.x(), cube->max.y(), cube->max.z());
    upNorthEast = new OcTreeNodeLOD(data, new Cube(min, max), maxSplats, distances);

    min = sofa::type::Vec3f(cube->min.x(), cube->min.y(), z_half);
    max = sofa::type::Vec3f(x_half, y_half, cube->max.z());
    upSouthWest = new OcTreeNodeLOD(data, new Cube(min, max), maxSplats, distances);

    min = sofa::type::Vec3f(x_half, cube->min.y(), z_half);
    max = sofa::type::Vec3f(cube->max.x(), y_half, cube->max.z());
    upSouthEast = new OcTreeNodeLOD(data, new Cube(min, max), maxSplats, distances);

    // Down
    min = sofa::type::Vec3f(cube->min.x(), y_half, cube->min.z());
    max = sofa::type::Vec3f(x_half, cube->max.y(), z_half);
    downNorthWest = new OcTreeNodeLOD(data, new Cube(min, max), maxSplats, distances);

    min = sofa::type::Vec3f(x_half, y_half, cube->min.z());
    max = sofa::type::Vec3f(cube->max.x(), cube->max.y(), z_half);
    downNorthEast = new OcTreeNodeLOD(data, new Cube(min, max), maxSplats, distances);

    min = sofa::type::Vec3f(cube->min.x(), cube->min.y(), cube->min.z());
    max = sofa::type::Vec3f(x_half, y_half, z_half);
    downSouthWest = new OcTreeNodeLOD(data, new Cube(min, max), maxSplats, distances);

    min = sofa::type::Vec3f(x_half, cube->min.y(), cube->min.z());
    max = sofa::type::Vec3f(cube->max.x(), y_half, z_half);
    downSouthEast = new OcTreeNodeLOD(data, new Cube(min, max), maxSplats, distances);

    for (auto it = lodIndices.begin(); it != lodIndices.end(); ++it) {
        int lodValue = it->first;
        auto& splatVec = it->second;
        for (int index : splatVec) {
            insertSplat(index, lodValue);
        }
        splatVec.clear(); 
    }
    lodIndices.clear();
}

void OcTreeNodeLOD::query(CameraView& camera, std::vector<int>& results){
    int lod = getLod(camera);
    if (data == nullptr || lod == -1) return;

    if (!cube->intersects(camera))
        return;

    if  (isSubdivided) {
        upNorthWest->query(camera, results);
        upNorthEast->query(camera, results);
        upSouthWest->query(camera, results);
        upSouthEast->query(camera, results);
        
        downNorthWest->query(camera, results);
        downNorthEast->query(camera, results);
        downSouthWest->query(camera, results);
        downSouthEast->query(camera, results);
        return;
    }
    

    auto indices = lodIndices.find(lod);
    if (indices != lodIndices.end()) {
        results.insert(results.end(), indices->second.begin(), indices->second.end());
    }
    /*auto indices = lodIndices.find(lod);
    if (indices != lodIndices.end()) {
        for (int index : indices->second) {
            sofa::type::Vec3f position(data->xyz(index, 0), data->xyz(index, 1), data->xyz(index, 2));
            if (camera.contains(position)) {
                results.push_back(index);
            }
        }
    }*/

    
}

int OcTreeNodeLOD::getLod(const CameraView& camera) const {
    auto center = (cube->min + cube->max) / 2;
    auto distance = (camera.position - center).norm();
    for (int i = 0; i < (int)distances->size(); ++i) {
        if (distance < (*distances)[i]) return i;
    }
    return -1;
}