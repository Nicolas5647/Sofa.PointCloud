#include <sofa/pointcloud/fwd.h>
#include <sofa/pointcloud/components/selectors/PointCloudOctreeSelector.h>
#include <sofa/core/ObjectFactory.h>


namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudOctreeSelector>(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A point cloud LOD selector.")
                             .add< sofa::pointcloud::components::PointCloudOctreeSelector >());
}

}

namespace sofa::pointcloud::components {
    
    PointCloudOctreeSelector::PointCloudOctreeSelector() :
        PointCloudOctreeBaseSelector<OcTreeNode>() {}

    PointCloudOctreeSelector::~PointCloudOctreeSelector() {}



    void PointCloudOctreeSelector::initOctree() {
            if (this->l_geometries.size() > 1) msg_warning() << "More than one geometry in an octree selector. Only the first one will be used.";


        auto data = this->l_geometries[0]->data;
        auto min = sofa::type::Vec3f(this->l_geometries[0]->data->xyz.col(0).minCoeff(), this->l_geometries[0]->data->xyz.col(1).minCoeff(), this->l_geometries[0]->data->xyz.col(2).minCoeff());
        auto max = sofa::type::Vec3f(this->l_geometries[0]->data->xyz.col(0).maxCoeff(), this->l_geometries[0]->data->xyz.col(1).maxCoeff(), this->l_geometries[0]->data->xyz.col(2).maxCoeff());
        auto cube = new Cube(min, max);
        this->ocTree = new OcTreeNode(data, cube, d_maxSplats.getValue());
        for (int i = 0; i < (int)this->l_geometries[0]->data->size(); i++) {
            this->ocTree->insertSplat(i);
        }
    }

};
