#pragma once

#include <sofa/pointcloud/config.h>
#include <sofa/pointcloud/components/selectors/PointCloudOctreeBaseSelector.h>
#include <sofa/pointcloud/components/OctreeNode.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa::pointcloud::components {

    class PointCloudOctreeSelector : public PointCloudOctreeBaseSelector<OcTreeNode> {
        public:
            SOFA_CLASS(PointCloudOctreeSelector, SOFA_TEMPLATE(PointCloudOctreeBaseSelector, OcTreeNode));

            using PointCloudOctreeBaseSelector<OcTreeNode>::l_geometries;
            using PointCloudOctreeBaseSelector<OcTreeNode>::ocTree;

            PointCloudOctreeSelector();
            ~PointCloudOctreeSelector() override;

            void initOctree() override;
            bool updateSh(GaussianData* renderingData, int offset) override {return false;};
            size_t size() {return ocTree->size();};

        protected:
    };
}

