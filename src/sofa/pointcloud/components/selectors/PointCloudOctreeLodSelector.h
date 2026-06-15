#pragma once

#include <sofa/pointcloud/config.h>
#include <sofa/pointcloud/components/selectors/PointCloudOctreeBaseSelector.h>
#include <sofa/pointcloud/components/OctreeNode.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa::pointcloud::components {

    class PointCloudOctreeLodSelector : public PointCloudOctreeBaseSelector<OcTreeNodeLOD> {
        public:
            SOFA_CLASS(PointCloudOctreeLodSelector, SOFA_TEMPLATE(PointCloudOctreeBaseSelector, OcTreeNodeLOD));

            using PointCloudOctreeBaseSelector<OcTreeNodeLOD>::l_geometries;
            using PointCloudOctreeBaseSelector<OcTreeNodeLOD>::ocTree;

            PointCloudOctreeLodSelector();
            ~PointCloudOctreeLodSelector() override;

            Data<bool> d_showLod;
            Data<sofa::type::vector<sofa::type::RGBAColor>> d_colors;
            Data<sofa::type::vector<float>> d_thresholds;

            GaussianData* data;


            void initOctree() override;
            bool updateSh(GaussianData* renderingData, int offset) override;
            int getLod(const size_t index) const;
            size_t size() {return ocTree->size();};

        protected:
            bool lastShowLod = false;
    };
}
