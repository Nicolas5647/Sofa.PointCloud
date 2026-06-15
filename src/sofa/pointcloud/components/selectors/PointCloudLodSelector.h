#pragma once

#include <sofa/pointcloud/config.h>
#include <sofa/pointcloud/components/selectors/PointCloudSelector.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa::pointcloud::components {
    class PointCloudLodSelector : public PointCloudSelector {
        public:
            SOFA_CLASS(PointCloudLodSelector, PointCloudSelector);

            Data<sofa::type::vector<float>> d_thresholds;

            PointCloudLodSelector();
            ~PointCloudLodSelector();

            bool updateSelection(BaseCamera* camera, float aspect, std::vector<int>& out_indices) override ;
            bool updateSh(GaussianData* renderingData, int offset) override {return false;};
            Data<int> d_currentLod;

            sofa::type::Vec<3U, float>* center = nullptr;
    };
}