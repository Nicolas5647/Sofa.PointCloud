#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/pointcloud/components/PointCloudContainer.h>
#include <sofa/core/visual/VisualParams.h>


namespace sofa::pointcloud::components {
    using sofa::component::visual::BaseCamera;

    class PointCloudVisualModel;


    class PointCloudSelector : public sofa::core::objectmodel::BaseObject {
        private:
            template<class T>
            using MultiLink = core::objectmodel::MultiLink<PointCloudSelector, T, core::objectmodel::BaseLink::FLAG_STOREPATH>;

        public:
            SOFA_CLASS(PointCloudSelector, sofa::core::objectmodel::BaseObject);

            MultiLink<PointCloudContainer> l_geometries;

            PointCloudSelector() : l_geometries(initLink("geometries", "links to the topology containers")) {}


            virtual bool updateSelection(BaseCamera* camera, float aspect, std::vector<int>& out_indices) = 0;
            virtual bool updateSh(GaussianData* renderingData, int offset) = 0;
    };

}
