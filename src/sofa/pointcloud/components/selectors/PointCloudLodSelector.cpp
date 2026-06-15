
#include <sofa/pointcloud/fwd.h>
#include <sofa/pointcloud/components/selectors/PointCloudLodSelector.h>
#include <sofa/core/ObjectFactory.h>


namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudLodSelector>(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A point cloud LOD selector.")
                             .add< sofa::pointcloud::components::PointCloudLodSelector >());
}

}


namespace sofa::pointcloud::components {

    PointCloudLodSelector::PointCloudLodSelector() :
        PointCloudSelector(),
        d_thresholds(initData(&d_thresholds, "thresholds", "List of distances for changing LOD (ex: \"10 50 100\")")),
        d_currentLod(initData(&d_currentLod, (int)-1, "currentLod", "Index of the current LOD", true, true)) {
            if (d_thresholds.getValue().size() != l_geometries.size()) {
                msg_error() << "The number of thresholds must be equal to the number of geometries";
                d_componentState = sofa::core::objectmodel::ComponentState::Invalid;
            }
        }

    PointCloudLodSelector::~PointCloudLodSelector() {
        delete center;
    }

    bool PointCloudLodSelector::updateSelection(BaseCamera* camera, float aspect, std::vector<int>& out_indices) {
        if (l_geometries.empty()) {
            msg_error() << "No geometries specified";
            return false;
        }

        const auto& thresholds = d_thresholds.getValue();
        if (thresholds.empty()) {
            msg_error() << "No thresholds specified";
            return false;
        }

        auto cameraPos = camera->getPosition();
        if (center == nullptr) {
            auto min = sofa::type::Vec3f(l_geometries[0]->data->xyz.col(0).minCoeff(), l_geometries[0]->data->xyz.col(1).minCoeff(), l_geometries[0]->data->xyz.col(2).minCoeff());
            auto max = sofa::type::Vec3f(l_geometries[0]->data->xyz.col(0).maxCoeff(), l_geometries[0]->data->xyz.col(1).maxCoeff(), l_geometries[0]->data->xyz.col(2).maxCoeff());
            center = new sofa::type::Vec<3U, float>((min + max) / 2);
        }
        float dist = (cameraPos - *center).norm(); 
        

        int lodIndex = -1;
        for (int i = 0; i < thresholds.size(); i++) {
            float t = thresholds[i];
            if (dist < t ) {
                lodIndex = i;
                break;
            }
        }

        if (d_currentLod.getValue() == lodIndex) return false; 
        d_currentLod.setValue(lodIndex);
        out_indices.clear();

        if (lodIndex >= 0 && lodIndex < static_cast<int>(l_geometries.size())) {
            const auto& targetGeometry = l_geometries[lodIndex];
            const auto& localIndices = targetGeometry->d_indices.getValue();
            
            msg_info() << "Selected LOD " << lodIndex << " for distance " << dist 
                       << " with " << localIndices.size() << " indices";
            
            int offset = 0;
            for (int i = 0; i < lodIndex; i++) {
                offset += l_geometries[i]->d_indices.getValue().size();
            }
           
            out_indices.resize(localIndices.size());
            for (size_t i = 0; i < localIndices.size(); ++i) {
                out_indices[i] = localIndices[i] + offset;
            }
        } else {
            msg_info() << "No LOD selected for distance " << dist;
        }
        return true;
    }
};
