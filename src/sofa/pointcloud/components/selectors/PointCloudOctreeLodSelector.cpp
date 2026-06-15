#include "sofa/helper/logging/Messaging.h"
#include <sofa/pointcloud/fwd.h>
#include <sofa/pointcloud/components/selectors/PointCloudOctreeLodSelector.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/pointcloud/components/OctreeNode.h>
#include <sofa/pointcloud/components/utils.h>

namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudOctreeLodSelector>(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A point cloud LOD selector.")
                             .add< sofa::pointcloud::components::PointCloudOctreeLodSelector >());
}

}

namespace sofa::pointcloud::components {

    PointCloudOctreeLodSelector::PointCloudOctreeLodSelector() :
        PointCloudOctreeBaseSelector<OcTreeNodeLOD>(),
        d_showLod(initData(&d_showLod, false, "showLod", "Change Color of splats according to LOD")),
        d_colors(initData(&d_colors, sofa::type::vector<sofa::type::RGBAColor>({sofa::type::RGBAColor(1.0f, 0.0f, 0.0f, 1.0f)}), "colors", "Colors of the splats", true, true)),
        d_thresholds(initData(&d_thresholds, "thresholds", "List of distances for changing LOD (ex: \"10 50 100\")")) {}

    PointCloudOctreeLodSelector::~PointCloudOctreeLodSelector() {
        delete data;
    }

    void PointCloudOctreeLodSelector::initOctree() {
        if (d_thresholds.getValue().size() == 0) {
            msg_error() << "No thresholds specified";
            return;
        }

        data = new GaussianData{};
        for (auto& visual : this->l_geometries) {
            if (visual->isComponentStateValid()) {
                append(*data, *visual->data);
            }


        }

        int n = d_thresholds.getValue().size();
        type::vector<sofa::type::RGBAColor> colors(n);
        for (int i = 0; i < n; ++i) {
            float ratio = (n > 1) ? (float)i / (n - 1) : 0.0f;
            colors[i] = sofa::type::RGBAColor(ratio, 1.0f - ratio, 0.0f, 0.5f);
        }
        d_colors.setValue(colors);

        auto min = sofa::type::Vec3f(data->xyz.col(0).minCoeff(), data->xyz.col(1).minCoeff(), data->xyz.col(2).minCoeff());
        auto max = sofa::type::Vec3f(data->xyz.col(0).maxCoeff(), data->xyz.col(1).maxCoeff(), data->xyz.col(2).maxCoeff());
        auto cube = new Cube(min, max);
        
        auto thresholds = d_thresholds.getValue();
        std::vector<float>* distances = new std::vector<float>();
        for (int i = 0; i < (int)thresholds.size(); i++) {
            distances->push_back(thresholds[i]);
        }


        this->ocTree = new OcTreeNodeLOD(data, cube, this->d_maxSplats.getValue(), distances);
        int offset = 0;
        for (int lod = 0; lod < (int)this->l_geometries.size(); lod++) {
            auto visual = this->l_geometries[lod];
            for (int i = 0; i < (int)visual->data->size(); i++) {
                this->ocTree->insertSplat(i + offset, lod);
            }
            offset += visual->data->size();
        }
    }

    int PointCloudOctreeLodSelector::getLod(const size_t index) const {
        int lod = 0;
        int offset = 0;
        for (auto view : l_geometries) {
            auto size = view->data->size();
            if (offset + size > index)
                return lod;
            lod++;
            offset += size;
        }
        return lod;
    }

    bool PointCloudOctreeLodSelector::updateSh(GaussianData* renderingData, int offset) {
        if (d_showLod.getValue() == lastShowLod) return false;
        msg_info() << "Show Lod " << d_showLod.getValue();

        lastShowLod = d_showLod.getValue();

        bool showLod = d_showLod.getValue();
        int N = data->size();
        for (size_t i = 0; i < N; i++) {
            int sh_dim = renderingData->sh.cols();
            int lodValue = getLod(i);
            
            auto& colorList = d_colors.getValue();
            if (lodValue >= (int)colorList.size()) lodValue = colorList.size() - 1;
            
            auto color = colorList[lodValue];
            const float SH_C0 = 0.28209479177387814f;
            
            if (showLod) {
                for (int j = 0; j < sh_dim; j += 3) {
                    float sh_orig_r = data->sh.row(offset + i)(j);
                    float sh_orig_g = data->sh.row(offset + i)(j + 1);
                    float sh_orig_b = data->sh.row(offset + i)(j + 2);

                    float r_vis = std::max(0.0f, SH_C0 * sh_orig_r + 0.5f);
                    float g_vis = std::max(0.0f, SH_C0 * sh_orig_g + 0.5f);
                    float b_vis = std::max(0.0f, SH_C0 * sh_orig_b + 0.5f);
                    float lum = 0.2126f * r_vis + 0.7152f * g_vis + 0.0722f * b_vis;

                    // Luminosité minimale (0.2) pour que le noir soit coloré
                    float mix_lum = 0.2f + (lum * 0.8f); 

                    float new_r = color.r() * mix_lum;
                    float new_g = color.g() * mix_lum;
                    float new_b = color.b() * mix_lum;

                    renderingData->sh.row(offset + i)(j)     = (new_r - 0.5f) / SH_C0;
                    renderingData->sh.row(offset + i)(j + 1) = (new_g - 0.5f) / SH_C0;
                    renderingData->sh.row(offset + i)(j + 2) = (new_b - 0.5f) / SH_C0;
                }
            } else {
                renderingData->sh.row(offset + i) = data->sh.row(i);
            }
        }
        return true;

    }

};
