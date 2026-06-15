#pragma once

#include <sofa/pointcloud/config.h>
#include <sofa/pointcloud/components/selectors/PointCloudSelector.h>
#include <sofa/pointcloud/components/OctreeNode.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa::pointcloud::components {

    template <typename T> requires std::derived_from<T, OcTreeNodeBase<T>> class PointCloudOctreeBaseSelector : public PointCloudSelector {

        public:
            SOFA_CLASS(SOFA_TEMPLATE(PointCloudOctreeBaseSelector, T), PointCloudSelector);

            PointCloudOctreeBaseSelector():
                PointCloudSelector(),
                d_maxSplats(initData(&d_maxSplats, (int)100, "maxSplats", "Maximum number of splats in the octree", true, true)),
                d_showCube(initData(&d_showCube, false, "showCube", "Display bounding boxes")),
                d_cubeColor(initData(&d_cubeColor, sofa::type::RGBAColor(1.0f, 0.0f, 0.0f, 1.0f), "cubeColor", "Color of the bounding boxes")),
                d_showIntersecCube(initData(&d_showIntersecCube, false, "showIntersecCube", "Display bounding boxes")),
                d_intersecCubeColor(initData(&d_intersecCubeColor, sofa::type::RGBAColor(0.0f, 1.0f, 0.0f, 1.0f), "intersecCubeColor", "Color of the bounding boxes")) {}
            
            ~PointCloudOctreeBaseSelector() {
                delete ocTree;
            };


            Data<int> d_maxSplats;
            T* ocTree = nullptr;

            Data<bool> d_showCube;
            Data<sofa::type::RGBAColor> d_cubeColor;
            Data<bool> d_showIntersecCube;
            Data<sofa::type::RGBAColor> d_intersecCubeColor;

            void draw(const sofa::core::visual::VisualParams* vparams) override  {
                if (ocTree == nullptr) return;
                vparams->drawTool()->saveLastState();
                vparams->drawTool()->disableLighting();
                
                if (d_showCube.getValue())
                    ocTree->draw(vparams, d_cubeColor.getValue());

                if (d_showIntersecCube.getValue())
                    ocTree->drawIntersec(vparams, d_intersecCubeColor.getValue(), lastCameraView);

                vparams->drawTool()->restoreLastState();
            };

            bool updateSelection(BaseCamera* camera, float aspect, std::vector<int>& out_indices) override {
                if (ocTree == nullptr) initOctree();

                sofa::type::Vec3 position = camera->getPosition();
                sofa::type::Quat direction = camera->getOrientation();
                auto fov = camera->getFieldOfView();
                auto near = camera->getZNear();
                auto far = camera->getZFar();
                if (position == lastCameraPosition && fov == lastFOV && near == lastNear && far == lastFar)
                    return false;
                
                lastCameraPosition = position;
                lastFOV = fov;
                lastNear = near;
                lastFar = far;
                
                CameraView view(position,
                    direction,
                    fov,
                    near,
                    far,
                    aspect);
                lastCameraView = view;
                
                out_indices.clear();
                ocTree->query(view, out_indices);
                return true;
            };

            virtual void initOctree() = 0;
            virtual bool updateSh(GaussianData* renderingData, int offset) override = 0;

        protected:
            CameraView lastCameraView;
            sofa::type::Vec3 lastCameraPosition;
            double lastFOV;
            double lastNear;
            double lastFar;

    };
}