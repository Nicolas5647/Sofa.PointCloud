/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2025 CNRS, INRIA, USTL, UJF, CNRS, MGH               *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/core/ObjectFactory.h>
#include <sofa/pointcloud/config.h>
#include <sofa/pointcloud/fwd.h>



extern "C" {
    SOFA_POINTCLOUD_API void initExternalModule();
    SOFA_POINTCLOUD_API const char* getModuleName();
    SOFA_POINTCLOUD_API const char* getModuleVersion();
    SOFA_POINTCLOUD_API const char* getModuleLicense();
    SOFA_POINTCLOUD_API const char* getModuleDescription();
    SOFA_POINTCLOUD_API void registerObjects(sofa::core::ObjectFactory* factory);
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

const char* getModuleName()
{
    return sofa_tostring(SOFA_TARGET);
}

const char* getModuleLicense()
{
    return "";
}

const char* getModuleVersion()
{
    return sofa_tostring(SOFA_POINTCLOUD_VERSION);
}

const char* getModuleDescription()
{
    return "Toolbox to manipulate gaussian splated point clouds";
}

void registerObjects(sofa::core::ObjectFactory* factory)
{
    registerToFactory<sofa::pointcloud::components::PointCloudContainer>(factory);
    registerToFactory<sofa::pointcloud::components::PointCloudRenderer>(factory);
    registerToFactory<sofa::pointcloud::components::PointCloudTransform>(factory);
    registerToFactory<sofa::pointcloud::components::PointCloudInspector>(factory);
    registerToFactory<sofa::pointcloud::components::PointCloudVisualModel>(factory);

    registerToFactory<sofa::pointcloud::components::PointCloudOctreeSelector>(factory);
    registerToFactory<sofa::pointcloud::components::PointCloudOctreeLodSelector>(factory);
}
