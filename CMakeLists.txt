cmake_minimum_required(VERSION 3.12)

project(Sofa.PointCloud LANGUAGES CXX)
option(SOFA_POINTCLOUD_CUDA "Enable CUDA support")
set(SOFA_POINTCLOUD_SRC "src/sofa/pointcloud")

find_package(Sofa.Core REQUIRED)
find_package(Sofa.Component.Visual REQUIRED)
find_package(Sofa.GL REQUIRED)
find_package(SofaPython3 REQUIRED)

include_directories(extlibs/tinyply/source)

# ================= CUDA detection =================
if(SOFA_POINTCLOUD_CUDA)
    include(CheckLanguage)
    check_language(CUDA)

    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        set(HAVE_CUDA TRUE)
        message(WARNING "CUDA enabled")
    else()
        set(HAVE_CUDA FALSE)
        message(WARNING "CUDA requested but not found, disabling")
    endif()
else()
    set(HAVE_CUDA FALSE)
    message(WARNING "CUDA requested but not found, disabling")
endif()


set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(HAVE_CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_FLAGS "-D__CUDA_NO_MATH_FUNCTIONS__ -O3 -std=c++17 -Xcompiler=-fPIC -Wno-attributes -fsized-deallocation -v")
endif()

set(HEADER_FILES
    ${SOFA_POINTCLOUD_SRC}/config.h.in
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudContainer.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudTransform.h
    ${SOFA_POINTCLOUD_SRC}/components/utils.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRenderer.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRendererBackend.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRendererCUDA.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudVisualModel.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudInspector.h
    ${SOFA_POINTCLOUD_SRC}/components/liteviz-dataloader.h
    ${SOFA_POINTCLOUD_SRC}/components/selectors/PointCloudSelector.h
    ${SOFA_POINTCLOUD_SRC}/components/OctreeNode.h
    ${SOFA_POINTCLOUD_SRC}/components/selectors/PointCloudOctreeBaseSelector.h
    ${SOFA_POINTCLOUD_SRC}/components/selectors/PointCloudOctreeSelector.h
    extlibs/liteviz-gs/liteviz/dataloader.h
    extlibs/liteviz-gs/liteviz/renderer.h
    extlibs/liteviz-gs/liteviz/shader.h
    extlibs/liteviz-gs/liteviz/utils.h
    extlibs/diff-gaussian-rasterization/cuda_rasterizer/rasterizer.h
    extlibs/diff-gaussian-rasterization/cuda_rasterizer/rasterizer_impl.h
    extlibs/diff-gaussian-rasterization/cuda_rasterizer/forward.h
)

set(SOURCE_FILES
    ${SOFA_POINTCLOUD_SRC}/init.cpp
    ${SOFA_POINTCLOUD_SRC}/fwd.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudContainer.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudTransform.cpp
    ${SOFA_POINTCLOUD_SRC}/components/utils.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRenderer.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudVisualModel.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudInspector.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRendererCUDA.cpp
    ${SOFA_POINTCLOUD_SRC}/components/OctreeNode.cpp
    ${SOFA_POINTCLOUD_SRC}/components/selectors/PointCloudOctreeSelector.cpp
)

# CUDA sources conditionnels
if(HAVE_CUDA)
    list(APPEND SOURCE_FILES
        ${SOFA_POINTCLOUD_SRC}/components/PointCloudRendererBackend.cu
    )
else()
    list(APPEND SOURCE_FILES
        ${SOFA_POINTCLOUD_SRC}/components/PointCloudRendererBackend.cpp
    )
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Core Sofa.Component.Visual Sofa.GL SofaPython3::Plugin tbb)

if(HAVE_CUDA)
    target_compile_definitions(${PROJECT_NAME} PRIVATE SOFA_POINTCLOUD_USE_CUDA)
endif()

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src" "extlibs"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)
