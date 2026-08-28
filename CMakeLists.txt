cmake_minimum_required(VERSION 3.10)

project(G1)

# ============================================================
# PCL
# ============================================================

find_package(PCL REQUIRED COMPONENTS
    common
    io
    filters
    segmentation
    visualization
)

# ============================================================
# Open3D
# ============================================================

set(Open3D_DIR
    "/home/atchayasree/Downloads/open3d-devel-linux-x86_64-cxx11-abi-0.19.0/lib/cmake/Open3D"
)

find_package(Open3D REQUIRED)

# ============================================================
# Include / Library settings
# ============================================================

include_directories(
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_definitions(
    ${PCL_DEFINITIONS}
)

# ============================================================
# ============================================================

add_executable(
    1_roi_to_mm
    1_roi_to_mm.cpp
)

target_link_libraries(
    1_roi_to_mm
    ${PCL_LIBRARIES}
)

add_executable(
    2_dbscan_clusters
    2_dbscan_clusters.cpp
)

target_link_libraries(
    2_dbscan_clusters
    Open3D::Open3D
)

add_executable(
    3_select_pallet_cluster
    3_select_pallet_cluster.cpp
)

target_link_libraries(
    3_select_pallet_cluster
    PRIVATE
    Open3D::Open3D
)

add_executable(
    4_roi_fork_entry
    4_roi_fork_entry.cpp
)

target_link_libraries(
    4_roi_fork_entry
    Open3D::Open3D
)

add_executable(
    5_template_fork_entry
    5_template_fork_entry.cpp
    )

target_link_libraries(
    5_template_fork_entry
    PRIVATE
    Open3D::Open3D
)


add_executable(
    6_icp_match
    6_icp_match.cpp
)

target_link_libraries(
    6_icp_match
    Open3D::Open3D
)

add_executable(pallet_detection pallet_detection.cpp)

target_link_libraries(
    pallet_detection
    PRIVATE
    Open3D::Open3D
)

add_executable(multipallet_detection
    multipallet_detection.cpp
)

target_link_libraries(
    multipallet_detection
    PRIVATE
    Open3D::Open3D
)



