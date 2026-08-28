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
# Convert ROI to millimetres
# ============================================================

add_executable(
    convert_roi_to_mm
    convert_roi_to_mm.cpp
)

target_link_libraries(
    convert_roi_to_mm
    ${PCL_LIBRARIES}
)

# ============================================================
# DBSCAN clustering
# ============================================================

add_executable(
    dbscan_clusters
    dbscan_clusters.cpp
)

target_link_libraries(
    dbscan_clusters
    Open3D::Open3D
)

# ============================================================
# Fork entry extraction
# ============================================================

add_executable(
    extract_fork_entry
    extract_fork_entry.cpp
)

target_link_libraries(
    extract_fork_entry
    Open3D::Open3D
)

# ============================================================
# Scene front plane extraction from ROI
# ============================================================

add_executable(
    extract_front_plane
    extract_front_plane.cpp
)

target_link_libraries(
    extract_front_plane
    Open3D::Open3D
)

# ============================================================
# Template 7-plane extraction
# ============================================================

add_executable(
    extract_template_front_plane
    extract_template_front_plane.cpp
)

target_link_libraries(
    extract_template_front_plane
    Open3D::Open3D
)


add_executable(
    icp_match
    icp_match.cpp
)

target_link_libraries(
    icp_match
    Open3D::Open3D
)