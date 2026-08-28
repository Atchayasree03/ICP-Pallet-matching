// #include <iostream>
// #include <vector>
// #include <memory>
// #include <filesystem>
// #include <iomanip>

// #include <open3d/Open3D.h>

// using namespace std;
// using namespace open3d;

// namespace fs = std::filesystem;


// // ============================================================
// // MAIN
// // ============================================================

// int main()
// {
//     string input_file = "../pallet_1.pcd";
//     string output_dir = "../template_planes";

//     fs::create_directories(output_dir);


//     cout << "\n";
//     cout << "============================================\n";
//     cout << "       TEMPLATE - 7 PLANE EXTRACTION\n";
//     cout << "============================================\n";

//     cout << "Input : " << input_file << "\n";
//     cout << "Output: " << output_dir << "\n";


//     // ========================================================
//     // LOAD TEMPLATE
//     // ========================================================

//     auto cloud =
//         make_shared<geometry::PointCloud>();

//     if (!io::ReadPointCloud(input_file, *cloud))
//     {
//         cerr << "ERROR: Cannot load pallet_1.pcd\n";
//         return -1;
//     }

//     cloud->RemoveNonFinitePoints();


//     cout << "\nLoaded template.\n";
//     cout << "Points: "
//          << cloud->points_.size()
//          << "\n";


//     // ========================================================
//     // TEMPLATE IS IN MM
//     // ========================================================

//     const double distance_threshold = 5.0;
//     const int ransac_n = 3;
//     const int iterations = 3000;
//     const double probability = 0.999;


//     // ========================================================
//     // REMAINING CLOUD
//     // ========================================================

//     auto remaining =
//         make_shared<geometry::PointCloud>(*cloud);


//     // ========================================================
//     // EXTRACT 7 PLANES
//     // ========================================================

//     for (int plane_id = 0;
//          plane_id < 7;
//          plane_id++)
//     {
//         if (remaining->points_.size() < 20)
//         {
//             cout << "\nNot enough points remaining.\n";
//             break;
//         }


//         // ----------------------------------------------------
//         // RANSAC PLANE
//         // ----------------------------------------------------

//         auto result =
//             remaining->SegmentPlane(
//                 distance_threshold,
//                 ransac_n,
//                 iterations,
//                 probability
//             );


//         Eigen::Vector4d plane =
//             std::get<0>(result);

//         vector<size_t> inliers =
//             std::get<1>(result);


//         if (inliers.size() < 20)
//         {
//             cout << "\nPlane extraction stopped.\n";
//             break;
//         }


//         // ----------------------------------------------------
//         // EXTRACT PLANE
//         // ----------------------------------------------------

//         auto plane_cloud =
//             remaining->SelectByIndex(inliers);


//         // ----------------------------------------------------
//         // DIMENSIONS
//         // ----------------------------------------------------

//         auto bbox =
//             plane_cloud->GetAxisAlignedBoundingBox();

//         Eigen::Vector3d min_bound =
//             bbox.GetMinBound();

//         Eigen::Vector3d max_bound =
//             bbox.GetMaxBound();

//         Eigen::Vector3d size =
//             max_bound - min_bound;


//         // ----------------------------------------------------
//         // NORMAL
//         // ----------------------------------------------------

//         double nx = plane(0);
//         double ny = plane(1);
//         double nz = plane(2);


//         // ----------------------------------------------------
//         // OUTPUT FILE
//         // ----------------------------------------------------

//         string filename =
//             output_dir +
//             "/plane_" +
//             to_string(plane_id) +
//             ".pcd";


//         // ----------------------------------------------------
//         // SAVE
//         // ----------------------------------------------------

//         io::WritePointCloud(
//             filename,
//             *plane_cloud
//         );


//         // ----------------------------------------------------
//         // PRINT INFORMATION
//         // ----------------------------------------------------

//         cout << "\n";
//         cout << "--------------------------------------------\n";

//         cout << "PLANE "
//              << plane_id
//              << "\n";

//         cout << "Points : "
//              << plane_cloud->points_.size()
//              << "\n";

//         cout << fixed << setprecision(3);

//         cout << "Normal : ("
//              << nx << ", "
//              << ny << ", "
//              << nz << ")\n";

//         cout << "Size X : "
//              << size(0)
//              << " mm\n";

//         cout << "Size Y : "
//              << size(1)
//              << " mm\n";

//         cout << "Size Z : "
//              << size(2)
//              << " mm\n";

//         cout << "Saved  : "
//              << filename
//              << "\n";


//         // ----------------------------------------------------
//         // REMOVE PLANE
//         // ----------------------------------------------------

//         vector<bool> is_inlier(
//             remaining->points_.size(),
//             false
//         );

//         for (size_t index : inliers)
//         {
//             if (index < is_inlier.size())
//                 is_inlier[index] = true;
//         }


//         vector<size_t> remaining_indices;

//         for (size_t i = 0;
//              i < is_inlier.size();
//              i++)
//         {
//             if (!is_inlier[i])
//                 remaining_indices.push_back(i);
//         }


//         remaining =
//             remaining->SelectByIndex(
//                 remaining_indices
//             );
//     }


//     // ========================================================
//     // DONE
//     // ========================================================

//     cout << "\n";
//     cout << "============================================\n";
//     cout << "       EXTRACTION COMPLETE\n";
//     cout << "============================================\n";

//     cout << "7 plane files are in:\n";
//     cout << output_dir << "\n";


//     return 0;
// } 7 plane extraction code is commented out above. The following code is for selecting the front plane from the extracted planes.
























#include <iostream>
#include <memory>

#include <open3d/Open3D.h>

using namespace std;
using namespace open3d;

int main()
{
    // ============================================================
    // INPUT / OUTPUT
    // ============================================================

    string input_file =
        "../template_planes/plane_0.pcd";

    string output_file =
        "../template_front_plane.pcd";


    cout << "\n";
    cout << "============================================\n";
    cout << "       SELECT TEMPLATE FRONT PLANE\n";
    cout << "============================================\n";

    cout << "Input  : " << input_file << "\n";
    cout << "Output : " << output_file << "\n";


    // ============================================================
    // LOAD PLANE 0
    // ============================================================

    auto plane =
        make_shared<geometry::PointCloud>();

    if (!io::ReadPointCloud(
            input_file,
            *plane))
    {
        cerr << "\nERROR: Cannot load plane_0.pcd\n";
        return -1;
    }


    // Remove invalid points
    plane->RemoveNonFinitePoints();


    cout << "\nPlane 0 loaded successfully.\n";

    cout << "Points : "
         << plane->points_.size()
         << "\n";


    // ============================================================
    // CALCULATE DIMENSIONS
    // ============================================================

    auto bbox =
        plane->GetAxisAlignedBoundingBox();

    Eigen::Vector3d min_bound =
        bbox.GetMinBound();

    Eigen::Vector3d max_bound =
        bbox.GetMaxBound();

    Eigen::Vector3d size =
        max_bound - min_bound;


    // ============================================================
    // PRINT DIMENSIONS
    // ============================================================

    cout << "\n";
    cout << "============================================\n";
    cout << "          TEMPLATE FRONT PLANE\n";
    cout << "============================================\n";

    cout << "Points : "
         << plane->points_.size()
         << "\n";

    cout << "\nDimensions:\n";

    cout << "X : "
         << size(0)
         << " mm\n";

    cout << "Y : "
         << size(1)
         << " mm\n";

    cout << "Z : "
         << size(2)
         << " mm\n";


    // ============================================================
    // SAVE
    // ============================================================

    if (!io::WritePointCloud(
            output_file,
            *plane))
    {
        cerr << "\nERROR: Could not save output.\n";
        return -1;
    }


    cout << "\nSaved successfully:\n";
    cout << output_file << "\n";


    // ============================================================
    // COLOR FOR VISUALIZATION
    // ============================================================

    plane->PaintUniformColor(
        Eigen::Vector3d(
            0.0,
            1.0,
            0.0
        )
    );


    // ============================================================
    // COORDINATE FRAME
    // ============================================================

    auto coordinate =
        geometry::TriangleMesh::
        CreateCoordinateFrame(
            100.0
        );


    // ============================================================
    // VIEW
    // ============================================================

    cout << "\nOpening Open3D viewer...\n";

    visualization::DrawGeometries(
        {
            plane,
            coordinate
        },
        "Template Front Plane",
        1280,
        720
    );


    cout << "\n============================================\n";
    cout << "                 DONE\n";
    cout << "============================================\n";


    return 0;
}