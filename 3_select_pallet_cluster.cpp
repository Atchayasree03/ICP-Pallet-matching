#include <open3d/Open3D.h>

#include <Eigen/Dense>

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <limits>

namespace fs = std::filesystem;

using namespace open3d;


// ============================================================
// CLUSTER INFORMATION
// ============================================================

struct ClusterInfo
{
    std::string filename;

    int index = -1;

    size_t points = 0;

    Eigen::Vector3d dimensions;

    double length = 0.0;
    double depth = 0.0;
    double height = 0.0;

    double score = 0.0;
};


// ============================================================
// GET SORTED DIMENSIONS
//
// After sorting:
//
// d[0] = smallest
// d[1] = middle
// d[2] = largest
//
// This makes the method independent of X/Y/Z orientation.
// ============================================================

Eigen::Vector3d getSortedDimensions(
    const geometry::PointCloud &cloud)
{
    Eigen::Vector3d min_bound =
        cloud.GetMinBound();

    Eigen::Vector3d max_bound =
        cloud.GetMaxBound();

    Eigen::Vector3d size =
        max_bound - min_bound;

    std::vector<double> d =
    {
        std::abs(size(0)),
        std::abs(size(1)),
        std::abs(size(2))
    };

    std::sort(d.begin(), d.end());

    return Eigen::Vector3d(
        d[0],
        d[1],
        d[2]);
}


// ============================================================
// MAIN
// ============================================================

int main()
{
    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "       AUTOMATIC PALLET CLUSTER SELECTION\n";
    std::cout << "============================================\n";


    // ========================================================
    // INPUT / OUTPUT DIRECTORY
    // ========================================================

    const std::string cluster_directory =
        "../clusters";

    const std::string output_file =
        "../selected_cluster.pcd";


    // ========================================================
    // EXPECTED PALLET DIMENSIONS
    //
    // Your pallet is approximately:
    //
    // Length = 1000 mm
    // Depth  = 850 mm
    // Height = 160 mm
    //
    // We compare sorted dimensions, so orientation
    // does not matter.
    // ========================================================

    const double EXPECTED_SMALL =
        160.0;

    const double EXPECTED_MIDDLE =
        850.0;

    const double EXPECTED_LARGE =
        1000.0;


    // ========================================================
    // TOLERANCES
    // ========================================================

    /*
        Real point clouds will not have exact dimensions.

        For example:

        Expected = 1000 mm
        Measured = 1028 mm

        is still acceptable.
    */

    const double SMALL_TOLERANCE =
        100.0;

    const double MIDDLE_TOLERANCE =
        250.0;

    const double LARGE_TOLERANCE =
        200.0;


    // ========================================================
    // FIND CLUSTER FILES
    // ========================================================

    if (!fs::exists(cluster_directory))
    {
        std::cerr
            << "\nERROR: Cluster directory does not exist:\n"
            << cluster_directory
            << "\n";

        return 1;
    }


    std::vector<fs::path> cluster_files;


    for (const auto &entry :
         fs::directory_iterator(cluster_directory))
    {
        if (!entry.is_regular_file())
            continue;

        std::string filename =
            entry.path().filename().string();

        /*
            Accept:

                cluster_0.pcd
                cluster_1.pcd
                cluster_2.pcd
                ...

            Ignore everything else.
        */

        if (filename.rfind(
                "cluster_",
                0) == 0 &&
            entry.path().extension() == ".pcd")
        {
            cluster_files.push_back(
                entry.path());
        }
    }


    if (cluster_files.empty())
    {
        std::cerr
            << "\nERROR: No cluster_*.pcd files found in:\n"
            << cluster_directory
            << "\n";

        return 1;
    }


    // Sort filenames
    std::sort(
        cluster_files.begin(),
        cluster_files.end());


    std::cout << "\nFound "
              << cluster_files.size()
              << " cluster files.\n";


    // ========================================================
    // INSPECT EVERY CLUSTER
    // ========================================================

    std::vector<ClusterInfo> clusters;


    for (const auto &path :
         cluster_files)
    {
        auto cloud =
            std::make_shared<
                geometry::PointCloud>();


        if (!io::ReadPointCloud(
                path.string(),
                *cloud))
        {
            std::cerr
                << "\nWARNING: Cannot read "
                << path.string()
                << "\n";

            continue;
        }


        cloud->RemoveNonFinitePoints();


        if (cloud->IsEmpty())
        {
            std::cout
                << "\nSkipping empty cluster: "
                << path.filename().string()
                << "\n";

            continue;
        }


        ClusterInfo info;

        info.filename =
            path.string();

        info.points =
            cloud->points_.size();


        // ----------------------------------------------------
        // GET SORTED DIMENSIONS
        // ----------------------------------------------------

        Eigen::Vector3d d =
            getSortedDimensions(*cloud);


        info.dimensions = d;

        info.height =
            d(0);

        info.depth =
            d(1);

        info.length =
            d(2);


        // ----------------------------------------------------
        // CALCULATE SCORE
        //
        // Lower = better
        // ----------------------------------------------------

        double small_error =
            std::abs(
                info.height -
                EXPECTED_SMALL)
            / SMALL_TOLERANCE;


        double middle_error =
            std::abs(
                info.depth -
                EXPECTED_MIDDLE)
            / MIDDLE_TOLERANCE;


        double large_error =
            std::abs(
                info.length -
                EXPECTED_LARGE)
            / LARGE_TOLERANCE;


        /*
            Length gets the highest weight because
            your pallet has a very characteristic
            ~1 meter length.
        */

        info.score =
            1.0 * small_error +
            1.5 * middle_error +
            2.0 * large_error;


        clusters.push_back(info);
    }


    if (clusters.empty())
    {
        std::cerr
            << "\nERROR: No valid clusters.\n";

        return 1;
    }


    // ========================================================
    // PRINT ALL CLUSTERS
    // ========================================================

    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "          CLUSTER ANALYSIS\n";
    std::cout << "============================================\n";


    std::cout << std::fixed
              << std::setprecision(3);


    for (size_t i = 0;
         i < clusters.size();
         ++i)
    {
        const auto &c =
            clusters[i];


        std::cout << "\n";
        std::cout << "--------------------------------------------\n";

        std::cout << "Cluster "
                  << i
                  << "\n";

        std::cout << "File   : "
                  << c.filename
                  << "\n";

        std::cout << "Points : "
                  << c.points
                  << "\n";


        std::cout << "\nSorted dimensions:\n";

        std::cout << "Smallest : "
                  << c.height
                  << " mm\n";

        std::cout << "Middle   : "
                  << c.depth
                  << " mm\n";

        std::cout << "Largest  : "
                  << c.length
                  << " mm\n";


        std::cout << "\nExpected pallet:\n";

        std::cout << "Smallest : "
                  << EXPECTED_SMALL
                  << " mm\n";

        std::cout << "Middle   : "
                  << EXPECTED_MIDDLE
                  << " mm\n";

        std::cout << "Largest  : "
                  << EXPECTED_LARGE
                  << " mm\n";


        std::cout << "\nScore : "
                  << c.score
                  << "\n";
    }


    // ========================================================
    // FIND BEST CLUSTER
    // ========================================================

    auto best_it =
        std::min_element(
            clusters.begin(),
            clusters.end(),
            [](const ClusterInfo &a,
               const ClusterInfo &b)
            {
                return a.score < b.score;
            });


    const ClusterInfo &best =
        *best_it;


    // ========================================================
    // PRINT SELECTION
    // ========================================================

    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "        AUTOMATIC PALLET SELECTION\n";
    std::cout << "============================================\n";


    std::cout << "\nSelected cluster file:\n";

    std::cout << best.filename
              << "\n";


    std::cout << "\nSelected cluster dimensions:\n";

    std::cout << "Smallest : "
              << best.height
              << " mm\n";

    std::cout << "Middle   : "
              << best.depth
              << " mm\n";

    std::cout << "Largest  : "
              << best.length
              << " mm\n";


    std::cout << "\nPoints : "
              << best.points
              << "\n";


    std::cout << "\nSelection score : "
              << best.score
              << "\n";


    // ========================================================
    // LOAD BEST CLUSTER AGAIN
    // ========================================================

    auto selected_cluster =
        std::make_shared<
            geometry::PointCloud>();


    if (!io::ReadPointCloud(
            best.filename,
            *selected_cluster))
    {
        std::cerr
            << "\nERROR: Cannot load selected cluster.\n";

        return 1;
    }


    selected_cluster->
        RemoveNonFinitePoints();


    // ========================================================
    // SAVE AS fork_entry.pcd
    // ========================================================

    if (!io::WritePointCloud(
            output_file,
            *selected_cluster))
    {
        std::cerr
            << "\nERROR: Cannot save selected cluster to:\n"
            << output_file
            << "\n";

        return 1;
    }


    // ========================================================
    // FINAL RESULT
    // ========================================================

    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "             SELECTION COMPLETE\n";
    std::cout << "============================================\n";

    std::cout << "\nSelected:\n";
    std::cout << best.filename
              << "\n";

    std::cout << "\nSaved as:\n";
    std::cout << output_file
              << "\n";

    std::cout << "\nThis file will now be used as:\n";
    std::cout << "SCENE -> selected_cluster.pcd\n";

    std::cout << "============================================\n";


    // ========================================================
    // VIEWER
    // ========================================================

    selected_cluster->PaintUniformColor(
        Eigen::Vector3d(
            0.0,
            1.0,
            0.0));


    visualization::DrawGeometries(
        {
            selected_cluster
        },
        "Automatically Selected Pallet Cluster",
        1280,
        720);


    return 0;
}
