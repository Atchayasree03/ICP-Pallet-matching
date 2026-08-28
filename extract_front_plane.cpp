#include <iostream>
#include <memory>
#include <vector>
#include <iomanip>
#include <cmath>
#include <algorithm>

#include <open3d/Open3D.h>

using namespace open3d;

void printPlaneInfo(
    const geometry::PointCloud& plane_cloud,
    const Eigen::Vector4d& plane,
    int plane_id)
{
    // ---------------------------------------------------------
    // Plane normal
    // ---------------------------------------------------------

    Eigen::Vector3d normal(
        plane(0),
        plane(1),
        plane(2)
    );

    normal.normalize();


    // ---------------------------------------------------------
    // Bounding box
    // ---------------------------------------------------------

    Eigen::Vector3d min_bound =
        plane_cloud.GetMinBound();

    Eigen::Vector3d max_bound =
        plane_cloud.GetMaxBound();

    Eigen::Vector3d size =
        max_bound - min_bound;


    // ---------------------------------------------------------
    // Center
    // ---------------------------------------------------------

    Eigen::Vector3d center =
        plane_cloud.GetCenter();


    // ---------------------------------------------------------
    // Print
    // ---------------------------------------------------------

    std::cout
        << "\n--------------------------------------------\n";

    std::cout
        << "PLANE "
        << plane_id
        << "\n";

    std::cout
        << "--------------------------------------------\n";

    std::cout
        << "Points : "
        << plane_cloud.points_.size()
        << "\n";


    std::cout
        << "\nNormal:\n";

    std::cout
        << "Nx = "
        << normal(0)
        << "\n";

    std::cout
        << "Ny = "
        << normal(1)
        << "\n";

    std::cout
        << "Nz = "
        << normal(2)
        << "\n";


    std::cout
        << "\nDimensions:\n";

    std::cout
        << "X = "
        << size(0)
        << " mm\n";

    std::cout
        << "Y = "
        << size(1)
        << " mm\n";

    std::cout
        << "Z = "
        << size(2)
        << " mm\n";


    std::cout
        << "\nCenter:\n";

    std::cout
        << "X = "
        << center(0)
        << " mm\n";

    std::cout
        << "Y = "
        << center(1)
        << " mm\n";

    std::cout
        << "Z = "
        << center(2)
        << " mm\n";


    // ---------------------------------------------------------
    // Determine dominant normal direction
    // ---------------------------------------------------------

    double ax = std::abs(normal(0));
    double ay = std::abs(normal(1));
    double az = std::abs(normal(2));


    std::cout
        << "\nDominant normal direction: ";

    if (ax > ay && ax > az)
    {
        std::cout << "X";
    }
    else if (ay > ax && ay > az)
    {
        std::cout << "Y";
    }
    else
    {
        std::cout << "Z";
    }

    std::cout << "\n";


    // ---------------------------------------------------------
    // Planarity check
    // ---------------------------------------------------------

    double max_dimension =
        std::max({
            size(0),
            size(1),
            size(2)
        });

    double min_dimension =
        std::min({
            size(0),
            size(1),
            size(2)
        });

    std::cout
        << "\nThickness / smallest dimension: "
        << min_dimension
        << " mm\n";

    std::cout
        << "Largest dimension: "
        << max_dimension
        << " mm\n";
}


int main()
{
    // =========================================================
    // INPUT
    // =========================================================

    std::string input_file =
        "../clusters/cluster_0.pcd";


    // =========================================================
    // LOAD
    // =========================================================

    auto cloud =
        std::make_shared<geometry::PointCloud>();

    if (!io::ReadPointCloud(
            input_file,
            *cloud))
    {
        std::cerr
            << "ERROR: Cannot load "
            << input_file
            << std::endl;

        return -1;
    }


    std::cout
        << "\n============================================\n";

    std::cout
        << "       MULTI-PLANE PALLET ANALYSIS\n";

    std::cout
        << "============================================\n";


    std::cout
        << "\nInput file : "
        << input_file
        << "\n";

    std::cout
        << "Total points : "
        << cloud->points_.size()
        << "\n";


    // =========================================================
    // REMOVE INVALID POINTS
    // =========================================================

    auto clean_cloud =
        std::make_shared<geometry::PointCloud>(
            cloud->RemoveNonFinitePoints()
        );

    cloud = clean_cloud;


    // =========================================================
    // PARAMETERS
    // =========================================================

    double distance_threshold =
        15.0;        // mm

    int ransac_n =
        3;

    int iterations =
        3000;

    double probability =
        0.999;


    std::cout
        << "\nRANSAC distance threshold : "
        << distance_threshold
        << " mm\n";


    // =========================================================
    // WORKING CLOUD
    // =========================================================

    auto remaining =
        std::make_shared<geometry::PointCloud>(
            *cloud
        );


    // =========================================================
    // STORE PLANES
    // =========================================================

    std::vector<
        std::shared_ptr<geometry::PointCloud>
    > plane_clouds;


    std::vector<
        Eigen::Vector4d
    > plane_models;


    // =========================================================
    // FIND MULTIPLE PLANES
    // =========================================================

    const int max_planes = 6;

    for (int i = 0;
         i < max_planes;
         ++i)
    {
        // -----------------------------------------------------
        // Stop if too few points remain
        // -----------------------------------------------------

        if (remaining->points_.size() < 50)
        {
            break;
        }


        // -----------------------------------------------------
        // RANSAC
        // -----------------------------------------------------

        auto result =
            remaining->SegmentPlane(
                distance_threshold,
                ransac_n,
                iterations,
                probability
            );


        Eigen::Vector4d plane =
            std::get<0>(result);

        std::vector<size_t> inliers =
            std::get<1>(result);


        // -----------------------------------------------------
        // Reject very small planes
        // -----------------------------------------------------

        if (inliers.size() < 50)
        {
            break;
        }


        // -----------------------------------------------------
        // Extract plane
        // -----------------------------------------------------

        auto plane_cloud =
            remaining->SelectByIndex(
                inliers,
                false
            );


        // -----------------------------------------------------
        // Save information
        // -----------------------------------------------------

        plane_models.push_back(
            plane
        );

        plane_clouds.push_back(
            plane_cloud
        );


        // -----------------------------------------------------
        // Print information
        // -----------------------------------------------------

        printPlaneInfo(
            *plane_cloud,
            plane,
            i
        );


        // -----------------------------------------------------
        // Remove this plane
        // -----------------------------------------------------

        remaining =
            remaining->SelectByIndex(
                inliers,
                true
            );
    }


    // =========================================================
    // SUMMARY
    // =========================================================

    std::cout
        << "\n\n============================================\n";

    std::cout
        << "              PLANE SUMMARY\n";

    std::cout
        << "============================================\n";


    for (size_t i = 0;
         i < plane_clouds.size();
         ++i)
    {
        Eigen::Vector3d normal(
            plane_models[i](0),
            plane_models[i](1),
            plane_models[i](2)
        );

        normal.normalize();


        Eigen::Vector3d size =
            plane_clouds[i]->GetMaxBound()
            -
            plane_clouds[i]->GetMinBound();


        std::cout
            << "\nPlane "
            << i
            << ": ";

        std::cout
            << plane_clouds[i]->points_.size()
            << " points";

        std::cout
            << " | Normal = ("
            << normal(0)
            << ", "
            << normal(1)
            << ", "
            << normal(2)
            << ")";

        std::cout
            << " | Size = ("
            << size(0)
            << ", "
            << size(1)
            << ", "
            << size(2)
            << ") mm";
    }


    // =========================================================
    // VISUALIZATION
    // =========================================================

    std::vector<
        std::shared_ptr<const geometry::Geometry>
    > geometries;


    // ---------------------------------------------------------
    // Add each plane
    // ---------------------------------------------------------

    std::vector<
        Eigen::Vector3d
    > colors =
    {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
        {1.0, 1.0, 0.0},
        {1.0, 0.0, 1.0},
        {0.0, 1.0, 1.0}
    };


    for (size_t i = 0;
         i < plane_clouds.size();
         ++i)
    {
        plane_clouds[i]->PaintUniformColor(
            colors[i % colors.size()]
        );

        geometries.push_back(
            plane_clouds[i]
        );
    }


    // ---------------------------------------------------------
    // Add remaining points
    // ---------------------------------------------------------

    if (!remaining->points_.empty())
    {
        remaining->PaintUniformColor(
            Eigen::Vector3d(
                0.5,
                0.5,
                0.5
            )
        );

        geometries.push_back(
            remaining
        );
    }


    // ---------------------------------------------------------
    // Coordinate frame
    // ---------------------------------------------------------

    auto coordinate =
        geometry::TriangleMesh::
        CreateCoordinateFrame(
            200.0
        );

    geometries.push_back(
        coordinate
    );


    // =========================================================
    // VIEW
    // =========================================================

    std::cout
        << "\n\nOpening viewer...\n";


    visualization::DrawGeometries(
        geometries,
        "Pallet Multiple Planes",
        1280,
        720
    );


    std::cout
        << "\n============================================\n";

    std::cout
        << "                  DONE\n";

    std::cout
        << "============================================\n";


    return 0;
}