#include <open3d/Open3D.h>

#include <Eigen/Dense>

#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>

using namespace open3d;


// ============================================================
// PLANE CANDIDATE
// ============================================================

struct PlaneCandidate
{
    std::shared_ptr<geometry::PointCloud> cloud;

    Eigen::Vector3d normal;
    Eigen::Vector3d min_bound;
    Eigen::Vector3d max_bound;
    Eigen::Vector3d dimensions;
    Eigen::Vector3d center;

    double largest;
    double middle;
    double smallest;

    double surface_area;
    double density;

    size_t points;

    double score;
};


// ============================================================
// SORT DIMENSIONS
// ============================================================

void sortDimensions(
    const Eigen::Vector3d &dims,
    double &smallest,
    double &middle,
    double &largest)
{
    std::vector<double> d =
    {
        std::abs(dims(0)),
        std::abs(dims(1)),
        std::abs(dims(2))
    };

    std::sort(d.begin(), d.end());

    smallest = d[0];
    middle   = d[1];
    largest  = d[2];
}


// ============================================================
// MAIN
// ============================================================

int main()
{
    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "   AUTOMATIC TEMPLATE FRONT PLANE\n";
    std::cout << "        GEOMETRY BASED\n";
    std::cout << "============================================\n";


    // ========================================================
    // INPUT / OUTPUT
    // ========================================================

    const std::string input_file =
        "../templates/pallet_1.pcd";

    const std::string output_file =
        "../template_front_plane.pcd";


    // ========================================================
    // PARAMETERS
    // ========================================================

    /*
        Point clouds are in millimeters.

        RANSAC distance threshold:
        points within 3 mm of a plane are considered
        plane inliers.
    */

    const double RANSAC_DISTANCE =
        3.0;

    const int RANSAC_N =
        3;

    const int RANSAC_ITERATIONS =
        2000;


    /*
        Ignore extremely small planes.
    */

    const size_t MIN_PLANE_POINTS =
        40;


    /*
        Maximum number of planes to detect.

        These planes are NOT written to disk.
        They exist only in memory.
    */

    const int MAX_PLANES =
        10;


    // ========================================================
    // LOAD TEMPLATE
    // ========================================================

    auto template_cloud =
        std::make_shared<
            geometry::PointCloud>();


    if (!io::ReadPointCloud(
            input_file,
            *template_cloud))
    {
        std::cerr
            << "\nERROR: Cannot read:\n"
            << input_file
            << "\n";

        return 1;
    }


    template_cloud->RemoveNonFinitePoints();


    if (template_cloud->IsEmpty())
    {
        std::cerr
            << "\nERROR: Template contains no points.\n";

        return 1;
    }


    std::cout
        << "\nInput : "
        << input_file
        << "\n";


    std::cout
        << "Points: "
        << template_cloud->points_.size()
        << "\n";


    // ========================================================
    // REMAINING CLOUD
    // ========================================================

    auto remaining =
        std::make_shared<
            geometry::PointCloud>(
                *template_cloud);


    std::vector<PlaneCandidate> candidates;


    // ========================================================
    // EXTRACT PLANES
    // ========================================================

    for (int plane_id = 0;
         plane_id < MAX_PLANES;
         ++plane_id)
    {
        if (remaining->points_.size() <
            MIN_PLANE_POINTS)
        {
            break;
        }


        // ----------------------------------------------------
        // RANSAC
        // ----------------------------------------------------

        auto result =
            remaining->SegmentPlane(
                RANSAC_DISTANCE,
                RANSAC_N,
                RANSAC_ITERATIONS);


        /*
            IMPORTANT:

            Open3D 0.19 returns:

            std::tuple<
                Eigen::Vector4d,
                std::vector<size_t>
            >

            Therefore use std::get<0>()
            and std::get<1>().
        */

        Eigen::Vector4d model =
            std::get<0>(result);


        std::vector<size_t> inliers =
            std::get<1>(result);


        if (inliers.size() <
            MIN_PLANE_POINTS)
        {
            break;
        }


        // ----------------------------------------------------
        // PLANE NORMAL
        // ----------------------------------------------------

        Eigen::Vector3d normal(
            model(0),
            model(1),
            model(2));


        if (normal.norm() < 1e-9)
        {
            break;
        }


        normal.normalize();


        // ----------------------------------------------------
        // EXTRACT PLANE
        //
        // Only in memory.
        // No plane files are created.
        // ----------------------------------------------------

        auto plane =
            remaining->SelectByIndex(
                inliers);


        plane->RemoveNonFinitePoints();


        if (plane->IsEmpty())
        {
            break;
        }


        // ----------------------------------------------------
        // BOUNDING BOX
        // ----------------------------------------------------

        Eigen::Vector3d min_bound =
            plane->GetMinBound();


        Eigen::Vector3d max_bound =
            plane->GetMaxBound();


        Eigen::Vector3d dimensions =
            max_bound - min_bound;


        // ----------------------------------------------------
        // SORT DIMENSIONS
        // ----------------------------------------------------

        double smallest = 0.0;
        double middle   = 0.0;
        double largest  = 0.0;


        sortDimensions(
            dimensions,
            smallest,
            middle,
            largest);


        // ----------------------------------------------------
        // SURFACE AREA
        //
        // Two largest dimensions represent the approximate
        // planar surface.
        // ----------------------------------------------------

        double surface_area =
            largest * middle;


        // ----------------------------------------------------
        // THICKNESS RATIO
        // ----------------------------------------------------

        double thickness_ratio =
            smallest /
            std::max(largest, 1.0);


        // ----------------------------------------------------
        // CENTER
        // ----------------------------------------------------

        Eigen::Vector3d center =
            plane->GetCenter();


        // ----------------------------------------------------
        // POINT DENSITY
        // ----------------------------------------------------

        double density =
            static_cast<double>(
                plane->points_.size())
            /
            std::max(
                surface_area,
                1.0);


        // ====================================================
        // GEOMETRY SCORE
        // ====================================================

        /*
            IMPORTANT:

            There are NO pallet dimensions here.

            We do NOT assume:

                1000 mm
                850 mm
                160 mm
                5 mm

            Instead we rank planes based on their
            own geometric properties.
        */

        double score = 0.0;


        // ----------------------------------------------------
        // LARGE SURFACE
        // ----------------------------------------------------

        score +=
            surface_area;


        // ----------------------------------------------------
        // MANY POINTS
        // ----------------------------------------------------

        score +=
            static_cast<double>(
                plane->points_.size())
            * 50.0;


        // ----------------------------------------------------
        // THIN PLANAR STRUCTURE
        // ----------------------------------------------------

        if (thickness_ratio < 0.02)
        {
            score +=
                surface_area * 0.30;
        }
        else if (thickness_ratio < 0.05)
        {
            score +=
                surface_area * 0.15;
        }


        // ----------------------------------------------------
        // POINT DENSITY
        // ----------------------------------------------------

        score +=
            density * 100000.0;


        // ====================================================
        // STORE CANDIDATE
        // ====================================================

        PlaneCandidate candidate;


        candidate.cloud =
            plane;


        candidate.normal =
            normal;


        candidate.min_bound =
            min_bound;


        candidate.max_bound =
            max_bound;


        candidate.dimensions =
            dimensions;


        candidate.center =
            center;


        candidate.largest =
            largest;


        candidate.middle =
            middle;


        candidate.smallest =
            smallest;


        candidate.surface_area =
            surface_area;


        candidate.density =
            density;


        candidate.points =
            plane->points_.size();


        candidate.score =
            score;


        candidates.push_back(
            candidate);


        // ====================================================
        // PRINT PLANE
        // ====================================================

        std::cout << "\n";
        std::cout
            << "--------------------------------------------\n";

        std::cout
            << "PLANE "
            << plane_id
            << "\n";


        std::cout
            << "Points : "
            << candidate.points
            << "\n";


        std::cout
            << std::fixed
            << std::setprecision(3);


        std::cout
            << "Normal : ("
            << normal(0)
            << ", "
            << normal(1)
            << ", "
            << normal(2)
            << ")\n";


        std::cout
            << "Size X : "
            << dimensions(0)
            << " mm\n";


        std::cout
            << "Size Y : "
            << dimensions(1)
            << " mm\n";


        std::cout
            << "Size Z : "
            << dimensions(2)
            << " mm\n";


        std::cout
            << "Largest  : "
            << largest
            << " mm\n";


        std::cout
            << "Middle   : "
            << middle
            << " mm\n";


        std::cout
            << "Smallest : "
            << smallest
            << " mm\n";


        std::cout
            << "Area     : "
            << surface_area
            << " mm^2\n";


        std::cout
            << "Density  : "
            << density
            << "\n";


        std::cout
            << "Score    : "
            << score
            << "\n";


        // ====================================================
        // REMOVE CURRENT PLANE
        // ====================================================

        std::vector<bool> is_inlier(
            remaining->points_.size(),
            false);


        for (size_t idx :
             inliers)
        {
            if (idx <
                is_inlier.size())
            {
                is_inlier[idx] = true;
            }
        }


        std::vector<size_t> outliers;


        outliers.reserve(
            remaining->points_.size()
            -
            inliers.size());


        for (size_t i = 0;
             i < remaining->points_.size();
             ++i)
        {
            if (!is_inlier[i])
            {
                outliers.push_back(i);
            }
        }


        remaining =
            remaining->SelectByIndex(
                outliers);
    }


    // ========================================================
    // CHECK CANDIDATES
    // ========================================================

    if (candidates.empty())
    {
        std::cerr
            << "\nERROR: No planes detected.\n";

        return 1;
    }


    // ========================================================
    // SORT PLANES
    //
    // Higher score = better candidate.
    // ========================================================

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const PlaneCandidate &a,
           const PlaneCandidate &b)
        {
            return a.score >
                   b.score;
        });


    // ========================================================
    // PRINT RANKING
    // ========================================================

    std::cout << "\n";
    std::cout
        << "============================================\n";

    std::cout
        << "             PLANE RANKING\n";

    std::cout
        << "============================================\n";


    for (size_t i = 0;
         i < candidates.size();
         ++i)
    {
        const auto &p =
            candidates[i];


        std::cout << "\n";

        std::cout
            << "Rank "
            << i + 1
            << "\n";


        std::cout
            << "Points : "
            << p.points
            << "\n";


        std::cout
            << "Normal : ("
            << p.normal(0)
            << ", "
            << p.normal(1)
            << ", "
            << p.normal(2)
            << ")\n";


        std::cout
            << "Dimensions : "
            << p.largest
            << " x "
            << p.middle
            << " x "
            << p.smallest
            << " mm\n";


        std::cout
            << "Area : "
            << p.surface_area
            << " mm^2\n";


        std::cout
            << "Score : "
            << p.score
            << "\n";
    }


    // ========================================================
    // BEST PLANE
    // ========================================================

    const PlaneCandidate &best =
        candidates[0];


    // ========================================================
    // FINAL RESULT
    // ========================================================

    std::cout << "\n";
    std::cout
        << "============================================\n";

    std::cout
        << "       SELECTED TEMPLATE FRONT PLANE\n";

    std::cout
        << "============================================\n";


    std::cout
        << "Points : "
        << best.points
        << "\n";


    std::cout
        << "Normal : ("
        << best.normal(0)
        << ", "
        << best.normal(1)
        << ", "
        << best.normal(2)
        << ")\n";


    std::cout
        << "\nDimensions:\n";


    std::cout
        << "Largest  : "
        << best.largest
        << " mm\n";


    std::cout
        << "Middle   : "
        << best.middle
        << " mm\n";


    std::cout
        << "Smallest : "
        << best.smallest
        << " mm\n";


    std::cout
        << "Surface area : "
        << best.surface_area
        << " mm^2\n";


    std::cout
        << "Score : "
        << best.score
        << "\n";


    // ========================================================
    // SAVE ONLY THE SELECTED FRONT PLANE
    // ========================================================

    if (!io::WritePointCloud(
            output_file,
            *best.cloud))
    {
        std::cerr
            << "\nERROR: Could not save:\n"
            << output_file
            << "\n";

        return 1;
    }


    std::cout
        << "\nSaved:\n"
        << output_file
        << "\n";


    // ========================================================
    // COLOR SELECTED PLANE GREEN
    // ========================================================

    best.cloud->PaintUniformColor(
        Eigen::Vector3d(
            0.0,
            1.0,
            0.0));


    // ========================================================
    // VIEWER
    // ========================================================

    std::cout
        << "\nOpening Open3D viewer...\n";


    visualization::DrawGeometries(
        {
            best.cloud
        },
        "Automatically Selected Template Front Plane",
        1280,
        720);


    std::cout
        << "\nViewer closed.\n";


    return 0;
}