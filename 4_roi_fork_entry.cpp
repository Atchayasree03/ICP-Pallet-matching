#include <open3d/Open3D.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <tuple>
#include <vector>

using namespace open3d;

struct PlaneInfo
{
    Eigen::Vector4d model;
    std::vector<size_t> inliers;

    Eigen::Vector3d normal;

    double xmin = 0.0;
    double xmax = 0.0;
    double ymin = 0.0;
    double ymax = 0.0;
    double zmin = 0.0;
    double zmax = 0.0;

    double xsize = 0.0;
    double ysize = 0.0;
    double zsize = 0.0;

    Eigen::Vector3d center;

    double score = -1e9;
};


// ============================================================
// Compute bounding box information
// ============================================================

static void ComputeBounds(
    const std::shared_ptr<geometry::PointCloud>& cloud,
    PlaneInfo& p)
{
    if (!cloud || cloud->points_.empty())
        return;

    p.xmin = p.ymin = p.zmin =
        std::numeric_limits<double>::max();

    p.xmax = p.ymax = p.zmax =
        std::numeric_limits<double>::lowest();

    Eigen::Vector3d sum(0.0, 0.0, 0.0);

    for (const auto& pt : cloud->points_)
    {
        p.xmin = std::min(p.xmin, pt.x());
        p.xmax = std::max(p.xmax, pt.x());

        p.ymin = std::min(p.ymin, pt.y());
        p.ymax = std::max(p.ymax, pt.y());

        p.zmin = std::min(p.zmin, pt.z());
        p.zmax = std::max(p.zmax, pt.z());

        sum += pt;
    }

    p.xsize = p.xmax - p.xmin;
    p.ysize = p.ymax - p.ymin;
    p.zsize = p.zmax - p.zmin;

    p.center = sum / static_cast<double>(cloud->points_.size());
}


// ============================================================
// Normalize plane normal
// ============================================================

static Eigen::Vector3d NormalizeNormal(
    const Eigen::Vector4d& model)
{
    Eigen::Vector3d n(
        model(0),
        model(1),
        model(2));

    double len = n.norm();

    if (len < 1e-12)
        return Eigen::Vector3d(0, 0, 0);

    return n / len;
}


// ============================================================
// Percentile
// ============================================================

static double Percentile(
    std::vector<double> values,
    double percentage)
{
    if (values.empty())
        return 0.0;

    std::sort(values.begin(), values.end());

    double pos =
        (percentage / 100.0) *
        static_cast<double>(values.size() - 1);

    size_t lower =
        static_cast<size_t>(std::floor(pos));

    size_t upper =
        static_cast<size_t>(std::ceil(pos));

    if (lower == upper)
        return values[lower];

    double alpha =
        pos - static_cast<double>(lower);

    return values[lower] * (1.0 - alpha)
         + values[upper] * alpha;
}


// ============================================================
// Print plane information
// ============================================================

static void PrintPlane(
    int id,
    const PlaneInfo& p)
{
    std::cout
        << "\n--------------------------------------------\n";

    std::cout
        << "Plane " << id << "\n";

    std::cout
        << "Points : "
        << p.inliers.size()
        << "\n";

    std::cout
        << "Normal : ("
        << p.normal.x() << ", "
        << p.normal.y() << ", "
        << p.normal.z() << ")\n";

    std::cout
        << "X size : "
        << p.xsize
        << " mm\n";

    std::cout
        << "Y size : "
        << p.ysize
        << " mm\n";

    std::cout
        << "Z size : "
        << p.zsize
        << " mm\n";

    std::cout
        << "Center : ("
        << p.center.x() << ", "
        << p.center.y() << ", "
        << p.center.z() << ")\n";

    std::cout
        << "Score  : "
        << p.score
        << "\n";
}


// ============================================================
// MAIN
// ============================================================

int main()
{
    std::cout
        << "\n============================================\n"
        << "      AUTOMATIC FORK ENTRY EXTRACTION\n"
        << "============================================\n\n";


    // --------------------------------------------------------
    // INPUT / OUTPUT
    // --------------------------------------------------------

    const std::string input_file =
        "../selected_cluster.pcd";

    const std::string output_file =
        "../roi_fork_entry.pcd";

    const std::string top_output_file =
        "../roi_top_portion.pcd";


    std::cout
        << "Input file  : "
        << input_file
        << "\n";

    std::cout
        << "Output file : "
        << output_file
        << "\n";


    // --------------------------------------------------------
    // READ CLOUD
    // --------------------------------------------------------

    auto cloud =
        std::make_shared<geometry::PointCloud>();

    if (!io::ReadPointCloud(
            input_file,
            *cloud))
    {
        std::cerr
            << "\nERROR: Could not read input cloud.\n";

        return 1;
    }


    std::cout
        << "Input points : "
        << cloud->points_.size()
        << "\n";


    if (cloud->points_.empty())
    {
        std::cerr
            << "ERROR: Input cloud is empty.\n";

        return 1;
    }


    // --------------------------------------------------------
    // REMOVE NON-FINITE POINTS
    // --------------------------------------------------------

    std::vector<size_t> valid_indices;

    valid_indices.reserve(
        cloud->points_.size());

    for (size_t i = 0;
         i < cloud->points_.size();
         ++i)
    {
        const auto& p =
            cloud->points_[i];

        if (std::isfinite(p.x()) &&
            std::isfinite(p.y()) &&
            std::isfinite(p.z()))
        {
            valid_indices.push_back(i);
        }
    }


    cloud =
        cloud->SelectByIndex(valid_indices);


    std::cout
        << "After cleaning : "
        << cloud->points_.size()
        << "\n";


    // --------------------------------------------------------
    // GLOBAL RANGES
    // --------------------------------------------------------

    std::vector<double> all_x;
    std::vector<double> all_y;
    std::vector<double> all_z;

    all_x.reserve(cloud->points_.size());
    all_y.reserve(cloud->points_.size());
    all_z.reserve(cloud->points_.size());

    for (const auto& p :
         cloud->points_)
    {
        all_x.push_back(p.x());
        all_y.push_back(p.y());
        all_z.push_back(p.z());
    }


    const double global_z_min =
        *std::min_element(
            all_z.begin(),
            all_z.end());

    const double global_z_max =
        *std::max_element(
            all_z.begin(),
            all_z.end());

    const double global_y_min =
        *std::min_element(
            all_y.begin(),
            all_y.end());

    const double global_y_max =
        *std::max_element(
            all_y.begin(),
            all_y.end());


    std::cout
        << "\nGLOBAL RANGE\n"
        << "--------------------------------------------\n";

    std::cout
        << "X : "
        << *std::min_element(all_x.begin(), all_x.end())
        << " -> "
        << *std::max_element(all_x.begin(), all_x.end())
        << " mm\n";

    std::cout
        << "Y : "
        << global_y_min
        << " -> "
        << global_y_max
        << " mm\n";

    std::cout
        << "Z : "
        << global_z_min
        << " -> "
        << global_z_max
        << " mm\n";


    // ========================================================
    // MULTI-PLANE EXTRACTION
    // ========================================================

    std::cout
        << "\n============================================\n"
        << "          MULTI-PLANE EXTRACTION\n"
        << "============================================\n";


    std::shared_ptr<geometry::PointCloud>
        remaining = cloud;

    std::vector<PlaneInfo> planes;


    const double ransac_threshold =
        15.0; // mm

    const int max_planes = 6;

    const size_t minimum_plane_points = 50;


    for (int plane_id = 0;
         plane_id < max_planes;
         ++plane_id)
    {
        if (remaining->points_.size() <
            minimum_plane_points)
        {
            break;
        }


        std::cout
            << "\nSearching plane "
            << plane_id
            << " ...\n";


        auto result =
            remaining->SegmentPlane(
                ransac_threshold,
                3,
                2000);


        // IMPORTANT:
        // Open3D 0.19 returns tuple.
        auto model =
            std::get<0>(result);

        auto inliers =
            std::get<1>(result);


        if (inliers.size() <
            minimum_plane_points)
        {
            std::cout
                << "Too few inliers: "
                << inliers.size()
                << "\n";

            break;
        }


        PlaneInfo info;

        info.model = model;
        info.inliers = inliers;

        info.normal =
            NormalizeNormal(model);


        auto plane_cloud =
            remaining->SelectByIndex(
                inliers);


        ComputeBounds(
            plane_cloud,
            info);


        planes.push_back(info);


        PrintPlane(
            plane_id,
            info);


        remaining =
            remaining->SelectByIndex(
                inliers,
                true);
    }


    if (planes.empty())
    {
        std::cerr
            << "\nERROR: No planes found.\n";

        return 1;
    }


    // ========================================================
    // AUTOMATIC TOP HORIZONTAL PLANE SCORING
    // ========================================================

    std::cout
        << "\n============================================\n"
        << "          TOP PLANE ANALYSIS\n"
        << "============================================\n";


    /*
        Desired structure:

        -----------------------------
              TOP FORK STRUCTURE
        -----------------------------

        Therefore we want a plane which:

        1. Has a strong Z normal
           -> approximately horizontal

        2. Has a large X dimension
           -> long across pallet

        3. Has a relatively small Z thickness
           -> thin top structure

        4. Is near the front Z region

        5. Is NOT selected merely because
           it has the largest number of points.
    */


    double best_score =
        -std::numeric_limits<double>::max();

    int best_plane = -1;


    const double z_range =
        global_z_max -
        global_z_min;


    for (size_t i = 0;
         i < planes.size();
         ++i)
    {
        auto& p =
            planes[i];


        const double abs_nz =
            std::abs(p.normal.z());


        const double abs_ny =
            std::abs(p.normal.y());


        // ----------------------------------------------------
        // X length score
        // ----------------------------------------------------

        double x_score =
            std::min(
                p.xsize / 1000.0,
                1.0);


        // ----------------------------------------------------
        // Horizontal-plane score
        // ----------------------------------------------------

        double horizontal_score =
            abs_nz;


        // ----------------------------------------------------
        // Thinness score
        // ----------------------------------------------------

        double thickness_score =
            1.0 /
            (1.0 +
             p.zsize / 80.0);


        // ----------------------------------------------------
        // Front-position score
        //
        // Higher Z gets higher score.
        //
        // This is NORMALIZED from the cloud.
        // No fixed Z coordinate.
        // ----------------------------------------------------

        double z_position =
            0.0;

        if (z_range > 1e-6)
        {
            z_position =
                (p.center.z() -
                 global_z_min)
                / z_range;
        }


        // ----------------------------------------------------
        // Y orientation penalty
        //
        // A vertical front wall has strong Y normal.
        // We don't want that.
        // ----------------------------------------------------

        double vertical_penalty =
            abs_ny;


        // ----------------------------------------------------
        // Final score
        // ----------------------------------------------------

        double score =
            100.0 * horizontal_score
          + 70.0  * x_score
          + 35.0  * thickness_score
          + 40.0  * z_position
          - 80.0  * vertical_penalty;


        /*
            Strong preference for a genuinely
            horizontal plane.

            This prevents Plane 0 from winning
            simply because it contains many points.
        */

        if (abs_nz < 0.75)
        {
            score -= 100.0;
        }


        /*
            The desired top structure should
            normally be long in X.
        */

        // Find the largest X dimension among all detected planes
double max_plane_xsize = 0.0;

for (const auto& plane : planes)
{
    if (plane.xsize > max_plane_xsize)
    {
        max_plane_xsize = plane.xsize;
    }
}

// Penalize planes that are significantly shorter
// than the largest detected pallet plane.
if (max_plane_xsize > 0.0 &&
    p.xsize < 0.60 * max_plane_xsize)
{
    score -= 50.0;
}


        p.score =
            score;


        std::cout
            << "\nPlane "
            << i
            << "\n";

        std::cout
            << "  |Nz|              : "
            << abs_nz
            << "\n";

        std::cout
            << "  X length          : "
            << p.xsize
            << " mm\n";

        std::cout
            << "  Z thickness       : "
            << p.zsize
            << " mm\n";

        std::cout
            << "  Z position        : "
            << z_position
            << "\n";

        std::cout
            << "  Final score       : "
            << score
            << "\n";


        if (score > best_score)
        {
            best_score = score;
            best_plane =
                static_cast<int>(i);
        }
    }


    if (best_plane < 0)
    {
        std::cerr
            << "\nERROR: Could not select "
            << "top plane.\n";

        return 1;
    }


    const PlaneInfo& selected =
        planes[best_plane];


    // ========================================================
    // SELECT TOP PLANE
    // ========================================================

    auto selected_plane =
        cloud->SelectByIndex(
            selected.inliers);


    std::cout
        << "\n============================================\n"
        << "        SELECTED TOP PLANE\n"
        << "============================================\n";


    std::cout
        << "Selected plane : "
        << best_plane
        << "\n";

    std::cout
        << "Points         : "
        << selected_plane->points_.size()
        << "\n";

    std::cout
        << "Normal         : ("
        << selected.normal.x()
        << ", "
        << selected.normal.y()
        << ", "
        << selected.normal.z()
        << ")\n";

    std::cout
        << "X size         : "
        << selected.xsize
        << " mm\n";

    std::cout
        << "Y size         : "
        << selected.ysize
        << " mm\n";

    std::cout
        << "Z size         : "
        << selected.zsize
        << " mm\n";

    std::cout
        << "Center         : ("
        << selected.center.x()
        << ", "
        << selected.center.y()
        << ", "
        << selected.center.z()
        << ")\n";


    // ========================================================
    // EXPAND PLANE INTO TOP PORTION
    // ========================================================

    /*
        RANSAC gives only points belonging to
        the mathematical surface.

        We now collect nearby points around
        that plane.

        This is important because you want
        the COMPLETE TOP PORTION, not just
        a thin mathematical surface.
    */


    const double expansion_distance =
        std::max(
            35.0,
            std::min(
                70.0,
                selected.zsize * 1.5));


    std::cout
        << "\n============================================\n"
        << "          TOP PORTION EXPANSION\n"
        << "============================================\n";

    std::cout
        << "Expansion distance : "
        << expansion_distance
        << " mm\n";


    auto top_portion =
        std::make_shared<
            geometry::PointCloud>();

    auto remaining_final =
        std::make_shared<
            geometry::PointCloud>();


    Eigen::Vector3d n =
        selected.normal;

    n.normalize();


    /*
        Plane equation:

        ax + by + cz + d = 0

        Distance of point from plane:

        |ax + by + cz + d|
        -----------------
        sqrt(a²+b²+c²)

        Our normal is normalized,
        therefore denominator = 1.
    */


    for (const auto& p :
         cloud->points_)
    {
        double distance =
            std::abs(
                selected.model(0) * p.x()
              + selected.model(1) * p.y()
              + selected.model(2) * p.z()
              + selected.model(3));


        /*
            Also require the point to be
            reasonably close in X.

            This prevents unrelated points
            far away from becoming part of
            the top portion.
        */

        bool x_inside =
            p.x() >=
                selected.xmin - 40.0
            &&
            p.x() <=
                selected.xmax + 40.0;


        if (distance <=
                expansion_distance
            &&
            x_inside)
        {
            top_portion->points_.push_back(p);
        }
        else
        {
            remaining_final->points_.push_back(p);
        }
    }


    std::cout
        << "Expanded top points : "
        << top_portion->points_.size()
        << "\n";


    // ========================================================
    // REMOVE ISOLATED POINTS
    // ========================================================

    /*
        The expansion can collect isolated
        noise points.

        Radius filtering keeps points that
        have neighboring points.
    */

    if (top_portion->points_.size() >= 10)
    {
        const double radius =
            35.0;

        const size_t min_neighbors =
            2;


        std::vector<size_t>
            keep_indices;


        std::vector<std::vector<int>>
            neighbors;


        top_portion->EstimateNormals(
            geometry::KDTreeSearchParamKNN(10));


        geometry::KDTreeFlann kdtree(
            *top_portion);


        for (size_t i = 0;
             i < top_portion->points_.size();
             ++i)
        {
            std::vector<int> ids;

            std::vector<double>
                distances;


            int count =
                kdtree.SearchRadius(
                    top_portion->points_[i],
                    radius,
                    ids,
                    distances);


            if (count >=
                static_cast<int>(
                    min_neighbors + 1))
            {
                keep_indices.push_back(i);
            }
        }


        if (keep_indices.size() >= 10)
        {
            top_portion =
                top_portion->SelectByIndex(
                    keep_indices);
        }
    }


    // ========================================================
    // FINAL VALIDATION
    // ========================================================

    if (top_portion->points_.size() < 20)
    {
        std::cerr
            << "\nERROR: Final top portion "
            << "contains too few points.\n";

        std::cerr
            << "The selected plane was found, "
            << "but its expansion was too small.\n";

        return 1;
    }


    // ========================================================
    // FINAL BOUNDS
    // ========================================================

    PlaneInfo final_info;

    ComputeBounds(
        top_portion,
        final_info);


    std::cout
        << "\n============================================\n"
        << "             FINAL RESULT\n"
        << "============================================\n";


    std::cout
        << "Top portion points : "
        << top_portion->points_.size()
        << "\n";

    std::cout
        << "X range            : "
        << final_info.xmin
        << " -> "
        << final_info.xmax
        << " mm\n";

    std::cout
        << "Y range            : "
        << final_info.ymin
        << " -> "
        << final_info.ymax
        << " mm\n";

    std::cout
        << "Z range            : "
        << final_info.zmin
        << " -> "
        << final_info.zmax
        << " mm\n";

    std::cout
        << "X size             : "
        << final_info.xsize
        << " mm\n";

    std::cout
        << "Y size             : "
        << final_info.ysize
        << " mm\n";

    std::cout
        << "Z size             : "
        << final_info.zsize
        << " mm\n";


    // ========================================================
    // SAVE
    // ========================================================

    bool saved1 =
        io::WritePointCloud(
            output_file,
            *top_portion);


    bool saved2 =
        io::WritePointCloud(
            top_output_file,
            *top_portion);


    if (!saved1 || !saved2)
    {
        std::cerr
            << "\nERROR: Failed to save "
            << "output point cloud.\n";

        return 1;
    }


    std::cout
        << "\nSaved:\n";

    std::cout
        << "  "
        << output_file
        << "\n";

    std::cout
        << "  "
        << top_output_file
        << "\n";


    // ========================================================
    // VISUALIZATION
    // ========================================================

    /*
        GREEN = selected top portion
        GRAY  = remaining pallet
    */


    top_portion->PaintUniformColor(
        Eigen::Vector3d(
            0.0,
            1.0,
            0.0));


    remaining_final->PaintUniformColor(
        Eigen::Vector3d(
            0.65,
            0.65,
            0.65));


    auto coordinate =
        geometry::TriangleMesh::
            CreateCoordinateFrame(
                150.0,
                Eigen::Vector3d(
                    0.0,
                    0.0,
                    0.0));


    std::vector<
        std::shared_ptr<
            const geometry::Geometry>>
        geometries;


    geometries.push_back(
        remaining_final);

    geometries.push_back(
        top_portion);

    geometries.push_back(
        coordinate);


    std::cout
        << "\n============================================\n"
        << "               VIEWER\n"
        << "============================================\n";

    std::cout
        << "GREEN = automatically selected "
        << "top fork-entry portion\n";

    std::cout
        << "GRAY  = remaining pallet points\n";

    std::cout
        << "\nOpening Open3D viewer...\n";


    visualization::DrawGeometries(
        geometries,
        "Automatic Fork Entry Extraction",
        1280,
        720);


    return 0;
}
