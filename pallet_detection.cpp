#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using namespace open3d;
namespace fs = std::filesystem;

// ============================================================
// ONE-FILE PALLET DETECTION PIPELINE
//
// Usage:
//
// ./pallet_detection <scene_roi.pcd> <template.pcd>
//
// Example:
//
// ./pallet_detection ../roi.pcd ../templates/pallet_1.pcd
//
// Pipeline:
//
// ROI
//  |
//  +--> meters -> millimeters
//  |
//  +--> DBSCAN
//  |
//  +--> automatic pallet cluster
//  |
//  +--> fork-entry extraction
//  |
//  +--> template front-plane extraction
//  |
//  +--> PCA initial alignment
//  |
//  +--> ICP
//  |
//  +--> final aligned template
//
// Only final aligned_template.pcd is written.
// Intermediate clouds remain in memory.
// ============================================================


// ============================================================
// BASIC UTILITIES
// ============================================================

static bool isFinitePoint(const Eigen::Vector3d& p)
{
    return std::isfinite(p.x()) &&
           std::isfinite(p.y()) &&
           std::isfinite(p.z());
}
static void removeInvalidPoints(
    std::shared_ptr<geometry::PointCloud>& cloud)
{
    std::vector<size_t> indices;

    indices.reserve(cloud->points_.size());

    for (size_t i = 0;
         i < cloud->points_.size();
         ++i)
    {
        if (isFinitePoint(cloud->points_[i]))
        {
            indices.push_back(i);
        }
    }

    cloud =
        cloud->SelectByIndex(indices);
}


// ============================================================
// CONVERT METERS -> MILLIMETERS
// ============================================================

static void convertMetersToMillimeters(
    geometry::PointCloud& cloud)
{
    for (auto& p : cloud.points_)
    {
        p *= 1000.0;
    }
}


// ============================================================
// SORTED DIMENSIONS
// ============================================================

static Eigen::Vector3d getSortedDimensions(
    const geometry::PointCloud& cloud)
{
    Eigen::Vector3d size =
        cloud.GetMaxBound() -
        cloud.GetMinBound();

    std::vector<double> d =
    {
        std::abs(size.x()),
        std::abs(size.y()),
        std::abs(size.z())
    };

    std::sort(d.begin(), d.end());

    return Eigen::Vector3d(
        d[0],
        d[1],
        d[2]);
}


// ============================================================
// STEP 2
// DBSCAN + AUTOMATIC PALLET CLUSTER
// ============================================================

struct ClusterCandidate
{
    int label = -1;

    size_t points = 0;

    Eigen::Vector3d dimensions =
        Eigen::Vector3d::Zero();

    double score =
        std::numeric_limits<double>::max();
};


static std::shared_ptr<geometry::PointCloud>
selectPalletCluster(
    const std::shared_ptr<geometry::PointCloud>& cloud)
{
    std::cout
        << "\n============================================\n"
        << "        STEP 2 : DBSCAN CLUSTERING\n"
        << "============================================\n";

    // Same working values from your current DBSCAN program.
    const double EPS =
        50.0;

    const int MIN_POINTS =
        10;

    std::cout
        << "EPS        : "
        << EPS
        << " mm\n";

    std::cout
        << "Min points : "
        << MIN_POINTS
        << "\n";

    std::vector<int> labels =
        cloud->ClusterDBSCAN(
            EPS,
            MIN_POINTS,
            true);

    int max_label = -1;

    size_t noise_points = 0;

    for (int label : labels)
    {
        if (label == -1)
        {
            ++noise_points;
        }
        else
        {
            max_label =
                std::max(
                    max_label,
                    label);
        }
    }

    int cluster_count =
        max_label + 1;

    std::cout
        << "\nClusters found : "
        << cluster_count
        << "\n";

    std::cout
        << "Noise points   : "
        << noise_points
        << "\n";

    if (cluster_count <= 0)
    {
        std::cerr
            << "ERROR: DBSCAN found no clusters.\n";

        return nullptr;
    }


    // --------------------------------------------------------
    // Expected pallet dimensions.
    //
    // These are only used to identify the pallet cluster.
    // --------------------------------------------------------

    const double EXPECTED_SMALL =
        160.0;

    const double EXPECTED_MIDDLE =
        850.0;

    const double EXPECTED_LARGE =
        1000.0;

    const double SMALL_TOLERANCE =
        100.0;

    const double MIDDLE_TOLERANCE =
        250.0;

    const double LARGE_TOLERANCE =
        200.0;


    std::vector<ClusterCandidate>
        candidates;


    // --------------------------------------------------------
    // Analyze every cluster
    // --------------------------------------------------------

    for (int label = 0;
         label < cluster_count;
         ++label)
    {
        std::vector<size_t>
            indices;

        for (size_t i = 0;
             i < labels.size();
             ++i)
        {
            if (labels[i] == label)
            {
                indices.push_back(i);
            }
        }

        if (indices.size() <
            static_cast<size_t>(MIN_POINTS))
        {
            continue;
        }


        auto cluster =
            cloud->SelectByIndex(
                indices);

        if (cluster->IsEmpty())
        {
            continue;
        }


        Eigen::Vector3d d =
            getSortedDimensions(
                *cluster);


        double small_error =
            std::abs(
                d(0) -
                EXPECTED_SMALL)
            /
            SMALL_TOLERANCE;


        double middle_error =
            std::abs(
                d(1) -
                EXPECTED_MIDDLE)
            /
            MIDDLE_TOLERANCE;


        double large_error =
            std::abs(
                d(2) -
                EXPECTED_LARGE)
            /
            LARGE_TOLERANCE;


        double score =
            1.0 * small_error
            +
            1.5 * middle_error
            +
            2.0 * large_error;


        ClusterCandidate candidate;

        candidate.label =
            label;

        candidate.points =
            cluster->points_.size();

        candidate.dimensions =
            d;

        candidate.score =
            score;

        candidates.push_back(
            candidate);


        std::cout
            << "\nCluster "
            << label
            << "\n";

        std::cout
            << "Points : "
            << cluster->points_.size()
            << "\n";

        std::cout
            << "Dimensions : "
            << d(0)
            << " x "
            << d(1)
            << " x "
            << d(2)
            << " mm\n";

        std::cout
            << "Score : "
            << score
            << "\n";
    }


    if (candidates.empty())
    {
        std::cerr
            << "\nERROR: No valid DBSCAN clusters.\n";

        return nullptr;
    }


    // --------------------------------------------------------
    // Select lowest score
    // --------------------------------------------------------

    auto best =
        std::min_element(
            candidates.begin(),
            candidates.end(),
            [](const ClusterCandidate& a,
               const ClusterCandidate& b)
            {
                return a.score <
                       b.score;
            });


    std::cout
        << "\n============================================\n"
        << "       AUTOMATIC PALLET CLUSTER\n"
        << "============================================\n";

    std::cout
        << "Selected label : "
        << best->label
        << "\n";

    std::cout
        << "Points         : "
        << best->points
        << "\n";

    std::cout
        << "Dimensions     : "
        << best->dimensions(0)
        << " x "
        << best->dimensions(1)
        << " x "
        << best->dimensions(2)
        << " mm\n";

    std::cout
        << "Score          : "
        << best->score
        << "\n";


    std::vector<size_t>
        best_indices;

    for (size_t i = 0;
         i < labels.size();
         ++i)
    {
        if (labels[i] ==
            best->label)
        {
            best_indices.push_back(i);
        }
    }


    return cloud->SelectByIndex(
        best_indices);
}


// ============================================================
// PLANE INFORMATION
// ============================================================

struct PlaneInfo
{
    Eigen::Vector4d model =
        Eigen::Vector4d::Zero();

    std::vector<size_t>
        inliers;

    Eigen::Vector3d normal =
        Eigen::Vector3d::Zero();

    double xmin = 0.0;
    double xmax = 0.0;

    double ymin = 0.0;
    double ymax = 0.0;

    double zmin = 0.0;
    double zmax = 0.0;

    double xsize = 0.0;
    double ysize = 0.0;
    double zsize = 0.0;

    Eigen::Vector3d center =
        Eigen::Vector3d::Zero();

    double score = -1e9;
};


// ============================================================
// COMPUTE PLANE BOUNDS
// ============================================================

static void computePlaneBounds(
    const std::shared_ptr<geometry::PointCloud>& cloud,
    PlaneInfo& plane)
{
    if (!cloud ||
        cloud->IsEmpty())
    {
        return;
    }


    plane.xmin =
        plane.ymin =
        plane.zmin =
        std::numeric_limits<double>::max();


    plane.xmax =
        plane.ymax =
        plane.zmax =
        std::numeric_limits<double>::lowest();


    Eigen::Vector3d sum =
        Eigen::Vector3d::Zero();


    for (const auto& p :
         cloud->points_)
    {
        plane.xmin =
            std::min(
                plane.xmin,
                p.x());

        plane.xmax =
            std::max(
                plane.xmax,
                p.x());


        plane.ymin =
            std::min(
                plane.ymin,
                p.y());

        plane.ymax =
            std::max(
                plane.ymax,
                p.y());


        plane.zmin =
            std::min(
                plane.zmin,
                p.z());

        plane.zmax =
            std::max(
                plane.zmax,
                p.z());


        sum += p;
    }


    plane.xsize =
        plane.xmax -
        plane.xmin;

    plane.ysize =
        plane.ymax -
        plane.ymin;

    plane.zsize =
        plane.zmax -
        plane.zmin;


    plane.center =
        sum /
        static_cast<double>(
            cloud->points_.size());
}


// ============================================================
// NORMALIZE PLANE
// ============================================================

static Eigen::Vector3d normalizePlaneNormal(
    const Eigen::Vector4d& model)
{
    Eigen::Vector3d n(
        model(0),
        model(1),
        model(2));

    if (n.norm() <
        1e-12)
    {
        return Eigen::Vector3d::Zero();
    }

    return n.normalized();
}


// ============================================================
// STEP 3
// FORK ENTRY EXTRACTION
// ============================================================

static std::shared_ptr<geometry::PointCloud>
extractForkEntry(
    const std::shared_ptr<geometry::PointCloud>& input)
{
    std::cout
        << "\n============================================\n"
        << "        STEP 3 : FORK ENTRY\n"
        << "============================================\n";


    auto remaining =
        std::make_shared<
            geometry::PointCloud>(
                *input);


    std::vector<PlaneInfo>
        planes;


    const double RANSAC_THRESHOLD =
        15.0;

    const int MAX_PLANES =
        6;

    const size_t MIN_PLANE_POINTS =
        50;


    double global_z_min =
        input->GetMinBound().z();

    double global_z_max =
        input->GetMaxBound().z();


    double z_range =
        global_z_max -
        global_z_min;


    // --------------------------------------------------------
    // Extract multiple planes
    // --------------------------------------------------------

    for (int plane_id = 0;
         plane_id < MAX_PLANES;
         ++plane_id)
    {
        if (remaining->points_.size() <
            MIN_PLANE_POINTS)
        {
            break;
        }


        auto result =
            remaining->SegmentPlane(
                RANSAC_THRESHOLD,
                3,
                2000);


        Eigen::Vector4d model =
            std::get<0>(result);

        std::vector<size_t>
            inliers =
            std::get<1>(result);


        if (inliers.size() <
            MIN_PLANE_POINTS)
        {
            break;
        }


        PlaneInfo info;

        info.model =
            model;

        info.inliers =
            inliers;

        info.normal =
            normalizePlaneNormal(
                model);


        auto plane_cloud =
            remaining->SelectByIndex(
                inliers);


        computePlaneBounds(
            plane_cloud,
            info);


        planes.push_back(
            info);


        std::cout
            << "\nPlane "
            << plane_id
            << "\n";

        std::cout
            << "Points : "
            << info.inliers.size()
            << "\n";

        std::cout
            << "Normal : ("
            << info.normal.x()
            << ", "
            << info.normal.y()
            << ", "
            << info.normal.z()
            << ")\n";

        std::cout
            << "X : "
            << info.xsize
            << " mm\n";

        std::cout
            << "Y : "
            << info.ysize
            << " mm\n";

        std::cout
            << "Z : "
            << info.zsize
            << " mm\n";


        remaining =
            remaining->SelectByIndex(
                inliers,
                true);
    }


    if (planes.empty())
    {
        std::cerr
            << "ERROR: No planes detected.\n";

        return nullptr;
    }


    // --------------------------------------------------------
    // Largest X plane
    // --------------------------------------------------------

    double max_xsize =
        0.0;

    for (const auto& p :
         planes)
    {
        max_xsize =
            std::max(
                max_xsize,
                p.xsize);
    }


    // --------------------------------------------------------
    // Select top horizontal plane
    // --------------------------------------------------------

    double best_score =
        -std::numeric_limits<double>::max();

    int best_plane =
        -1;


    for (size_t i = 0;
         i < planes.size();
         ++i)
    {
        auto& p =
            planes[i];


        double abs_nz =
            std::abs(
                p.normal.z());


        double abs_ny =
            std::abs(
                p.normal.y());


        double x_score =
            std::min(
                p.xsize / 1000.0,
                1.0);


        double horizontal_score =
            abs_nz;


        double thickness_score =
            1.0 /
            (
                1.0 +
                p.zsize / 80.0
            );


        double z_position =
            0.0;


        if (z_range >
            1e-6)
        {
            z_position =
                (
                    p.center.z()
                    -
                    global_z_min
                )
                /
                z_range;
        }


        double vertical_penalty =
            abs_ny;


        double score =
            100.0 *
            horizontal_score

            +

            70.0 *
            x_score

            +

            35.0 *
            thickness_score

            +

            40.0 *
            z_position

            -

            80.0 *
            vertical_penalty;


        if (abs_nz < 0.75)
        {
            score -=
                100.0;
        }


        if (max_xsize > 0.0 &&
            p.xsize <
                0.60 *
                max_xsize)
        {
            score -=
                50.0;
        }


        p.score =
            score;


        std::cout
            << "Plane "
            << i
            << " score : "
            << score
            << "\n";


        if (score >
            best_score)
        {
            best_score =
                score;

            best_plane =
                static_cast<int>(i);
        }
    }


    if (best_plane < 0)
    {
        return nullptr;
    }


    const PlaneInfo& selected =
        planes[best_plane];


    std::cout
        << "\nSelected fork-entry plane : "
        << best_plane
        << "\n";


    // --------------------------------------------------------
    // Expand selected plane
    // --------------------------------------------------------

    const double expansion_distance =
        std::max(
            35.0,
            std::min(
                70.0,
                selected.zsize *
                1.5));


    auto fork_entry =
        std::make_shared<
            geometry::PointCloud>();


    for (const auto& p :
         input->points_)
    {
        double distance =
            std::abs(
                selected.model(0) *
                p.x()

                +

                selected.model(1) *
                p.y()

                +

                selected.model(2) *
                p.z()

                +

                selected.model(3));


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
            fork_entry->points_.push_back(
                p);
        }
    }


    // --------------------------------------------------------
    // Remove isolated points
    // --------------------------------------------------------

    if (fork_entry->points_.size() >=
        10)
    {
        const double radius =
            35.0;

        const size_t minimum_neighbors =
            2;


        geometry::KDTreeFlann tree(
            *fork_entry);


        std::vector<size_t>
            keep_indices;


        for (size_t i = 0;
             i < fork_entry->points_.size();
             ++i)
        {
            std::vector<int>
                ids;

            std::vector<double>
                distances;


            int count =
                tree.SearchRadius(
                    fork_entry->points_[i],
                    radius,
                    ids,
                    distances);


            if (count >=
                static_cast<int>(
                    minimum_neighbors + 1))
            {
                keep_indices.push_back(i);
            }
        }


        if (keep_indices.size() >=
            20)
        {
            fork_entry =
                fork_entry->SelectByIndex(
                    keep_indices);
        }
    }


    if (fork_entry->points_.size() <
        20)
    {
        std::cerr
            << "ERROR: Fork-entry extraction failed.\n";

        return nullptr;
    }


    std::cout
        << "\nFork-entry points : "
        << fork_entry->points_.size()
        << "\n";


    return fork_entry;
}


// ============================================================
// STEP 4
// TEMPLATE FRONT PLANE
// ============================================================

struct TemplatePlaneCandidate
{
    std::shared_ptr<
        geometry::PointCloud>
        cloud;

    Eigen::Vector3d normal =
        Eigen::Vector3d::Zero();

    double xsize = 0.0;
    double ysize = 0.0;
    double zsize = 0.0;

    double score = 0.0;

    size_t points = 0;
};


static std::shared_ptr<geometry::PointCloud>
extractTemplateFrontPlane(
    const std::shared_ptr<geometry::PointCloud>& input)
{
    std::cout
        << "\n============================================\n"
        << "     STEP 4 : TEMPLATE FRONT PLANE\n"
        << "============================================\n";


    auto remaining =
        std::make_shared<
            geometry::PointCloud>(
                *input);


    std::vector<
        TemplatePlaneCandidate>
        candidates;


    const double RANSAC_DISTANCE =
        3.0;

    const int RANSAC_ITERATIONS =
        2000;

    const size_t MIN_POINTS =
        40;

    const int MAX_PLANES =
        10;


    // --------------------------------------------------------
    // Detect planes
    // --------------------------------------------------------

    for (int i = 0;
         i < MAX_PLANES;
         ++i)
    {
        if (remaining->points_.size() <
            MIN_POINTS)
        {
            break;
        }


        auto result =
            remaining->SegmentPlane(
                RANSAC_DISTANCE,
                3,
                RANSAC_ITERATIONS);


        Eigen::Vector4d model =
            std::get<0>(result);


        std::vector<size_t>
            inliers =
            std::get<1>(result);


        if (inliers.size() <
            MIN_POINTS)
        {
            break;
        }


        auto plane =
            remaining->SelectByIndex(
                inliers);


        if (plane->IsEmpty())
        {
            break;
        }


        Eigen::Vector3d min_bound =
            plane->GetMinBound();

        Eigen::Vector3d max_bound =
            plane->GetMaxBound();

        Eigen::Vector3d dimensions =
            max_bound -
            min_bound;


        std::vector<double>
            dims =
        {
            std::abs(dimensions.x()),
            std::abs(dimensions.y()),
            std::abs(dimensions.z())
        };


        std::sort(
            dims.begin(),
            dims.end());


        double smallest =
            dims[0];

        double middle =
            dims[1];

        double largest =
            dims[2];


        Eigen::Vector3d normal(
            model(0),
            model(1),
            model(2));


        if (normal.norm() <
            1e-9)
        {
            continue;
        }


        normal.normalize();


        double area =
            largest *
            middle;


        double thinness =
            smallest /
            std::max(
                largest,
                1.0);


        double density =
            static_cast<double>(
                plane->points_.size())
            /
            std::max(
                area,
                1.0);


        double score =
            area
            +

            static_cast<double>(
                plane->points_.size())
            *
            50.0;


        if (thinness <
            0.02)
        {
            score +=
                area * 0.30;
        }
        else if (thinness <
                 0.05)
        {
            score +=
                area * 0.15;
        }


        score +=
            density *
            100000.0;


        TemplatePlaneCandidate c;

        c.cloud =
            plane;

        c.normal =
            normal;

        c.xsize =
            std::abs(
                dimensions.x());

        c.ysize =
            std::abs(
                dimensions.y());

        c.zsize =
            std::abs(
                dimensions.z());

        c.score =
            score;

        c.points =
            plane->points_.size();


        candidates.push_back(c);


        remaining =
            remaining->SelectByIndex(
                inliers,
                true);
    }


    if (candidates.empty())
    {
        std::cerr
            << "ERROR: No template planes found.\n";

        return nullptr;
    }


    // --------------------------------------------------------
    // Select best plane
    // --------------------------------------------------------

    auto best =
        std::max_element(
            candidates.begin(),
            candidates.end(),
            [](const TemplatePlaneCandidate& a,
               const TemplatePlaneCandidate& b)
            {
                return a.score <
                       b.score;
            });


    std::cout
        << "\nSelected template front plane\n";


    std::cout
        << "Points : "
        << best->points
        << "\n";


    std::cout
        << "Normal : ("
        << best->normal.x()
        << ", "
        << best->normal.y()
        << ", "
        << best->normal.z()
        << ")\n";


    std::cout
        << "X : "
        << best->xsize
        << " mm\n";


    std::cout
        << "Y : "
        << best->ysize
        << " mm\n";


    std::cout
        << "Z : "
        << best->zsize
        << " mm\n";


    return best->cloud;
}


// ============================================================
// PCA
// ============================================================

static Eigen::Matrix3d computePCA(
    const geometry::PointCloud& cloud)
{
    Eigen::Vector3d center =
        cloud.GetCenter();


    Eigen::Matrix3d covariance =
        Eigen::Matrix3d::Zero();


    for (const auto& p :
         cloud.points_)
    {
        Eigen::Vector3d q =
            p - center;

        covariance +=
            q *
            q.transpose();
    }


    covariance /=
        static_cast<double>(
            std::max<size_t>(
                1,
                cloud.points_.size()));


    Eigen::SelfAdjointEigenSolver<
        Eigen::Matrix3d>
        solver(
            covariance);


    Eigen::Matrix3d eigenvectors =
        solver.eigenvectors();


    Eigen::Matrix3d axes;


    axes.col(0) =
        eigenvectors.col(2);

    axes.col(1) =
        eigenvectors.col(1);

    axes.col(2) =
        eigenvectors.col(0);


    return axes;
}


// ============================================================
// PCA INITIAL TRANSFORM
// ============================================================

static Eigen::Matrix4d createInitialTransform(
    const geometry::PointCloud& source,
    const geometry::PointCloud& target,
    const Eigen::Matrix3d& source_axes,
    const Eigen::Matrix3d& target_axes,
    const Eigen::Vector3i& permutation,
    const Eigen::Vector3i& signs)
{
    Eigen::Matrix3d mapping =
        Eigen::Matrix3d::Zero();


    for (int target_axis = 0;
         target_axis < 3;
         ++target_axis)
    {
        int source_axis =
            permutation(
                target_axis);


        mapping(
            target_axis,
            source_axis) =
            static_cast<double>(
                signs(target_axis));
    }


    Eigen::Matrix3d R =
        target_axes *
        mapping *
        source_axes.transpose();


    if (R.determinant() <
        0.0)
    {
        R.col(2) *=
            -1.0;
    }


    Eigen::Vector3d source_center =
        source.GetCenter();

    Eigen::Vector3d target_center =
        target.GetCenter();


    Eigen::Vector3d translation =
        target_center -
        R * source_center;


    Eigen::Matrix4d T =
        Eigen::Matrix4d::Identity();


    T.block<3,3>(0,0) =
        R;

    T.block<3,1>(0,3) =
        translation;


    return T;
}


// ============================================================
// NEAREST NEIGHBOUR RMSE
// ============================================================

static double computeNearestRMSE(
    geometry::PointCloud& source,
    const geometry::PointCloud& target)
{
    if (source.IsEmpty() ||
        target.IsEmpty())
    {
        return std::numeric_limits<double>::max();
    }


    std::vector<double>
        distances =
        source.ComputePointCloudDistance(
            target);


    if (distances.empty())
    {
        return std::numeric_limits<double>::max();
    }


    double sum_squared =
        0.0;


    for (double d :
         distances)
    {
        sum_squared +=
            d * d;
    }


    return std::sqrt(
        sum_squared /
        static_cast<double>(
            distances.size()));
}


// ============================================================
// STEP 5 + STEP 6
// PCA + ICP
// ============================================================

struct ICPResult
{
    Eigen::Matrix4d transform =
        Eigen::Matrix4d::Identity();

    double fitness =
        0.0;

    double rmse =
        0.0;
};


static ICPResult runICP(
    const std::shared_ptr<geometry::PointCloud>& scene,
    const std::shared_ptr<geometry::PointCloud>& templ)
{
    std::cout
        << "\n============================================\n"
        << "          STEP 5 : PCA + ICP\n"
        << "============================================\n";


    const double VOXEL_SIZE =
        5.0;

    const double MAX_CORRESPONDENCE =
        80.0;


    // --------------------------------------------------------
    // Downsample
    // --------------------------------------------------------

    auto scene_down =
        scene->VoxelDownSample(
            VOXEL_SIZE);


    auto template_down =
        templ->VoxelDownSample(
            VOXEL_SIZE);


    std::cout
        << "Scene downsampled : "
        << scene_down->points_.size()
        << "\n";


    std::cout
        << "Template downsampled : "
        << template_down->points_.size()
        << "\n";


    if (scene_down->points_.size() <
            10
        ||
        template_down->points_.size() <
            10)
    {
        throw std::runtime_error(
            "Too few points after downsampling.");
    }


    // --------------------------------------------------------
    // PCA
    // --------------------------------------------------------

    Eigen::Matrix3d scene_axes =
        computePCA(
            *scene_down);


    Eigen::Matrix3d template_axes =
        computePCA(
            *template_down);


    // --------------------------------------------------------
    // All permutations
    // --------------------------------------------------------

    std::vector<Eigen::Vector3i>
        permutations =
    {
        Eigen::Vector3i(0,1,2),
        Eigen::Vector3i(0,2,1),
        Eigen::Vector3i(1,0,2),
        Eigen::Vector3i(1,2,0),
        Eigen::Vector3i(2,0,1),
        Eigen::Vector3i(2,1,0)
    };


    // --------------------------------------------------------
    // All signs
    // --------------------------------------------------------

    std::vector<Eigen::Vector3i>
        signs =
    {
        Eigen::Vector3i( 1, 1, 1),
        Eigen::Vector3i( 1, 1,-1),
        Eigen::Vector3i( 1,-1, 1),
        Eigen::Vector3i( 1,-1,-1),
        Eigen::Vector3i(-1, 1, 1),
        Eigen::Vector3i(-1, 1,-1),
        Eigen::Vector3i(-1,-1, 1),
        Eigen::Vector3i(-1,-1,-1)
    };


    // --------------------------------------------------------
    // Find best PCA orientation
    // --------------------------------------------------------

    double best_rmse =
        std::numeric_limits<double>::max();


    Eigen::Matrix4d best_transform =
        Eigen::Matrix4d::Identity();


    for (const auto& permutation :
         permutations)
    {
        for (const auto& sign :
             signs)
        {
            Eigen::Matrix4d T =
                createInitialTransform(
                    *template_down,
                    *scene_down,
                    template_axes,
                    scene_axes,
                    permutation,
                    sign);


            auto test_cloud =
                std::make_shared<
                    geometry::PointCloud>(
                        *template_down);


            test_cloud->Transform(
                T);


            double rmse =
                computeNearestRMSE(
                    *test_cloud,
                    *scene_down);


            if (rmse <
                best_rmse)
            {
                best_rmse =
                    rmse;

                best_transform =
                    T;
            }
        }
    }


    std::cout
        << "\nBest PCA RMSE : "
        << best_rmse
        << " mm\n";


    // --------------------------------------------------------
    // Apply PCA
    // --------------------------------------------------------

    auto aligned_template =
        std::make_shared<
            geometry::PointCloud>(
                *template_down);


    aligned_template->Transform(
        best_transform);


    // --------------------------------------------------------
    // ICP criteria
    // --------------------------------------------------------

    pipelines::registration::
        ICPConvergenceCriteria
        criteria;


    criteria.relative_fitness_ =
        1e-6;

    criteria.relative_rmse_ =
        1e-6;

    criteria.max_iteration_ =
        100;


    std::cout
        << "\nRunning ICP...\n";


    // --------------------------------------------------------
    // ICP
    // --------------------------------------------------------

    auto result =
        pipelines::registration::
        RegistrationICP(
            *aligned_template,
            *scene_down,
            MAX_CORRESPONDENCE,
            Eigen::Matrix4d::Identity(),
            pipelines::registration::
                TransformationEstimationPointToPoint(),
            criteria);


    ICPResult output;


    output.transform =
        result.transformation_ *
        best_transform;


    output.fitness =
        result.fitness_;


    output.rmse =
        result.inlier_rmse_;


    std::cout
        << "\n============================================\n"
        << "              ICP RESULT\n"
        << "============================================\n";


    std::cout
        << std::fixed
        << std::setprecision(6);


    std::cout
        << "Fitness : "
        << output.fitness
        << "\n";


    std::cout
        << "RMSE    : "
        << output.rmse
        << " mm\n";


    std::cout
        << "\nFinal transformation:\n"
        << output.transform
        << "\n";


    return output;
}


// ============================================================
// FINAL CENTER + ANGLE
// ============================================================

static void printFinalPose(
    const geometry::PointCloud& template_cloud,
    const Eigen::Matrix4d& transform)
{
    auto aligned =
        std::make_shared<
            geometry::PointCloud>(
                template_cloud);


    aligned->Transform(
        transform);


    Eigen::Vector3d center =
        aligned->GetCenter();


    Eigen::Matrix3d R =
        transform.block<3,3>(0,0);


    double angle_rad =
        std::atan2(
            R(1,0),
            R(0,0));


    double angle_deg =
        angle_rad *
        180.0 /
        M_PI;


    std::cout
        << "\n============================================\n"
        << "             FINAL DETECTION\n"
        << "============================================\n";


    std::cout
        << std::fixed
        << std::setprecision(3);


    std::cout
        << "Center X : "
        << center.x()
        << " mm\n";


    std::cout
        << "Center Y : "
        << center.y()
        << " mm\n";


    std::cout
        << "Center Z : "
        << center.z()
        << " mm\n";


    std::cout
        << "Angle Z  : "
        << angle_deg
        << " degrees\n";
}


// ============================================================
// MAIN
// ============================================================

int main(int argc, char** argv)
{
    std::cout
        << "\n============================================================\n"
        << "             ONE-FILE PALLET DETECTION\n"
        << "============================================================\n";


    // --------------------------------------------------------
    // COMMAND LINE
    // --------------------------------------------------------

    if (argc < 3)
    {
        std::cerr
            << "\nUsage:\n\n"
            << "  "
            << argv[0]
            << " <scene_roi.pcd> <template.pcd>\n\n"

            << "Example:\n\n"
            << "  "
            << argv[0]
            << " ../roi/roi.pcd "
            << "../templates/pallet_1.pcd\n";

        return 1;
    }


    const std::string scene_file =
        argv[1];


    const std::string template_file =
        argv[2];


    const std::string output_file =
        "../aligned_template.pcd";


    std::cout
        << "\nScene    : "
        << scene_file
        << "\n";


    std::cout
        << "Template : "
        << template_file
        << "\n";


    std::cout
        << "Output   : "
        << output_file
        << "\n";


    // ========================================================
    // STEP 1
    // LOAD SCENE
    // ========================================================

    std::cout
        << "\n============================================\n"
        << "          STEP 1 : LOAD SCENE\n"
        << "============================================\n";


    auto scene =
        std::make_shared<
            geometry::PointCloud>();


    if (!io::ReadPointCloud(
            scene_file,
            *scene))
    {
        std::cerr
            << "ERROR: Cannot read scene:\n"
            << scene_file
            << "\n";

        return 1;
    }


    removeInvalidPoints(
        scene);


    if (scene->IsEmpty())
    {
        std::cerr
            << "ERROR: Scene is empty.\n";

        return 1;
    }


    std::cout
        << "Input points : "
        << scene->points_.size()
        << "\n";


    // --------------------------------------------------------
    // Your ROI is currently in meters.
    // Convert to millimeters.
    // --------------------------------------------------------

    convertMetersToMillimeters(
        *scene);


    std::cout
        << "Converted scene: meters -> millimeters\n";


    // ========================================================
    // STEP 2
    // DBSCAN + PALLET SELECTION
    // ========================================================

    auto pallet_cluster =
        selectPalletCluster(
            scene);


    if (!pallet_cluster)
    {
        return 1;
    }


    // ========================================================
    // STEP 3
    // FORK ENTRY
    // ========================================================

    auto fork_entry =
        extractForkEntry(
            pallet_cluster);


    if (!fork_entry)
    {
        return 1;
    }


    // ========================================================
    // LOAD TEMPLATE
    // ========================================================

    std::cout
        << "\n============================================\n"
        << "          LOADING TEMPLATE\n"
        << "============================================\n";


    auto template_cloud =
        std::make_shared<
            geometry::PointCloud>();


    if (!io::ReadPointCloud(
            template_file,
            *template_cloud))
    {
        std::cerr
            << "ERROR: Cannot read template:\n"
            << template_file
            << "\n";

        return 1;
    }


    removeInvalidPoints(
        template_cloud);


    if (template_cloud->IsEmpty())
    {
        std::cerr
            << "ERROR: Template is empty.\n";

        return 1;
    }


    std::cout
        << "Template points : "
        << template_cloud->points_.size()
        << "\n";


    // ========================================================
    // STEP 4
    // TEMPLATE FRONT PLANE
    // ========================================================

    auto template_front =
        extractTemplateFrontPlane(
            template_cloud);


    if (!template_front)
    {
        return 1;
    }


    // ========================================================
    // STEP 5 + 6
    // PCA + ICP
    // ========================================================

    ICPResult icp;


    try
    {
        icp =
            runICP(
                fork_entry,
                template_front);
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "\nICP ERROR:\n"
            << e.what()
            << "\n";

        return 1;
    }


    // ========================================================
    // FINAL TRANSFORM
    // ========================================================

    auto aligned_template =
        std::make_shared<
            geometry::PointCloud>(
                *template_front);


    aligned_template->Transform(
        icp.transform);


    // ========================================================
    // SAVE OUTPUT
    // ========================================================

    if (!io::WritePointCloud(
            output_file,
            *aligned_template))
    {
        std::cerr
            << "\nERROR: Could not save:\n"
            << output_file
            << "\n";

        return 1;
    }


    // ========================================================
    // FINAL POSE
    // ========================================================

    printFinalPose(
        *template_front,
        icp.transform);


    // ========================================================
    // VIEWER
    // ========================================================

    std::cout
        << "\nOpening Open3D viewer...\n";


    pallet_cluster->PaintUniformColor(
        Eigen::Vector3d(
            0.0,
            1.0,
            0.0));


    aligned_template->PaintUniformColor(
        Eigen::Vector3d(
            1.0,
            0.0,
            0.0));


    visualization::DrawGeometries(
        {
            pallet_cluster,
            aligned_template
        },
        "ONE-FILE PALLET ICP",
        1280,
        720);


    // ========================================================
    // COMPLETE
    // ========================================================

    std::cout
        << "\n============================================================\n"
        << "                 PIPELINE COMPLETE\n"
        << "============================================================\n";


    std::cout
        << "Input scene : "
        << scene_file
        << "\n";


    std::cout
        << "Template    : "
        << template_file
        << "\n";


    std::cout
        << "Output      : "
        << output_file
        << "\n";


    std::cout
        << "============================================================\n";


    return 0;
}
