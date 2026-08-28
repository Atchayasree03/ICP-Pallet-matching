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

namespace fs = std::filesystem;

using namespace open3d;

// ============================================================
// ONE FILE - MULTI TEMPLATE PALLET DETECTION
//
// Usage:
//
// ./pallet_detection <scene_roi.pcd> <template_directory>
//
// Example:
//
// ./pallet_detection ../roi.pcd ../templates
//
// Template directory:
//
// templates/
//     pallet_1.pcd
//     pallet_2.pcd
//     pallet_3.pcd
//     pallet_4.pcd
//
// Scene ROI is assumed to be in METERS.
// Templates are assumed to be in MILLIMETERS.
//
// ============================================================


// ============================================================
// STRUCTURES
// ============================================================

struct PlaneInfo
{
    Eigen::Vector4d model =
        Eigen::Vector4d::Zero();

    std::vector<size_t> inliers;

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


struct ClusterInfo
{
    int label = -1;

    size_t points = 0;

    Eigen::Vector3d dimensions =
        Eigen::Vector3d::Zero();

    double score =
        std::numeric_limits<double>::max();
};


struct TemplateInfo
{
    std::string filename;

    std::shared_ptr<geometry::PointCloud>
        original;

    std::shared_ptr<geometry::PointCloud>
        front_plane;

    Eigen::Vector3d dimensions =
        Eigen::Vector3d::Zero();
};


struct ICPResult
{
    std::string template_name;

    double fitness = 0.0;

    double rmse =
        std::numeric_limits<double>::max();

    Eigen::Matrix4d transformation =
        Eigen::Matrix4d::Identity();

    std::shared_ptr<geometry::PointCloud>
        aligned_template;

    std::shared_ptr<geometry::PointCloud>
        template_front;
};


// ============================================================
// FINITE POINT CHECK
// ============================================================

static bool isFinitePoint(
    const Eigen::Vector3d& p)
{
    return
        std::isfinite(p.x()) &&
        std::isfinite(p.y()) &&
        std::isfinite(p.z());
}


// ============================================================
// REMOVE INVALID POINTS
// ============================================================

static void removeInvalidPoints(
    std::shared_ptr<geometry::PointCloud>& cloud)
{
    std::vector<size_t> indices;

    indices.reserve(
        cloud->points_.size());

    for (size_t i = 0;
         i < cloud->points_.size();
         ++i)
    {
        if (isFinitePoint(
                cloud->points_[i]))
        {
            indices.push_back(i);
        }
    }

    cloud =
        cloud->SelectByIndex(indices);
}


// ============================================================
// METERS -> MILLIMETERS
// ============================================================

static void metersToMillimeters(
    geometry::PointCloud& cloud)
{
    for (auto& p :
         cloud.points_)
    {
        p *= 1000.0;
    }
}


// ============================================================
// SORTED DIMENSIONS
// ============================================================

static Eigen::Vector3d sortedDimensions(
    const geometry::PointCloud& cloud)
{
    Eigen::Vector3d d =
        cloud.GetMaxBound() -
        cloud.GetMinBound();

    std::vector<double> values =
    {
        std::abs(d.x()),
        std::abs(d.y()),
        std::abs(d.z())
    };

    std::sort(
        values.begin(),
        values.end());

    return Eigen::Vector3d(
        values[0],
        values[1],
        values[2]);
}


// ============================================================
// PLANE BOUNDS
// ============================================================

static void calculatePlaneBounds(
    const std::shared_ptr<geometry::PointCloud>& cloud,
    PlaneInfo& p)
{
    if (!cloud ||
        cloud->IsEmpty())
    {
        return;
    }

    Eigen::Vector3d minb =
        cloud->GetMinBound();

    Eigen::Vector3d maxb =
        cloud->GetMaxBound();

    p.xmin = minb.x();
    p.ymin = minb.y();
    p.zmin = minb.z();

    p.xmax = maxb.x();
    p.ymax = maxb.y();
    p.zmax = maxb.z();

    p.xsize =
        p.xmax - p.xmin;

    p.ysize =
        p.ymax - p.ymin;

    p.zsize =
        p.zmax - p.zmin;

    p.center =
        cloud->GetCenter();
}


// ============================================================
// NORMALIZE NORMAL
// ============================================================

static Eigen::Vector3d normalizeNormal(
    const Eigen::Vector4d& model)
{
    Eigen::Vector3d n(
        model(0),
        model(1),
        model(2));

    if (n.norm() < 1e-9)
    {
        return Eigen::Vector3d::Zero();
    }

    return n.normalized();
}


// ============================================================
// STEP 1
// DBSCAN
// ============================================================

static std::shared_ptr<geometry::PointCloud>
runDBSCAN(
    const std::shared_ptr<geometry::PointCloud>& scene)
{
    std::cout
        << "\n================================================\n"
        << "                 STEP 1\n"
        << "             DBSCAN CLUSTERING\n"
        << "================================================\n";

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
        scene->ClusterDBSCAN(
            EPS,
            MIN_POINTS,
            true);

    int max_label = -1;

    size_t noise = 0;

    for (int label :
         labels)
    {
        if (label == -1)
        {
            noise++;
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
        << "\nClusters : "
        << cluster_count
        << "\n";

    std::cout
        << "Noise    : "
        << noise
        << "\n";

    if (cluster_count <= 0)
    {
        return nullptr;
    }


    // --------------------------------------------------------
    // For now collect all clusters.
    // Pallet cluster is selected after templates are loaded.
    // --------------------------------------------------------

    std::vector<
        std::shared_ptr<geometry::PointCloud>>
        clusters;

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
            static_cast<size_t>(
                MIN_POINTS))
        {
            continue;
        }

        auto cluster =
            scene->SelectByIndex(
                indices);

        clusters.push_back(
            cluster);
    }


    if (clusters.empty())
    {
        return nullptr;
    }


    // --------------------------------------------------------
    // Temporarily choose largest cluster.
    //
    // The final template-based selection happens later.
    // --------------------------------------------------------

    auto best =
        std::max_element(
            clusters.begin(),
            clusters.end(),
            [](const auto& a,
               const auto& b)
            {
                return a->points_.size() <
                       b->points_.size();
            });


    std::cout
        << "\nLargest cluster points : "
        << (*best)->points_.size()
        << "\n";

    std::cout
        << "Dimensions : "
        << sortedDimensions(**best).transpose()
        << " mm\n";


    return *best;
}


// ============================================================
// STEP 2
// FORK ENTRY
// ============================================================

static std::shared_ptr<geometry::PointCloud>
extractForkEntry(
    const std::shared_ptr<geometry::PointCloud>& input)
{
    std::cout
        << "\n================================================\n"
        << "                 STEP 2\n"
        << "             FORK ENTRY EXTRACTION\n"
        << "================================================\n";

    auto remaining =
        std::make_shared<
            geometry::PointCloud>(
                *input);

    std::vector<PlaneInfo>
        planes;


    const double threshold =
        15.0;

    const int max_planes =
        6;

    const size_t min_points =
        50;


    double global_z_min =
        input->GetMinBound().z();

    double global_z_max =
        input->GetMaxBound().z();

    double z_range =
        global_z_max -
        global_z_min;


    // --------------------------------------------------------
    // Detect planes
    // --------------------------------------------------------

    for (int i = 0;
         i < max_planes;
         ++i)
    {
        if (remaining->points_.size() <
            min_points)
        {
            break;
        }

        auto result =
            remaining->SegmentPlane(
                threshold,
                3,
                2000);

        Eigen::Vector4d model =
            std::get<0>(result);

        std::vector<size_t>
            inliers =
            std::get<1>(result);

        if (inliers.size() <
            min_points)
        {
            break;
        }

        auto plane =
            remaining->SelectByIndex(
                inliers);

        PlaneInfo info;

        info.model =
            model;

        info.inliers =
            inliers;

        info.normal =
            normalizeNormal(
                model);

        calculatePlaneBounds(
            plane,
            info);

        planes.push_back(
            info);

        remaining =
            remaining->SelectByIndex(
                inliers,
                true);
    }


    if (planes.empty())
    {
        std::cerr
            << "ERROR: No planes found.\n";

        return nullptr;
    }


    // --------------------------------------------------------
    // Find largest X plane
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
    // Score planes
    // --------------------------------------------------------

    double best_score =
        -1e9;

    int best_plane =
        -1;


    for (size_t i = 0;
         i < planes.size();
         ++i)
    {
        auto& p =
            planes[i];

        double nz =
            std::abs(
                p.normal.z());

        double ny =
            std::abs(
                p.normal.y());

        double x_score =
            std::min(
                p.xsize / 1000.0,
                1.0);

        double thickness_score =
            1.0 /
            (
                1.0 +
                p.zsize / 80.0
            );

        double z_position =
            0.0;

        if (z_range > 1e-6)
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


        double score =
            100.0 * nz
            +
            70.0 * x_score
            +
            35.0 * thickness_score
            +
            40.0 * z_position
            -
            80.0 * ny;


        if (nz < 0.75)
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
            << " score = "
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


    // --------------------------------------------------------
    // Expand around selected plane
    // --------------------------------------------------------

    const double expansion =
        std::max(
            35.0,
            std::min(
                70.0,
                selected.zsize * 1.5));


    auto fork_entry =
        std::make_shared<
            geometry::PointCloud>();


    for (const auto& p :
         input->points_)
    {
        double distance =
            std::abs(
                selected.model(0) * p.x()
                +
                selected.model(1) * p.y()
                +
                selected.model(2) * p.z()
                +
                selected.model(3));


        bool x_inside =
            p.x() >=
                selected.xmin - 40.0
            &&
            p.x() <=
                selected.xmax + 40.0;


        if (distance <= expansion &&
            x_inside)
        {
            fork_entry->points_.push_back(
                p);
        }
    }


    std::cout
        << "\nSelected plane : "
        << best_plane
        << "\n";

    std::cout
        << "Fork entry points : "
        << fork_entry->points_.size()
        << "\n";


    if (fork_entry->points_.size() <
        20)
    {
        std::cerr
            << "ERROR: Fork entry too small.\n";

        return nullptr;
    }


    return fork_entry;
}


// ============================================================
// STEP 3
// TEMPLATE FRONT PLANE
// ============================================================

static std::shared_ptr<geometry::PointCloud>
extractTemplateFrontPlane(
    const std::shared_ptr<geometry::PointCloud>& input)
{
    std::cout
        << "      Extracting template front plane...\n";


    auto remaining =
        std::make_shared<
            geometry::PointCloud>(
                *input);


    std::vector<
        std::shared_ptr<
            geometry::PointCloud>>
        planes;


    std::vector<
        Eigen::Vector4d>
        models;


    const double threshold =
        3.0;

    const int iterations =
        2000;

    const size_t min_points =
        40;


    for (int i = 0;
         i < 10;
         ++i)
    {
        if (remaining->points_.size() <
            min_points)
        {
            break;
        }


        auto result =
            remaining->SegmentPlane(
                threshold,
                3,
                iterations);


        Eigen::Vector4d model =
            std::get<0>(result);


        std::vector<size_t>
            inliers =
            std::get<1>(result);


        if (inliers.size() <
            min_points)
        {
            break;
        }


        auto plane =
            remaining->SelectByIndex(
                inliers);


        planes.push_back(
            plane);

        models.push_back(
            model);


        remaining =
            remaining->SelectByIndex(
                inliers,
                true);
    }


    if (planes.empty())
    {
        return nullptr;
    }


    // --------------------------------------------------------
    // Your template front face is generally a large thin
    // plane. Score planes by area, point count and thinness.
    // --------------------------------------------------------

    double best_score =
        -1e9;

    int best =
        -1;


    for (size_t i = 0;
         i < planes.size();
         ++i)
    {
        Eigen::Vector3d d =
            planes[i]->GetMaxBound()
            -
            planes[i]->GetMinBound();


        std::vector<double>
            dimensions =
        {
            std::abs(d.x()),
            std::abs(d.y()),
            std::abs(d.z())
        };


        std::sort(
            dimensions.begin(),
            dimensions.end());


        double smallest =
            dimensions[0];

        double middle =
            dimensions[1];

        double largest =
            dimensions[2];


        Eigen::Vector3d normal =
            normalizeNormal(
                models[i]);


        double area =
            largest *
            middle;


        double thinness =
            smallest /
            std::max(
                largest,
                1.0);


        double score =
            area
            +
            50.0 *
            static_cast<double>(
                planes[i]->points_.size());


        if (thinness <
            0.02)
        {
            score +=
                area * 0.30;
        }


        /*
          Your known pallet front plane has a strong X-normal.
          Give X-normal planes a preference.
        */

        double x_normal =
            std::abs(
                normal.x());


        if (x_normal >
            0.90)
        {
            score +=
                area * 1.0;
        }


        std::cout
            << "        Plane "
            << i
            << " : points="
            << planes[i]->points_.size()
            << " X="
            << std::abs(d.x())
            << " Y="
            << std::abs(d.y())
            << " Z="
            << std::abs(d.z())
            << " normalX="
            << x_normal
            << "\n";


        if (score >
            best_score)
        {
            best_score =
                score;

            best =
                static_cast<int>(i);
        }
    }


    if (best < 0)
    {
        return nullptr;
    }


    std::cout
        << "        Selected plane : "
        << best
        << "\n";


    return planes[best];
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


    Eigen::Matrix3d e =
        solver.eigenvectors();


    Eigen::Matrix3d axes;


    axes.col(0) =
        e.col(2);

    axes.col(1) =
        e.col(1);

    axes.col(2) =
        e.col(0);


    return axes;
}


// ============================================================
// PCA INITIAL TRANSFORM
// ============================================================

static Eigen::Matrix4d makePCATransform(
    const geometry::PointCloud& source,
    const geometry::PointCloud& target,
    const Eigen::Matrix3d& source_axes,
    const Eigen::Matrix3d& target_axes,
    const Eigen::Vector3i& permutation,
    const Eigen::Vector3i& signs)
{
    Eigen::Matrix3d mapping =
        Eigen::Matrix3d::Zero();


    for (int i = 0;
         i < 3;
         ++i)
    {
        mapping(
            i,
            permutation(i)) =
            static_cast<double>(
                signs(i));
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


    Eigen::Vector3d t =
        target_center -
        R *
        source_center;


    Eigen::Matrix4d T =
        Eigen::Matrix4d::Identity();


    T.block<3,3>(0,0) =
        R;


    T.block<3,1>(0,3) =
        t;


    return T;
}


// ============================================================
// RMSE
// ============================================================

static double computeRMSE(
    geometry::PointCloud& source,
    const geometry::PointCloud& target)
{
    std::vector<double>
        distances =
        source.ComputePointCloudDistance(
            target);


    if (distances.empty())
    {
        return std::numeric_limits<double>::max();
    }


    double sum =
        0.0;


    for (double d :
         distances)
    {
        sum +=
            d * d;
    }


    return std::sqrt(
        sum /
        static_cast<double>(
            distances.size()));
}


// ============================================================
// STEP 4
// ICP FOR ONE TEMPLATE
// ============================================================

static ICPResult
runICPForTemplate(
    const std::shared_ptr<geometry::PointCloud>& scene,
    const std::shared_ptr<geometry::PointCloud>& templ,
    const std::string& template_name)
{
    std::cout
        << "\n--------------------------------------------\n";

    std::cout
        << "ICP : "
        << template_name
        << "\n";


    const double voxel_size =
        5.0;

    const double max_correspondence =
        80.0;


    auto scene_down =
        scene->VoxelDownSample(
            voxel_size);


    auto template_down =
        templ->VoxelDownSample(
            voxel_size);


    if (scene_down->points_.size() <
            10
        ||
        template_down->points_.size() <
            10)
    {
        throw std::runtime_error(
            "Too few points after downsampling.");
    }


    Eigen::Matrix3d scene_axes =
        computePCA(
            *scene_down);


    Eigen::Matrix3d template_axes =
        computePCA(
            *template_down);


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


    double best_pca_rmse =
        std::numeric_limits<double>::max();


    Eigen::Matrix4d
        best_initial =
        Eigen::Matrix4d::Identity();


    // --------------------------------------------------------
    // PCA orientation search
    // --------------------------------------------------------

    for (const auto& permutation :
         permutations)
    {
        for (const auto& sign :
             signs)
        {
            Eigen::Matrix4d T =
                makePCATransform(
                    *template_down,
                    *scene_down,
                    template_axes,
                    scene_axes,
                    permutation,
                    sign);


            auto test =
                std::make_shared<
                    geometry::PointCloud>(
                        *template_down);


            test->Transform(T);


            double rmse =
                computeRMSE(
                    *test,
                    *scene_down);


            if (rmse <
                best_pca_rmse)
            {
                best_pca_rmse =
                    rmse;

                best_initial =
                    T;
            }
        }
    }


    std::cout
        << "PCA RMSE : "
        << best_pca_rmse
        << " mm\n";


    // --------------------------------------------------------
    // Apply PCA initial alignment
    // --------------------------------------------------------

    auto aligned =
        std::make_shared<
            geometry::PointCloud>(
                *template_down);


    aligned->Transform(
        best_initial);


    // --------------------------------------------------------
    // ICP
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


    auto result =
        pipelines::registration::
        RegistrationICP(
            *aligned,
            *scene_down,
            max_correspondence,
            Eigen::Matrix4d::Identity(),
            pipelines::registration::
                TransformationEstimationPointToPoint(),
            criteria);


    ICPResult output;


    output.template_name =
        template_name;


    output.fitness =
        result.fitness_;


    output.rmse =
        result.inlier_rmse_;


    output.transformation =
        result.transformation_ *
        best_initial;


    output.template_front =
        templ;


    output.aligned_template =
        std::make_shared<
            geometry::PointCloud>(
                *templ);


    output.aligned_template->Transform(
        output.transformation);


    std::cout
        << "Fitness : "
        << output.fitness
        << "\n";


    std::cout
        << "RMSE    : "
        << output.rmse
        << " mm\n";


    return output;
}


// ============================================================
// TEMPLATE DIRECTORY
// ============================================================

static std::vector<std::string>
findTemplates(
    const std::string& directory)
{
    std::vector<std::string>
        files;


    if (!fs::exists(directory))
    {
        return files;
    }


    for (const auto& entry :
         fs::directory_iterator(
             directory))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }


        std::string extension =
            entry.path()
                .extension()
                .string();


        std::transform(
            extension.begin(),
            extension.end(),
            extension.begin(),
            ::tolower);


        if (extension ==
                ".pcd"
            ||
            extension ==
                ".ply")
        {
            files.push_back(
                entry.path().string());
        }
    }


    std::sort(
        files.begin(),
        files.end());


    return files;
}


// ============================================================
// MAIN
// ============================================================

int main(
    int argc,
    char** argv)
{
    std::cout
        << "\n"
        << "============================================================\n"
        << "          MULTI-TEMPLATE PALLET DETECTION\n"
        << "============================================================\n";


    // --------------------------------------------------------
    // Arguments
    // --------------------------------------------------------

    if (argc < 3)
    {
        std::cerr
            << "\nUsage:\n\n"

            << "  "
            << argv[0]
            << " <roi.pcd> <template_directory>\n\n"

            << "Example:\n\n"

            << "  "
            << argv[0]
            << " ../roi.pcd ../templates\n\n";


        return 1;
    }


    std::string scene_file =
        argv[1];


    std::string template_directory =
        argv[2];


    // ========================================================
    // STEP 1
    // LOAD SCENE
    // ========================================================

    std::cout
        << "\n================================================\n"
        << "              LOADING SCENE\n"
        << "================================================\n";


    auto scene =
        std::make_shared<
            geometry::PointCloud>();


    if (!io::ReadPointCloud(
            scene_file,
            *scene))
    {
        std::cerr
            << "ERROR: Cannot load:\n"
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
        << "Scene points : "
        << scene->points_.size()
        << "\n";


    // --------------------------------------------------------
    // ROI is meters -> mm
    // --------------------------------------------------------

    metersToMillimeters(
        *scene);


    std::cout
        << "Scene converted m -> mm\n";


    // ========================================================
    // LOAD ALL TEMPLATES
    // ========================================================

    std::cout
        << "\n================================================\n"
        << "             LOADING TEMPLATES\n"
        << "================================================\n";


    std::vector<std::string>
        template_files =
        findTemplates(
            template_directory);


    if (template_files.empty())
    {
        std::cerr
            << "ERROR: No PCD/PLY templates found in:\n"
            << template_directory
            << "\n";

        return 1;
    }


    std::cout
        << "Templates found : "
        << template_files.size()
        << "\n";


    std::vector<TemplateInfo>
        templates;


    for (const auto& file :
         template_files)
    {
        std::cout
            << "\nLoading "
            << fs::path(file).filename().string()
            << "\n";


        auto cloud =
            std::make_shared<
                geometry::PointCloud>();


        if (!io::ReadPointCloud(
                file,
                *cloud))
        {
            std::cerr
                << "WARNING: Could not load "
                << file
                << "\n";

            continue;
        }


        removeInvalidPoints(
            cloud);


        if (cloud->IsEmpty())
        {
            continue;
        }


        TemplateInfo info;


        info.filename =
            file;


        info.original =
            cloud;


        info.dimensions =
            sortedDimensions(
                *cloud);


        std::cout
            << "Points : "
            << cloud->points_.size()
            << "\n";


        std::cout
            << "Dimensions : "
            << info.dimensions.transpose()
            << " mm\n";


        // ----------------------------------------------------
        // Extract front plane
        // ----------------------------------------------------

        info.front_plane =
            extractTemplateFrontPlane(
                cloud);


        if (!info.front_plane)
        {
            std::cerr
                << "WARNING: Front plane extraction failed.\n";

            continue;
        }


        templates.push_back(
            info);
    }


    if (templates.empty())
    {
        std::cerr
            << "ERROR: No usable templates.\n";

        return 1;
    }


    // ========================================================
    // STEP 2
    // DBSCAN
    // ========================================================

    auto pallet_cluster =
        runDBSCAN(
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
    // STEP 4
    // ICP AGAINST EVERY TEMPLATE
    // ========================================================

    std::cout
        << "\n================================================\n"
        << "          STEP 4 : MULTI TEMPLATE ICP\n"
        << "================================================\n";


    std::vector<ICPResult>
        results;


    for (const auto& templ :
         templates)
    {
        try
        {
            std::string name =
                fs::path(
                    templ.filename)
                    .filename()
                    .string();


            ICPResult result =
                runICPForTemplate(
                    fork_entry,
                    templ.front_plane,
                    name);


            results.push_back(
                result);
        }
        catch (const std::exception& e)
        {
            std::cerr
                << "ICP failed for "
                << templ.filename
                << " : "
                << e.what()
                << "\n";
        }
    }


    if (results.empty())
    {
        std::cerr
            << "ERROR: All template ICP operations failed.\n";

        return 1;
    }


    // ========================================================
    // STEP 5
    // COMPARE RESULTS
    // ========================================================

    std::cout
        << "\n============================================================\n"
        << "                    TEMPLATE COMPARISON\n"
        << "============================================================\n";


    std::cout
        << std::left
        << std::setw(25)
        << "Template"
        << std::setw(15)
        << "Fitness"
        << std::setw(15)
        << "RMSE(mm)"
        << "\n";


    std::cout
        << "------------------------------------------------------------\n";


    for (const auto& r :
         results)
    {
        std::cout
            << std::left
            << std::setw(25)
            << r.template_name

            << std::setw(15)
            << std::fixed
            << std::setprecision(5)
            << r.fitness

            << std::setw(15)
            << std::setprecision(3)
            << r.rmse

            << "\n";
    }


    // ========================================================
    // SELECT BEST TEMPLATE
    // ========================================================

    /*
       Primary criterion:
           highest ICP fitness

       Secondary criterion:
           lowest RMSE
    */

    auto best =
        std::max_element(
            results.begin(),
            results.end(),
            [](const ICPResult& a,
               const ICPResult& b)
            {
                if (std::abs(
                        a.fitness -
                        b.fitness) >
                    1e-6)
                {
                    return a.fitness <
                           b.fitness;
                }

                return a.rmse >
                       b.rmse;
            });


    // ========================================================
    // FINAL RESULT
    // ========================================================

    std::cout
        << "\n============================================================\n"
        << "                    BEST MATCH\n"
        << "============================================================\n";


    std::cout
        << "Template : "
        << best->template_name
        << "\n";


    std::cout
        << "Fitness  : "
        << best->fitness
        << "\n";


    std::cout
        << "RMSE     : "
        << best->rmse
        << " mm\n";


    // ========================================================
    // CENTER
    // ========================================================

    Eigen::Vector3d center =
        best->aligned_template
            ->GetCenter();


    // ========================================================
    // ANGLE
    // ========================================================

    Eigen::Matrix3d R =
        best->transformation
            .block<3,3>(0,0);


    double angle_rad =
        std::atan2(
            R(1,0),
            R(0,0));


    double angle_deg =
        angle_rad *
        180.0 /
        M_PI;


    std::cout
        << "\n============================================================\n"
        << "                  PALLET DETECTION\n"
        << "============================================================\n";


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


    // ========================================================
    // TRANSFORMATION
    // ========================================================

    std::cout
        << "\nTransformation matrix:\n";

    std::cout
        << best->transformation
        << "\n";


    // ========================================================
    // SAVE FINAL RESULT
    // ========================================================

    std::string output_file =
        "../aligned_best_template.pcd";


    if (!io::WritePointCloud(
            output_file,
            *best->aligned_template))
    {
        std::cerr
            << "WARNING: Could not save final cloud.\n";
    }
    else
    {
        std::cout
            << "\nFinal aligned template:\n"
            << output_file
            << "\n";
    }


    // ========================================================
    // VIEWER
    // ========================================================

    pallet_cluster->PaintUniformColor(
        Eigen::Vector3d(
            0.0,
            1.0,
            0.0));


    best->aligned_template
        ->PaintUniformColor(
            Eigen::Vector3d(
                1.0,
                0.0,
                0.0));


    std::cout
        << "\n============================================================\n"
        << "              OPENING FINAL VIEWER\n"
        << "============================================================\n";


    visualization::DrawGeometries(
        {
            pallet_cluster,
            best->aligned_template
        },
        "BEST PALLET TEMPLATE MATCH",
        1280,
        720);


    // ========================================================
    // END
    // ========================================================

    std::cout
        << "\n============================================================\n"
        << "                 PIPELINE COMPLETE\n"
        << "============================================================\n";


    return 0;
}
