#include <open3d/Open3D.h>

#include <Eigen/Dense>

#include <iostream>
#include <iomanip>
#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace open3d;


// ============================================================
// PCA AXES
// ============================================================

Eigen::Matrix3d computePCA(
    const geometry::PointCloud &cloud)
{
    Eigen::Vector3d center = cloud.GetCenter();

    Eigen::Matrix3d covariance =
        Eigen::Matrix3d::Zero();

    for (const auto &p : cloud.points_)
    {
        Eigen::Vector3d q = p - center;
        covariance += q * q.transpose();
    }

    covariance /= static_cast<double>(
        std::max<size_t>(1, cloud.points_.size()));

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
        covariance);

    Eigen::Matrix3d eigenvectors =
        solver.eigenvectors();

    // Eigenvalues are in ascending order.
    // We want:
    //
    // column 0 = largest direction
    // column 1 = middle direction
    // column 2 = smallest direction

    Eigen::Matrix3d axes;

    axes.col(0) = eigenvectors.col(2);
    axes.col(1) = eigenvectors.col(1);
    axes.col(2) = eigenvectors.col(0);

    return axes;
}


// ============================================================
// PRINT DIMENSIONS
// ============================================================

void printDimensions(
    const geometry::PointCloud &cloud,
    const std::string &name)
{
    if (cloud.IsEmpty())
    {
        std::cout << name << " is empty.\n";
        return;
    }

    Eigen::Vector3d min_bound =
        cloud.GetMinBound();

    Eigen::Vector3d max_bound =
        cloud.GetMaxBound();

    Eigen::Vector3d size =
        max_bound - min_bound;

    Eigen::Vector3d center =
        cloud.GetCenter();

    std::cout << "\n";
    std::cout << "--------------------------------------------\n";
    std::cout << name << " DIMENSIONS\n";
    std::cout << "--------------------------------------------\n";

    std::cout << std::fixed
              << std::setprecision(3);

    std::cout << "X = "
              << size(0)
              << " mm\n";

    std::cout << "Y = "
              << size(1)
              << " mm\n";

    std::cout << "Z = "
              << size(2)
              << " mm\n";

    std::cout << "\nCenter:\n";

    std::cout << "X = "
              << center(0)
              << " mm\n";

    std::cout << "Y = "
              << center(1)
              << " mm\n";

    std::cout << "Z = "
              << center(2)
              << " mm\n";
}


// ============================================================
// NEAREST NEIGHBOUR RMSE
//
// IMPORTANT:
// Open3D 0.19 defines ComputePointCloudDistance()
// as a NON-CONST function.
//
// Therefore source is intentionally passed by reference.
// ============================================================

double computeNearestRMSE(
    geometry::PointCloud &source,
    const geometry::PointCloud &target)
{
    if (source.IsEmpty() ||
        target.IsEmpty())
    {
        return std::numeric_limits<double>::max();
    }

    std::vector<double> distances =
        source.ComputePointCloudDistance(target);

    if (distances.empty())
    {
        return std::numeric_limits<double>::max();
    }

    double sum_squared = 0.0;

    for (double d : distances)
    {
        sum_squared += d * d;
    }

    return std::sqrt(
        sum_squared /
        static_cast<double>(distances.size()));
}


// ============================================================
// CREATE PCA INITIAL TRANSFORMATION
// ============================================================

Eigen::Matrix4d createInitialTransform(
    const geometry::PointCloud &source,
    const geometry::PointCloud &target,
    const Eigen::Matrix3d &source_axes,
    const Eigen::Matrix3d &target_axes,
    const Eigen::Vector3i &permutation,
    const Eigen::Vector3i &signs)
{
    /*
        permutation:

        target axis 0 <- source axis permutation(0)
        target axis 1 <- source axis permutation(1)
        target axis 2 <- source axis permutation(2)
    */

    Eigen::Matrix3d mapping =
        Eigen::Matrix3d::Zero();

    for (int target_axis = 0;
         target_axis < 3;
         ++target_axis)
    {
        int source_axis =
            permutation(target_axis);

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

    /*
        ICP requires a proper rotation.
        A determinant of -1 means reflection.
    */

    if (R.determinant() < 0.0)
    {
        R.col(2) *= -1.0;
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

    T.block<3,3>(0,0) = R;

    T.block<3,1>(0,3) =
        translation;

    return T;
}


// ============================================================
// MAIN
// ============================================================

int main()
{
    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "        OPEN3D PALLET ICP MATCHING\n";
    std::cout << "============================================\n";


    // ========================================================
    // INPUT FILES
    // ========================================================

    const std::string scene_file =
        "../roi_fork_entry.pcd";

    const std::string template_file =
        "../templates/pallet_1.pcd";

    const std::string output_file =
        "../aligned_template.pcd";


    std::cout << "\n";
    std::cout << "Scene    : "
              << scene_file
              << "\n";

    std::cout << "Template : "
              << template_file
              << "\n";


    // ========================================================
    // LOAD SCENE
    // ========================================================

    auto scene =
        std::make_shared<geometry::PointCloud>();

    if (!io::ReadPointCloud(
            scene_file,
            *scene))
    {
        std::cerr
            << "\nERROR: Cannot load scene:\n"
            << scene_file
            << "\n";

        return 1;
    }


    // ========================================================
    // LOAD TEMPLATE
    // ========================================================

    auto templ =
        std::make_shared<geometry::PointCloud>();

    if (!io::ReadPointCloud(
            template_file,
            *templ))
    {
        std::cerr
            << "\nERROR: Cannot load template:\n"
            << template_file
            << "\n";

        return 1;
    }


    // ========================================================
    // REMOVE INVALID POINTS
    // ========================================================

    scene->RemoveNonFinitePoints();

    templ->RemoveNonFinitePoints();


    // ========================================================
    // POINT COUNTS
    // ========================================================

    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "              INPUT DATA\n";
    std::cout << "============================================\n";

    std::cout << "Scene points    : "
              << scene->points_.size()
              << "\n";

    std::cout << "Template points : "
              << templ->points_.size()
              << "\n";


    // ========================================================
    // DIMENSIONS
    // ========================================================

    printDimensions(
        *scene,
        "SCENE");

    printDimensions(
        *templ,
        "TEMPLATE");


    if (scene->points_.size() < 10 ||
        templ->points_.size() < 10)
    {
        std::cerr
            << "\nERROR: Not enough points.\n";

        return 1;
    }


    // ========================================================
    // DOWNSAMPLE
    // ========================================================

    /*
        Your point clouds are in millimetres.

        5 mm voxel gives a reasonably dense
        representation while making ICP faster.
    */

    const double voxel_size = 5.0;

    auto scene_down =
        scene->VoxelDownSample(
            voxel_size);

    auto template_down =
        templ->VoxelDownSample(
            voxel_size);


    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "           AFTER DOWNSAMPLING\n";
    std::cout << "============================================\n";

    std::cout << "Scene points    : "
              << scene_down->points_.size()
              << "\n";

    std::cout << "Template points : "
              << template_down->points_.size()
              << "\n";


    if (scene_down->points_.size() < 10 ||
        template_down->points_.size() < 10)
    {
        std::cerr
            << "\nERROR: Too few points after downsampling.\n";

        return 1;
    }


    // ========================================================
    // PCA
    // ========================================================

    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "          PCA INITIAL ALIGNMENT\n";
    std::cout << "============================================\n";

    Eigen::Matrix3d scene_axes =
        computePCA(
            *scene_down);

    Eigen::Matrix3d template_axes =
        computePCA(
            *template_down);


    // ========================================================
    // ALL PCA PERMUTATIONS
    // ========================================================

    std::vector<Eigen::Vector3i> permutations =
    {
        Eigen::Vector3i(0, 1, 2),
        Eigen::Vector3i(0, 2, 1),
        Eigen::Vector3i(1, 0, 2),
        Eigen::Vector3i(1, 2, 0),
        Eigen::Vector3i(2, 0, 1),
        Eigen::Vector3i(2, 1, 0)
    };


    // PCA axis signs are ambiguous.
    //
    // +X and -X represent the same PCA axis.
    // Therefore try all sign combinations.

    std::vector<Eigen::Vector3i> signs =
    {
        Eigen::Vector3i( 1,  1,  1),
        Eigen::Vector3i( 1,  1, -1),
        Eigen::Vector3i( 1, -1,  1),
        Eigen::Vector3i( 1, -1, -1),
        Eigen::Vector3i(-1,  1,  1),
        Eigen::Vector3i(-1,  1, -1),
        Eigen::Vector3i(-1, -1,  1),
        Eigen::Vector3i(-1, -1, -1)
    };


    // ========================================================
    // FIND BEST PCA ORIENTATION
    // ========================================================

    double best_rmse =
        std::numeric_limits<double>::max();

    Eigen::Matrix4d best_transform =
        Eigen::Matrix4d::Identity();


    int candidate = 0;


    for (const auto &perm :
         permutations)
    {
        for (const auto &sgn :
             signs)
        {
            candidate++;

            Eigen::Matrix4d T =
                createInitialTransform(
                    *template_down,
                    *scene_down,
                    template_axes,
                    scene_axes,
                    perm,
                    sgn);


            auto test_cloud =
                std::make_shared<
                    geometry::PointCloud>(
                    *template_down);


            test_cloud->Transform(T);


            double rmse =
                computeNearestRMSE(
                    *test_cloud,
                    *scene_down);


            if (rmse < best_rmse)
            {
                best_rmse = rmse;

                best_transform = T;
            }
        }
    }


    // ========================================================
    // BEST INITIAL ALIGNMENT
    // ========================================================

    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "       BEST PCA INITIAL ALIGNMENT\n";
    std::cout << "============================================\n";

    std::cout << std::fixed
              << std::setprecision(6);

    std::cout << "Best initial RMSE : "
              << best_rmse
              << " mm\n";

    std::cout << "\nInitial transformation:\n";

    std::cout
        << best_transform
        << "\n";


    // ========================================================
    // APPLY PCA TRANSFORM
    // ========================================================

    auto aligned_template =
        std::make_shared<
            geometry::PointCloud>(
                *template_down);

    aligned_template->Transform(
        best_transform);


    // ========================================================
    // ICP PARAMETERS
    // ========================================================

    const double max_correspondence_distance =
        80.0;


    pipelines::registration::
        ICPConvergenceCriteria criteria;


    criteria.relative_fitness_ =
        1e-6;

    criteria.relative_rmse_ =
        1e-6;

    criteria.max_iteration_ =
        100;


    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "             ICP PARAMETERS\n";
    std::cout << "============================================\n";

    std::cout << "Voxel size                   : "
              << voxel_size
              << " mm\n";

    std::cout << "Max correspondence distance : "
              << max_correspondence_distance
              << " mm\n";

    std::cout << "Max iterations               : "
              << criteria.max_iteration_
              << "\n";


    // ========================================================
    // RUN ICP
    // ========================================================

    std::cout << "\n";
    std::cout << "Running ICP...\n";


    auto result =
        pipelines::registration::
            RegistrationICP(
                *aligned_template,
                *scene_down,
                max_correspondence_distance,
                Eigen::Matrix4d::Identity(),
                pipelines::registration::
                    TransformationEstimationPointToPoint(),
                criteria);


    // ========================================================
    // FINAL TRANSFORMATION
    //
    // IMPORTANT:
    //
    // Open3D 0.19 uses:
    //
    // result.transformation_
    //
    // NOT:
    //
    // result.transformation
    // ========================================================

    Eigen::Matrix4d final_transform =
        result.transformation_ *
        best_transform;


    // ========================================================
    // TRANSFORM ORIGINAL TEMPLATE
    // ========================================================

    auto final_template =
        std::make_shared<
            geometry::PointCloud>(
                *templ);

    final_template->Transform(
        final_transform);


    // ========================================================
    // SAVE RESULT
    // ========================================================

    if (!io::WritePointCloud(
            output_file,
            *final_template))
    {
        std::cerr
            << "\nERROR: Cannot save:\n"
            << output_file
            << "\n";

        return 1;
    }


    // ========================================================
    // ICP RESULT
    // ========================================================

    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "                ICP RESULT\n";
    std::cout << "============================================\n";

    std::cout << std::fixed
              << std::setprecision(6);

    std::cout << "Fitness : "
              << result.fitness_
              << "\n";

    std::cout << "RMSE    : "
              << result.inlier_rmse_
              << " mm\n";


    std::cout << "\nFinal transformation:\n";

    std::cout
        << final_transform
        << "\n";


    // ========================================================
    // FINAL FULL-RESOLUTION RMSE
    // ========================================================

    std::vector<double> final_distances =
        final_template->ComputePointCloudDistance(
            *scene);


    if (!final_distances.empty())
    {
        double sum_squared = 0.0;

        for (double d :
             final_distances)
        {
            sum_squared += d * d;
        }

        double final_rmse =
            std::sqrt(
                sum_squared /
                static_cast<double>(
                    final_distances.size()));

        std::cout << "\n";
        std::cout
            << "Full-resolution nearest-neighbour RMSE : "
            << final_rmse
            << " mm\n";
    }


    // ========================================================
    // VIEWER
    // ========================================================

    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "              OPEN3D VIEWER\n";
    std::cout << "============================================\n";

    /*
        GREEN = scene / fork_entry.pcd

        RED = transformed template / template_front_plane.pcd
    */

    scene->PaintUniformColor(
        Eigen::Vector3d(
            0.0,
            1.0,
            0.0));

    final_template->PaintUniformColor(
        Eigen::Vector3d(
            1.0,
            0.0,
            0.0));


    visualization::DrawGeometries(
        {
            scene,
            final_template
        },
        "ICP Pallet Matching",
        1280,
        720);


    // ========================================================
    // FINAL MESSAGE
    // ========================================================

    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "              FINAL RESULT\n";
    std::cout << "============================================\n";

    std::cout << "Aligned template saved:\n";
    std::cout << output_file << "\n";

    std::cout << "\n";
    std::cout << "GREEN = roi_fork_entry.pcd\n";
    std::cout << "RED   = transformed template_front_plane.pcd\n";

    std::cout << "============================================\n";


    return 0;
}