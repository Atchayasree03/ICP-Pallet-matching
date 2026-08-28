#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>
#include <iomanip>

#include <open3d/Open3D.h>

using namespace std;
using namespace open3d;

struct ClusterInfo
{
    int label;
    size_t points;
};

int main(int argc, char **argv)
{
    // =========================================================
    // PARAMETERS
    // =========================================================

    string input_file = "../roi_mm.pcd";
    string output_dir = "../clusters";

    double eps = 50.0;       // mm
    int min_points = 10;

    // Number of clusters we want to save
    int number_to_save = 7;

    // Allow command-line values:
    //
    // ./dbscan_7_clusters 50 10
    //
    if (argc >= 2)
        eps = stod(argv[1]);

    if (argc >= 3)
        min_points = stoi(argv[2]);


    // =========================================================
    // HEADER
    // =========================================================

    cout << "\n";
    cout << "============================================\n";
    cout << "          DBSCAN - 7 CLUSTERS\n";
    cout << "============================================\n";

    cout << "Input file : " << input_file << "\n";
    cout << "Output dir : " << output_dir << "\n";
    cout << "EPS        : " << eps << " mm\n";
    cout << "Min points : " << min_points << "\n";
    cout << "Clusters to save : " << number_to_save << "\n";

    cout << "============================================\n";


    // =========================================================
    // LOAD POINT CLOUD
    // =========================================================

    auto cloud = make_shared<geometry::PointCloud>();

    if (!io::ReadPointCloud(input_file, *cloud))
    {
        cerr << "\nERROR: Could not load point cloud:\n"
             << input_file << "\n";

        return -1;
    }

    cout << "\nLoaded point cloud.\n";
    cout << "Number of points : "
         << cloud->points_.size()
         << "\n";


    // =========================================================
    // CHECK EMPTY
    // =========================================================

    if (cloud->points_.empty())
    {
        cerr << "ERROR: Point cloud is empty.\n";
        return -1;
    }


    // =========================================================
    // RUN DBSCAN
    // =========================================================

    cout << "\nRunning DBSCAN...\n";

    vector<int> labels =
        cloud->ClusterDBSCAN(
            eps,
            min_points,
            true
        );


    // =========================================================
    // FIND NUMBER OF CLUSTERS
    // =========================================================

    int max_label = -1;

    for (int label : labels)
    {
        if (label > max_label)
            max_label = label;
    }

    int total_clusters = max_label + 1;


    // =========================================================
    // COUNT POINTS IN EACH CLUSTER
    // =========================================================

    vector<ClusterInfo> clusters;

    for (int i = 0; i < total_clusters; i++)
    {
        size_t count = 0;

        for (int label : labels)
        {
            if (label == i)
                count++;
        }

        ClusterInfo info;

        info.label = i;
        info.points = count;

        clusters.push_back(info);
    }


    // =========================================================
    // SORT CLUSTERS BY SIZE
    // LARGEST FIRST
    // =========================================================

    sort(
        clusters.begin(),
        clusters.end(),
        [](const ClusterInfo &a,
           const ClusterInfo &b)
        {
            return a.points > b.points;
        }
    );


    // =========================================================
    // CREATE OUTPUT DIRECTORY
    // =========================================================

    filesystem::create_directories(output_dir);


    // =========================================================
    // PRINT DBSCAN SUMMARY
    // =========================================================

    size_t noise_points = 0;

    for (int label : labels)
    {
        if (label == -1)
            noise_points++;
    }

    cout << "\n============================================\n";
    cout << "             DBSCAN RESULTS\n";
    cout << "============================================\n";

    cout << "Total clusters found : "
         << total_clusters
         << "\n";

    cout << "Noise points         : "
         << noise_points
         << "\n";


    // =========================================================
    // CHECK NUMBER OF CLUSTERS
    // =========================================================

    if (total_clusters < number_to_save)
    {
        cout << "\nWARNING:\n";
        cout << "DBSCAN found only "
             << total_clusters
             << " clusters.\n";

        cout << "Cannot create "
             << number_to_save
             << " real clusters.\n";

        cout << "Reduce EPS or MIN_POINTS if you need more clusters.\n";
    }


    // =========================================================
    // SAVE LARGEST 7 CLUSTERS
    // =========================================================

    int save_count =
        min(
            number_to_save,
            total_clusters
        );


    cout << "\n============================================\n";
    cout << "          SAVING CLUSTERS\n";
    cout << "============================================\n";


    for (int rank = 0; rank < save_count; rank++)
    {
        int original_label =
            clusters[rank].label;


        // -----------------------------------------------------
        // Collect indices belonging to this cluster
        // -----------------------------------------------------

        vector<size_t> indices;

        for (size_t i = 0;
             i < labels.size();
             i++)
        {
            if (labels[i] == original_label)
            {
                indices.push_back(i);
            }
        }


        // -----------------------------------------------------
        // Extract cluster
        // -----------------------------------------------------

        auto cluster =
            cloud->SelectByIndex(indices);


        // -----------------------------------------------------
        // Calculate bounding box
        // -----------------------------------------------------

        auto bbox =
            cluster->GetAxisAlignedBoundingBox();

        Eigen::Vector3d min_bound =
            bbox.GetMinBound();

        Eigen::Vector3d max_bound =
            bbox.GetMaxBound();

        Eigen::Vector3d dimensions =
            max_bound - min_bound;


        // -----------------------------------------------------
        // Centroid
        // -----------------------------------------------------

        Eigen::Vector3d center =
            cluster->GetCenter();


        // -----------------------------------------------------
        // Output filename
        // -----------------------------------------------------

        string filename =
            output_dir +
            "/cluster_" +
            to_string(rank) +
            ".pcd";


        // -----------------------------------------------------
        // Save
        // -----------------------------------------------------

        if (!io::WritePointCloud(
                filename,
                *cluster))
        {
            cerr << "ERROR: Could not save "
                 << filename
                 << "\n";

            continue;
        }


        // -----------------------------------------------------
        // Print information
        // -----------------------------------------------------

        cout << "\n--------------------------------------------\n";

        cout << "Cluster " << rank << "\n";

        cout << "Original DBSCAN label : "
             << original_label
             << "\n";

        cout << "Points : "
             << cluster->points_.size()
             << "\n";

        cout << fixed
             << setprecision(3);

        cout << "Size X : "
             << dimensions(0)
             << " mm\n";

        cout << "Size Y : "
             << dimensions(1)
             << " mm\n";

        cout << "Size Z : "
             << dimensions(2)
             << " mm\n";

        cout << "Center X : "
             << center(0)
             << " mm\n";

        cout << "Center Y : "
             << center(1)
             << " mm\n";

        cout << "Center Z : "
             << center(2)
             << " mm\n";

        cout << "Saved : "
             << filename
             << "\n";
    }


    // =========================================================
    // FINAL
    // =========================================================

    cout << "\n============================================\n";
    cout << "              COMPLETE\n";
    cout << "============================================\n";

    cout << "Saved "
         << save_count
         << " cluster files.\n";

    cout << "\nOutput directory:\n"
         << output_dir
         << "\n";

    return 0;
}
