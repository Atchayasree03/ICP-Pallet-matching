#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
    std::string input_file = "../roi.pcd";
    std::string output_file = "../roi_mm.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    // ------------------------------------------------------------
    // Load ROI point cloud
    // ------------------------------------------------------------
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        std::cerr << "ERROR: Could not load " << input_file << std::endl;
        return -1;
    }

    std::cout << "Loaded ROI point cloud\n";
    std::cout << "Points: " << cloud->size() << std::endl;

    // ------------------------------------------------------------
    // Convert meters -> millimeters
    // ------------------------------------------------------------
    for (auto& point : cloud->points)
    {
        point.x *= 1000.0f;
        point.y *= 1000.0f;
        point.z *= 1000.0f;
    }

    // ------------------------------------------------------------
    // Save converted cloud
    // ------------------------------------------------------------
    if (pcl::io::savePCDFileBinary(output_file, *cloud) == -1)
    {
        std::cerr << "ERROR: Could not save " << output_file << std::endl;
        return -1;
    }

    std::cout << "\nConversion complete.\n";
    std::cout << "Output file: " << output_file << std::endl;

    return 0;
}
