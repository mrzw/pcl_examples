#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
// http://www.pointclouds.org/documentation/tutorials/octree_change.php#octree-change-detection
// 两幅点云对比检测
int main (int argc, char** argv)
{
    // Octree resolution - side length of octree voxels
    float resolution = 1.0;

    // Instantiate octree-based point cloud change detection class
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA> octree (resolution);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGBA> );

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("1.pcd", *cloudA) == -1) //* load the file
    {
        std::cout << "Error loading cloud 1.pcd." << std::endl;
        return -1;
    }

    // Add points from cloudA to octree
    octree.setInputCloud (cloudA);
    octree.addPointsFromInputCloud ();

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers ();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGBA> );

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("2.pcd", *cloudB) == -1) //* load the file
    {
        std::cout << "Error loading cloud 2.pcd." << std::endl;
        return -1;
    }

    // Add points from cloudB to octree
    octree.setInputCloud (cloudB);
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;

    // Get vector of point indices from octree voxels which did not exist in previous buffer
    octree.getPointIndicesFromNewVoxels (newPointIdxVector);

    // Output points
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    if(newPointIdxVector.size ()==0)
    {
        std::cout << "No difference between the two pcd" << std::endl;
    }
    for (size_t i = 0; i < newPointIdxVector.size (); ++i)
        std::cout << i << "# Index:" << newPointIdxVector[i]
                  << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
                  << cloudB->points[newPointIdxVector[i]].y << " "
                  << cloudB->points[newPointIdxVector[i]].z << std::endl;
    return 0;
}
