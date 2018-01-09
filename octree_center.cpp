#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
// 获取点云octree中心
using namespace std;
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  //pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (1);  //都行
    pcl::octree::OctreePointCloud<pcl::PointXYZRGBA> octree(1);         //都行 1为分辨率
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("1.pcd", *cloud) == -1) //* load the file
    {
        std::cout << "Error loading cloud 1.pcd." << std::endl;
        return -1;
    }
  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();
    std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA> > voxelCenters;
    octree.getOccupiedVoxelCenters(voxelCenters);
    auto a = voxelCenters.size();
    double d[a][3];
    for (int i = 0; i < a; i++) {

        d[i][0] = voxelCenters[i].x;
        d[i][1] = voxelCenters[i].y;
        d[i][2] = voxelCenters[i].z;
        cout<<d[i][0]<<' '<<d[i][1]<<' '<<d[i][2]<<endl;
    }
}
