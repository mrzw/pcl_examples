// 获取TUM数据集里的rgb和depth，生成点云并合成转化到世界坐标系下
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>
using namespace std;
// 点云拼接
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

const double camera_factor = 5000;
const double camera_cx = 320.1;
const double camera_cy = 247.6;
const double camera_fx = 535.4;
const double camera_fy = 539.2;

const string project_dir = "/home/zw/vimproject/pcl_examples";
const string data_dir = project_dir+"/data/pictures/freiburg3/";
const string ground_dir = project_dir+"/data/pictures/associate_ground1.txt";
const string save_dir = project_dir+"/output/cloud_pcd/";

vector<string> rgb_file;
vector<string> depth_file;
vector<vector<double> > trajectory;
vector<Eigen::Isometry3d> poses;

void obtain_path()  // 获取rgb和depth路径，和对应的trajectory
{
    ifstream myfile(ground_dir);
    assert(myfile.is_open());
    string temp;
    vector<string> vec;
    vector<string> field;
    int a = 0;
    while(getline(myfile, temp))
    {
        if(a % 1 == 0)
        {
            vec.push_back(temp);
            a = 0;
        }
        a++;
    }
    myfile.close();
    vector<double> temp1;
    for(vector<string>::size_type i=0; i<vec.size(); i++)
    {
        boost::split(field, vec[i], boost::is_any_of(" "));
        rgb_file.push_back(field[1]);
        depth_file.push_back(field[3]);
        for(vector<string>::size_type j=0; j<7; j++)
            temp1.push_back(stod(field[j+5]));
        trajectory.push_back(temp1);
        temp1.clear();
//        for(vector<string>::size_type j=0; j<7; j++)
//            trajectory[i][j] = stod(field[j+5]);  // stod: string转double
    }
}
void siyuanTotran()  // 根据四元数和平移向量获取变换矩阵
{
    for(vector<vector<double> >::size_type i = 0; i < trajectory.size(); i++)
    {
        Eigen::Quaterniond q(trajectory[i][6], trajectory[i][3], trajectory[i][4], trajectory[i][5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(trajectory[i][0], trajectory[i][1], trajectory[i][2]));
        poses.push_back(T);
    }
}
/* 二维vector迭代
vector<vector<int> > v = {{1,2}, {3,4}};
for (const auto& inner : v) {
    for (const auto& item : inner) {
        cout << item << " ";
    }
}

vector< vector<int> >::iterator row;
vector<int>::iterator col;
for (row = v.begin(); row != v.end(); row++) {
    for (col = row->begin(); col != row->end(); col++) {
        // do stuff ...
    }
}
*/
int main(int argc, char** argv)
{
    obtain_path();
    siyuanTotran();
    PointCloud::Ptr cloud (new PointCloud);
    cout << "转换图像数：" << rgb_file.size() << endl;
    for(vector<string>::size_type i=0; i<rgb_file.size(); i++)
    {
        cout << "转换图像中： " << i+1 << endl;
        cv::Mat rgb = cv::imread(data_dir+rgb_file[i]);
        cv::Mat depth = cv::imread(data_dir+depth_file[i], -1);
        for(int m=0; m<depth.rows; m++)
            for(int n=0; n<depth.cols; n++)
            {
                ushort d = depth.ptr<ushort>(m)[n];
                if(d==0)
                    continue;
                Eigen::Vector3d point;
                point[2] = double(d)/camera_factor;
                point[0] = (n-camera_cx)*point[2]/camera_fx;
                point[1] = (m-camera_cy)*point[2]/camera_fy;
                Eigen::Vector3d pointWorld = poses[i]*point;
                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = rgb.ptr<uchar>(m)[n*3];
                p.g = rgb.ptr<uchar>(m)[n*3+1];
                p.r = rgb.ptr<uchar>(m)[n*3+2];
                cloud->points.push_back(p);
            }
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    pcl::VoxelGrid<PointT> filter;
    PointCloud::Ptr filterdcloud(new PointCloud);
    filter.setInputCloud(cloud);
    filter.setLeafSize(0.01,0.01,0.01);
    filter.filter(*filterdcloud);
    pcl::io::savePCDFile(save_dir+"all.pcd", *filterdcloud, true);
    return 0;
}
