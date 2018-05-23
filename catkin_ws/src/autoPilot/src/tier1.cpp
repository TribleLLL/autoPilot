#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  /*// Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;*/
  
  std::cout<<"test3\n";
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  pcl_conversions::toPCL(*input, *cloud);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.1f,0.1f,0.1f);
  sor.filter(cloud_filtered);

  sensor_msgs::PointCloud2 output;

  pcl_conversions::fromPCL(cloud_filtered, output);
  std::cout<<"test2\n";

/*****/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(output, *cloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 output2;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

  coefficients->values.resize(4);

  coefficients->values[0] = 0;

  coefficients->values[1] = 0;

  coefficients->values[2] = 1;

  coefficients->values[3] = 0;


  std::cout << "IN XYZ:" << cloud2->points[0].x << " " << cloud2->points[0].y << " " << cloud2->points[0].z;


// Create the filtering object

  pcl::ProjectInliers<pcl::PointXYZ> proj;//创建投影滤波对象

  proj.setModelType(pcl::SACMODEL_PLANE);//设置对象对应的投影模型

  proj.setInputCloud(cloud2);//设置输入点云

  proj.setModelCoefficients(coefficients);//设置模型对应的系数

  proj.filter(*cloud_projected);//执行投影滤波存储结果cloud_projected

  pcl::toROSMsg(*cloud_projected, output2);

  std::cout<<"test1\n";
/******/


  // Publish the data.
  pub.publish (output2);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  std::cout<<"test4\n";
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  ros::Subscriber sub = nh.subscribe ("point_cloud", 1, cloud_cb);
  
  // Spin
  ros::spin ();
}
