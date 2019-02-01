#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/point_types.h>
// #include <pcl/pcl_base.h>
// #include <pcl/common/io.h>
// #include <pcl/conversions.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/filters/covariance_sampling.h>
// #include <pcl/filters/normal_space.h>
// #include <pcl/filters/random_sample.h>
// #include <pcl/common/transforms.h>
// #include <pcl/common/eigen.h>
// #include <pcl/filters/boost.h>
// #include <cfloat>
// #include <pcl/PointIndices.h>

sensor_msgs::PointCloud2 output;

void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2 cloud_filtered;

  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  // pcl::fromROSMsg(*cloud_msg, cloud);
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_filt;
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> ror_filt;
  ror_filt.setInputCloud(cloudPtr);
  ror_filt.setRadiusSearch(0.3);
  ror_filt.setMinNeighborsInRadius (5);
  ror_filt.filter (cloud_filtered);

  // pcl::PCLPointCloud2ConstPtr cloudPtr2(&cloud_filtered);
  //
  // pcl::VoxelGrid<pcl::PCLPointCloud2> vg_filt;
  // vg_filt.setInputCloud (cloudPtr2);
  // vg_filt.setLeafSize (0.1f, 0.1f, 0.1f);
  // vg_filt.filter (cloud_filtered);

  pcl_conversions::fromPCL(cloud_filtered, output);
  // pcl::toROSMsg(*cloud, output);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_filter_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("input",1,&callback);
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("output", 1);

  while (n.ok())
  {
    pub.publish(output);
    ros::spinOnce();
    ros::Rate(100).sleep();
  }

  return 0;
} // end main()
