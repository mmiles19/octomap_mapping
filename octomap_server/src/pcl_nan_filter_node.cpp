#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
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
  // PCLPointCloud cloud_in;
  // pcl::fromROSMsg(*msg, cloud_in);
  // PCLPointCloud cloud_out;
  // for (size_t i = 0; i < cloud_in->points.size (); ++i)
  // {
  //   if (pcl::isFinite(cloud_in->points[i].x) && pcl::isFinite(cloud_in->points[i].y) && pcl::isFinite(cloud_in->points[i].z)   )
  //     cloud_out.push_back(cloud_in->points[i]);
  // }

  // // Container for original & filtered data
  // pcl::PointCloud<pcl::PointXYZ> cloud;
  // pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // // Convert to PCL data type
  // pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  // // Perform the actual filtering
  // for (size_t i = 0; i < cloud->points.size (); ++i)
  // {}
  //   if (pcl_isfinite(cloud->points[i].x) && pcl_isfinite(cloud->points[i].y) && pcl_isfinite(cloud->points[i].z))
  //     cloud_filtered.push_back(cloud->points[i]);
  // }
  // // Convert to ROS data type
  // pcl_conversions::fromPCL(cloud_filtered, output);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr p_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // for (int i = 0; i < (*cloud).size(); i++)
  // {
  //   pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
  //   if ( !pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z) )
  //   {
  //     inliers->indices.push_back(i);
  //   }
  // }
  // extract.setInputCloud(cloud);
  // extract.setIndices(inliers);
  // extract.setNegative(true);
  // extract.filter(*cloud_filtered);

  const pcl::PointCloud<pcl::PointXYZ> const_cloud = *cloud;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(const_cloud, *cloud_filtered, indices);
  cloud = cloud_filtered;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.8);
  outrem.setMinNeighborsInRadius (2);
  // apply filter
  outrem.filter (*cloud_filtered);

  pcl::toROSMsg(*cloud_filtered, output);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_nan_filter_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points",100,&callback);
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("filter_output", 100);

  while (n.ok())
  {
    pub.publish(output);
    ros::spinOnce();
    ros::Rate(100).sleep();
  }

  return 0;
} // end main()
