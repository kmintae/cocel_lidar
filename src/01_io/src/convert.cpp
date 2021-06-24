#include <pcl/conversions.h>
pcl::PCLPointCloud2 point_cloud2;
pcl::PointCloud<pcl::PointXYZ> point_cloud;
pcl::fromPCLPointCloud2 (point_cloud2, point_cloud);
pcl::toPCLPointCloud2(point_cloud, point_cloud2);
