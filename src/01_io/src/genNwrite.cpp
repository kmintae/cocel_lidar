#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int 
main (int argc, char** argv)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	cloud->width = 5;
	cloud->height = 1;
	cloud->is_dense = false;
	(cloud->points).resize (cloud->width * cloud->height);
	
	for (size_t i = 0; i < (cloud->points).size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	
	pcl::io::savePCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud);
	
	std::cerr << "Saved " << (cloud->points).size() << " data points to test_pcd.pcd." << std::endl;
	
	return 0;
}