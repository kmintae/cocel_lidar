#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>

int
main (int argc, char** argv)
{
	/// Definition
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	
	/// .pcd File Read
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
	
	/// cout # of Points
	std::cerr << "Loaded :" << cloud->width * cloud->height << std::endl;
	
	/// Generating Object
	pcl::UniformSampling<pcl::PointXYZ> filter;
	filter.setInputCloud (cloud);
	filter.setRadiusSearch (0.01F); /// Search Range
	filter.filter (*cloud_filtered);
	
	/// cout # of Filtered Points
	std::cerr << "Filtered : " << cloud_filtered->width * cloud_filtered->height << std::endl;
	
	/// filtered .pcd File Save (Inliner)
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("uniform_sampling.pcd", *cloud_filtered, false);
	// pcl::io::savePCDFile<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered);
	
	return 0;
}