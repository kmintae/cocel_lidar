#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
	
	/// Generating Object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK(50); /// Considering 50 Nearest Points
	sor.setStddevMulThresh (1.0); /// Stddev Threshold < 1.0
	sor.filter (*cloud_filtered);
	
	/// cout # of Filtered Points
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	
	/// filtered .pcd File Save (Inliner)
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
	
	/// ==============
	/// filtered .pcd File Save (Outliner)
	sor.setNegative (true);
	sor.filter (*cloud_filtered);
	writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
	
	return 0;
}