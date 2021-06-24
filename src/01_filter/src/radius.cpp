#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

int
main (int argc, char** argv)
{
	/// Definition
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	
	/// .pcd File Read
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
	// pcl::io::loadPCDFile<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
	
	/// cout # of Points
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
	
	/// Generating Object
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud (cloud);
	outrem.setRadiusSearch (0.01); /// Search Range: Radius 0.01
	outrem.setMinNeighborsInRadius (10); /// > 10 Points in 0.01?
	outrem.filter (*cloud_filtered);
	
	/// cout # of Filtered Points
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	
	/// filtered .pcd File Save (Inliner)
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("Radius_Outlier_Removal.pcd", *cloud_filtered, false);
	
	/// ==============
	/// filtered .pcd File Save (Outliner)
	outrem.setNegative (true);
	outrem.filter (*cloud_filtered);
	writer.write<pcl::PointXYZ> ("Radius_Outlier_Removal_Neg.pcd", *cloud_filtered, false);
	
	return 0;
}