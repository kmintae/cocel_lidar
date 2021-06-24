/**
 * Voxelization = Uniform Sampling (in Mechanism)
 *
 * Voxelization: Returns 'PointCloud'
 * Uniform Sampling: Returns 'Well-Formatted Point Indices' -> Used w/ Features Library
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f); /// Size of Voxel
	sor.filter (*cloud_filtered);
	
	/// cout # of Filtered Points
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	
	/// filtered .pcd File Save (Inliner)
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
	// pcl::io::savePCDFile<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered);
	
	return 0;
}