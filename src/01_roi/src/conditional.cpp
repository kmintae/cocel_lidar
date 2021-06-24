/**
 * Conditional Outlier Removal
 *
 * Cropping w/ GT, GE, LT, LE, EQ of ROI (x, y, z)
 *
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop.pcd", *cloud);
	
	std::cout << "Loaded :" << cloud->width * cloud -> height << std::endl;
	
	/// Generating Conditions
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
	
	range_cond -> addComparison (
		pcl::FieldComparison <pcl::PointXYZRGB>::ConstPtr (
			new pcl::FieldComparison <pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.0)));
	
	range_cond -> addComparison (
		pcl::FieldComparison <pcl::PointXYZRGB>::ConstPtr (
			new pcl::FieldComparison <pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, 0.8)));
	
	/// Generationg Objects
	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
	condrem.setInputCloud(cloud);
	condrem.setCondition (range_cond);
	condrem.setKeepOrganized(true);
	condrem.filter (*cloud_filtered);
	
	std::cout << "Filtered :" << cloud_filtered->width * cloud_filtered->height << std::endl;
	
	// Save
	pcl::io::savePCDFile<pcl::PointXYZRGB>("tabletop_conditional.pcd", *cloud_filtered);

}
