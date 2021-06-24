/**
 * RANSAC: Random Sample Consensus
 *
 * Sample Consensus determines inliers & outliers w/ specific models.
 * RANsac randomly chooses m points, designs specific model, 
 * and then counts the # of points whose residuals < T.
 *
 * After repeating N times, 
 * pcl adopts the final model w/ maximum inlier points.
 *
 * Parameter N, T
 *
 * Limitations:
 * (-) Different Results with same data case
 * (-) High computational overhead
 * (-) Error Overfitting Problem?
 */

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
											cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>),
											inlierPoints (new pcl::PointCloud<pcl::PointXYZRGB>),
											inlierPoints_neg (new pcl::PointCloud<pcl::PointXYZRGB>);
											
	
	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop_passthrough.pcd", *cloud);
	
	std::cout << "Loaded :" << cloud->width * cloud->height << std::endl;
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
	
	/// Generation of RANSAC Object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients(true); // Coeff Refinement (Optional)
	seg.setInputCloud (cloud);
	seg.setModelType (pcl::SACMODEL_PLANE); // Decision Model: Plane
	seg.setMethodType (pcl::SAC_RANSAC); // RANSAC
	seg.setMaxIterations (1000); // N
	seg.setDistanceThreshold (0.01);
	
	seg.segment (*inliers, *coefficients);
	
	/// ax + by + cz + d = 0 (a, b, c, d)
	std::cerr << "Model Coeff: " << coefficients->values[0] << " "
									<< coefficients->values[1] << " "
									<< coefficients->values[2] << " "
									<< coefficients->values[3] << std::endl;
									
	pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, *inliers, *inlierPoints);
	pcl::io::savePCDFile<pcl::PointXYZRGB>("RANSAC_test_result.pcd", *inlierPoints);
	
	// Floor Elimination
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative(true);
	extract.filter (*inlierPoints_neg);
	pcl::io::savePCDFile<pcl::PointXYZRGB>("RANSAC_test_result_neg.pcd", *inlierPoints_neg);
	
	return 0;
}
