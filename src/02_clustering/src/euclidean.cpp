/**
 * Euclidean Cluster Extraction
 * 
 * Grouping points w/ distance < CONSTANT
 * 
 * Improvements: Adaptive Clustering
 */
 
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("RANSAC_plane_true.pcd", *cloud);
	
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
	
	// Generation of KdTree Object: Searching K Nearest Points
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud);
	
	std::vector<pcl::PointIndices> cluster_indices; // Clustering Result
	
	// Generation of Clustering Object
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setInputCloud (cloud);
	ec.setClusterTolerance (0.02); // Distance Threshold 0.02
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	
	ec.setSearchMethod (tree); // KdTree Search
	ec.extract (cluster_indices);
	
	
	// For Every Clusters...
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
		it != cluster_indices.end(); ++it) {
		
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); 
				pit != it->indices.end(); ++pit)
					cloud_cluster->points.push_back (cloud->points[*pit]);
			
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			
			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
			
			std::stringstream ss;
			ss << "cloud_cluster_" << j << ".pcd";
			pcl::PCDWriter writer;
			writer.write<pcl::PointXYZRGB> (ss.str(), *cloud_cluster, false);
			j++;
	}
	return 0;
}