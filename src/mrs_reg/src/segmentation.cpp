/*
 * segmentation.cpp
 *
 *  Created on: Jul 4, 2016
 *      Author: jaakko
 */

#include "segmentation.h"

using namespace MRSreg::segment;

int Segmentation::segmentEuclidean(PointCloudPtr cloud)
{
	// kd-tree object for searches.
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	kdtree->setInputCloud(cloud);

	pcl::EuclideanClusterExtraction<PointT> clustering; 	// Euclidean clustering object.
	clustering.setClusterTolerance(0.01); 						// Cluster tolerance to 1cm (original was 2cm); smaller values => more clusters
	clustering.setMinClusterSize(100);							// Minimum points a cluster can have
	clustering.setMaxClusterSize(25000);						// Maximum points a cluster can have
	clustering.setSearchMethod(kdtree);
	clustering.setInputCloud(cloud);
	std::vector<pcl::PointIndices> clusters;
	clustering.extract(clusters);

	// Loop over clusters
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
			// Load its points to a new cloud
			PointCloudPtr cluster(new PointCloud);
			for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
				cluster->points.push_back(cloud->points[*point]);
			cluster->width = cluster->points.size();
			cluster->height = 1;
			cluster->is_dense = true;

			// Save cloud to disk
			if (cluster->points.size() <= 0)
				break;
			ROS_INFO_STREAM("		Cluster " << currentClusterNum << " has " << cluster->points.size() << " points.");
			mkdir("registration_files/euclidean", S_IRWXU | S_IRWXG | S_IRWXO);
			std::string fileName = "registration_files/euclidean/cluster" + boost::to_string(currentClusterNum) + ".pcd";
			if(cluster->points.size())
			pcl::io::savePCDFileASCII(fileName, *cluster);
			currentClusterNum++;
	}
	return clusters.size();
}

int Segmentation::segmentRegionGrow(PointCloudPtr cloud, SurfaceNormalsPtr normals)
{
	// kd-tree object for searches.
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	kdtree->setInputCloud(cloud);

	// Region growing clustering object.
	pcl::RegionGrowing<PointT, NormalT> clustering;
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(10000);
	clustering.setSearchMethod(kdtree);
	clustering.setNumberOfNeighbours(30);
	clustering.setInputCloud(cloud);
	clustering.setInputNormals(normals);
	// Set the angle in radians for the smoothness threshold (the maximum allowable deviation of the normals).
	clustering.setSmoothnessThreshold(15.0 / 180.0 * M_PI); // 7 degrees.
	// Set the curvature threshold. The disparity between curvatures will be tested after the normal deviation check has passed.
	clustering.setCurvatureThreshold(1);
	std::vector <pcl::PointIndices> clusters;
	clustering.extract(clusters);

	// Loop over clusters
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
			// Load its points to a new cloud
			PointCloudPtr cluster(new PointCloud);
			for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
				cluster->points.push_back(cloud->points[*point]);
			cluster->width = cluster->points.size();
			cluster->height = 1;
			cluster->is_dense = true;
			// Save cloud to disk
			if (cluster->points.size() <= 0)
				break;
			ROS_INFO_STREAM("		Cluster " << currentClusterNum << " has " << cluster->points.size() << " points.");
			mkdir("registration_files/regiongrow", S_IRWXU | S_IRWXG | S_IRWXO);
			std::string fileName = "registration_files/regiongrow/cluster" + boost::to_string(currentClusterNum) + ".pcd";
			if(cluster->points.size())
			pcl::io::savePCDFileASCII(fileName, *cluster);
			currentClusterNum++;
	}
	return clusters.size();
}

PointCloudPtr Segmentation::chooseBestCluster()
{
	std::vector <pcl::PointIndices> clusters;
	std::string bestPCDFile;
	PointCloudPtr bestCluster(new PointCloud);
	int currentClusterNum = 1;
		for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		{
			// Load its points to a new cloud
			PointCloudPtr cluster(new PointCloud);
			// Save cloud to disk
			if (cluster->points.size() <= 0)
				break;
			std::string fileName = "/home/ir/catkin_ws/build/registration_files/regiongrow/cluster" + boost::to_string(currentClusterNum) + ".pcd";
			pcl::io::loadPCDFile<PointT>(fileName, *cluster);
			currentClusterNum++;
		}
		return bestCluster;
}

bool Segmentation::segmentPlaneSAC(PointCloudPtr cloud, PointCloudPtr output_cloud)
{
	//Inlier points
	PointCloudPtr inlierPoints (new PointCloud);
	//PointCloudPtr output_cloud (new PointCloud);
	*output_cloud = *cloud;															//Output cloud
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);			// Object for storing the plane model coeff	icients.
	pcl::SACSegmentation<PointT> segmentation;								// Create the segmentation object.
	segmentation.setInputCloud(cloud);												// Configure the object to look for a plane.
	segmentation.setModelType(pcl::SACMODEL_PLANE);									// Use RANSAC method.
	segmentation.setMethodType(pcl::SAC_RANSAC);									// Set the maximum allowed distance to the model.
	segmentation.setDistanceThreshold(0.01);										// Enable model coefficient refinement (optional).
	segmentation.setOptimizeCoefficients(true);

	pcl::PointIndices inlierIndices;
	segmentation.segment(inlierIndices, *coefficients);

	if (inlierIndices.indices.size() == 0){
		ROS_INFO("		Could not find any points that fitted the plane model.");
		return false;
	}
	else
	{
//Uncommnet this to see model coefficients
//			std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//					  << coefficients->values[1] << " "
//					  << coefficients->values[2] << " "
//					  << coefficients->values[3];
//			std::cout << std::endl;
		pcl::copyPointCloud<PointT>(*cloud, inlierIndices, *inlierPoints);	// Copy all inliers of the model to another cloud.
	}

	pcl::PointIndices::Ptr indices (new pcl::PointIndices);			// Delete inliers from scene
	*indices = inlierIndices;
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(output_cloud);
	extract.setIndices(indices);
	extract.setNegative(true);
	extract.filter(*output_cloud);
	return true;
}
