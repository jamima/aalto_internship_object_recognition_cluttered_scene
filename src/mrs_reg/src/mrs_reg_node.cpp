/*
 * 		  Name: mrs_reg_node.cpp
 *  Created on: Jan 14, 2015
 *    Modified: May 7, 2016
 *      Author: Rajkumar Muthusamy, Jaakko Mattila, Arthur Araújo
 */

// INCLUDES
#include <iostream>
//#include <fstream>
#include <vector>
#include <exception>
//#include <iterator>
#include <Eigen/Dense>

//ROS
//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>


#include "typedefs.h"

// CLASSES
#include "visualizer.h"
#include "filtering.h"
#include "segmentation.h"
#include "features.h"
#include "matching.h"

using namespace ros;

// MAIN
int main (int argc, char** argv)
{
			// 1. Initialize ROS
	ros::init (argc, argv, "mrs_registration_node");
	ros::NodeHandle nh;

			// 2a. Pointers Initialized
	PointCloudPtr scene				(new PointCloud);
	PointCloudPtr model				(new PointCloud);
	PointCloudPtr keypoints_scene 	(new PointCloud);
	PointCloudPtr keypoints_model 	(new PointCloud);
	PointCloudPtr filter_scene		(new PointCloud);
	PointCloudPtr filter_model		(new PointCloud);
	SurfaceNormalsPtr norm_scene	(new PointCloudN);
	SurfaceNormalsPtr norm_model	(new PointCloudN);
	LocalDescriptorsPtr scene_features (new LocalDescriptors);
	LocalDescriptorsPtr model_features (new LocalDescriptors);


			// 2b. Objects Initialized
	MRSreg::visualize::Visualizer viz;
	MRSreg::filter::Filtering filt;
	MRSreg::segment::Segmentation segm;
	MRSreg::feature::Features feat;
	MRSreg::match::Matching match;

		// 2c. Initialize Transformations
	Eigen::Matrix4f initial_transform;
	Eigen::Matrix4f icp_transformation;
	ROS_INFO("Pointers Initialized");


			// 3. Loading Scene and object point clouds
	pcl::io::loadPCDFile<PointT>("scene.pcd",*scene);
	model = viz.chooseModels();
	ROS_INFO("Loaded scene and model point clouds");
//	viz.viewMultiClouds(scene, model);


		// 4. Preprocessing
	//Filtering
	scene = filt.filterPassthroughZ(scene,"P_scene1.pcd");      //Pass Through Filter
	ROS_INFO("Passthrough filter 1/3 visualized for scene");

	scene = filt.filterPassthroughY(scene,"P_scene1.pcd");      //Pass Through Filter
	ROS_INFO("Passthrough filter 2/3 visualized for scene");

	scene = filt.filterPassthroughX(scene,"P_scene1.pcd");      //Pass Through Filter
	ROS_INFO("Passthrough filter 3/3 visualized for scene");

	scene = filt.filterVoxel(scene,"V_scene1.pcd");            	//Voxel Filter
	ROS_INFO("Voxel filter visualized for scene");

	scene = filt.filterStatistical(scene,"S_scene1.pcd");       //Statistical filter
	ROS_INFO("Statistical filter visualized for scene");

	//scene = filterRadial(scene,"R_scene1.pcd");          		//Radial filter
	//ROS_INFO("Radial filter visualized for scene");

	//scene = filterMLS(scene,"M_scene1.pcd");           		//MLS smoother
	//ROS_INFO("MLS filter visualized for scene");

	//model = PassThroughFilter(model,"P_model2.pcd");   		//Passthrough Filter
	//ROS_INFO("Passthrough filter visualized for model");

	model = filt.filterVoxel(model,"V_model2.pcd");           	//Voxel Filter
	ROS_INFO("Voxel filter visualized for model");

	//model = filterStatistical(model,"S_model2.pcd");       	//Statistical filter
	//ROS_INFO("Statistical filter visualized for model");

	//model = filterRadial(model,"R_model2.pcd");          		//Radial filter
	//ROS_INFO("Radial filter visualized for model");

	//model = filterMLS(model,"M_model2.pcd");           		//MLS smoother
	//ROS_INFO("MLS filter visualized for model");

		// Segmentation
	//segment a plane out
	if(segm.segmentPlaneSAC(scene,scene))
		ROS_INFO("Segmented plane removed from scene");
	else
		ROS_INFO("No plane was found for segmentation");

		// Keypoint extraction
//	keypoints_scene = feat.extractISSNoBorder(scene);
//	keypoints_model = feat.extractISSNoBorder(model);

		// Normals for keypoints
	float feature_radius = 0.08;
	norm_scene = feat.estimateNormals(scene,feature_radius,"Norm_scene1.pcd");
	ROS_INFO("Norm_scene1.pcd Generated");

	norm_model = feat.estimateNormals(model,feature_radius,"Norm_model2.pcd");
	ROS_INFO("Norm_model2.pcd Generated");

		////Visualizing scene and model with normals
	viz.viewMultiNormals(scene,norm_scene, model, norm_model);
	ROS_INFO("visualized scene and model Normals");

		//Feature Extraction
//	float feature_radius = 0.08;
//	feat.computeLocalDescriptors(scene, norm_scene, keypoints_scene, feature_radius, "scene_local_feature1.pcd");
	scene_features = feat.extractFPFH(scene, norm_scene,"scene_local_feature1.pcd") ;
	ROS_INFO("Scene local features generated");
	model_features = feat.extractFPFH(model, norm_model,"model_local_feature2.pcd") ;
	ROS_INFO("Model local features generated");
	ROS_INFO("Successfully extracted local descriptors");

			// 5. Processing
		// Matching using SAC
	ROS_INFO("Performing initial matching...");
	PointCloudPtr aligned_model (new PointCloud);
	initial_transform = match.matchInitial(scene, scene_features, model, model_features, aligned_model);
	match.printTransformation(initial_transform);

			// 6. Postprocessing
		// Time for ICP
	PointCloudPtr icp_aligned_points (new PointCloud);
	icp_transformation = match.matchFinal(aligned_model, scene, initial_transform, icp_aligned_points);
	match.printTransformation(icp_transformation);

//		//Total transformation in camera frame
//	Eigen::Matrix4f final_transformation = icp_transformation * initial_transform;
//	match.checkFinalScene(scene, model, final_transformation);
//	match.printTransformation(final_transformation);
	ROS_INFO("==========================Finished============================================");

	ros::spinOnce ();
}


