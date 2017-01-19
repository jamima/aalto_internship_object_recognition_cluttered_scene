/*
* matching.cpp
*
*  Created on: Jul 4, 2016
*      Author: jaakko
*/

#include "matching.h"


using namespace MRSreg::match;

// MATCHING
// Matching using SAC, change descriptor parameter accordingly
Eigen::Matrix4f Matching::matchInitial (PointCloudPtr scene, LocalDescriptorsPtr scene_features,
		PointCloudPtr model,LocalDescriptorsPtr model_features, PointCloudPtr aligned_model)
{
	// Setting the pointers
	pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> Initial;
	MRSreg::visualize::Visualizer viz;

	// Setting the parameters
	Initial.setMinSampleDistance (0.01);
	Initial.setMaxCorrespondenceDistance (0.1);
	Initial.setMaximumIterations (100);
	//Initial.setInputCloud (model);
	Initial.setInputSource(model);
	Initial.setSourceFeatures (model_features);
	Initial.setInputTarget (scene);
	Initial.setTargetFeatures (scene_features);

	//2. Align
	Initial.align (*aligned_model);
//	double score;
//	int trials = 0;
//	do
//	{
//		Initial.align (*aligned_model);
//		score = Initial.getFitnessScore();
//		ROS_INFO_STREAM("		SCORE: " << score);
//		trials++;
//		if(trials == 10)
//			break;
//	} while(score > 0.0007);
	ROS_INFO("Initial Matching Done");

	// Calculating the 6-DoF transformation
	Eigen::Matrix4f initial_transform = Initial.getFinalTransformation();

	// Saving transformation to text file
	std::ofstream transs("object2cam_initial.txt");
	transs << initial_transform << std::endl;

	pcl::io::savePCDFile ("Initial_new_aligned_model.pcd", *aligned_model);

	ROS_INFO("Visualizing initial alignment <close viewer to proceed>");
	viz.viewMultiClouds(scene, aligned_model);
	return(initial_transform);
}

//PointCloudPtr Matching::matchInitialPCD (PointCloudPtr model,LocalDescriptorsPtr model_features,PointCloudPtr scene,LocalDescriptorsPtr scene_features)
//{
//	// Setting the pointers
//	pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> Initial;
//	PointCloudPtr aligned_model(new PointCloud);
//	PointCloudPtr new_aligned_model(new PointCloud);
//	PointCloudPtr final_aligned(new PointCloud);
//
//	// Setting the parameters
//	Initial.setMinSampleDistance (0.01);
//	Initial.setMaxCorrespondenceDistance (0.1);
//	Initial.setMaximumIterations (100);
//	//Initial.setInputCloud (model);
//	Initial.setInputSource(model);
//	Initial.setSourceFeatures (model_features);
//	Initial.setInputTarget (scene);
//	Initial.setTargetFeatures (scene_features);
//	Initial.align (*aligned_model);
//	ROS_INFO("Initial Matching Done");
//
//	// Getting the 6-DoF transformation
//	Eigen::Matrix4f initial_Transform = Initial.getFinalTransformation();
//	Eigen::Matrix3f rotation = initial_Transform.block<3, 3>(0, 0);
//	Eigen::Vector3f translation = initial_Transform.block<3, 1>(0, 3);
//
//	// Executing and saving
//	transformPointCloud(*model,*new_aligned_model,initial_Transform);
//	pcl::io::savePCDFile ("Initial_new_aligned_model.pcd", *new_aligned_model);
//	PointCloud aligned_model_scene = *new_aligned_model;
//	aligned_model_scene += *scene;
//	pcl::io::savePCDFile ("final_aligned_model_scene.pcd", aligned_model_scene);
//	ROS_INFO("Successfully Done Initial Alignment");
//	pcl::copyPointCloud(aligned_model_scene,*final_aligned);
//	//viewSingleCloud(final_aligned);
//	return(new_aligned_model);
//}

// POSTPROCESSING
Eigen::Matrix4f Matching::matchFinal (PointCloudPtr new_aligned_model, PointCloudPtr scene, Eigen::Matrix4f initial_Transform, PointCloudPtr icp_aligned_points)
{
	std::cout << "Refine matching..." <<std::endl;
	MRSreg::visualize::Visualizer viz;
	pcl::IterativeClosestPoint<PointT, PointT> icp; //Final ICP
	//icp.setInputCloud(new_aligned_model);
	icp.setInputSource(new_aligned_model);
	icp.setInputTarget(scene);
	icp.setMaximumIterations(100);
	icp.setMaxCorrespondenceDistance(0.2);
	icp.setRANSACOutlierRejectionThreshold(0.15);//0.1 works
	icp.align(*icp_aligned_points);
	if(icp_aligned_points->points.size())
		pcl::io::savePCDFile("pose_object_align_icp.pcd", *icp_aligned_points);
	*icp_aligned_points += *scene;
	if(icp_aligned_points->points.size())
		pcl::io::savePCDFile("final_pose_object_align.pcd", *icp_aligned_points);

	// Fitness score
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	Eigen::Matrix4f tranm = icp.getFinalTransformation();
	viz.viewSingleCloud(icp_aligned_points);
	return(tranm);
}

//PointCloudPtr Matching::matchFinalPCD (PointCloudPtr new_aligned_model, PointCloudPtr scene, Eigen::Matrix4f initial_Transform)
//{
//	std::cerr<<"Refine matching..."<< std::endl;
//	pcl::IterativeClosestPoint<PointT, PointT> icp; //Final ICP
//	//icp.setInputCloud(new_aligned_model);
//	icp.setInputSource(new_aligned_model);
//
//	icp.setInputTarget(scene);
//	icp.setMaximumIterations(50);
//	icp.setMaxCorrespondenceDistance(0.2);
//	icp.setRANSACOutlierRejectionThreshold(0.15);//0.1 works
//	pcl::PointCloud<PointT> icpFinal;
//	icp.align(icpFinal);
//	pcl::io::savePCDFile ("pose_object_align_icp.pcd", icpFinal);
//	icpFinal += *scene;
//	pcl::io::savePCDFile ("Final_pose_object_align.pcd", icpFinal);
//
//	//loading
//	PointCloudPtr icpfinalpoints (new PointCloud);
//	pcl::io::loadPCDFile ("pose_object_align_icp.pcd", *icpfinalpoints);
//	std::cerr<<"Show the icp result:"<<std::endl;
//
//	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//	icp.getFitnessScore() << std::endl;
//	Eigen::Matrix4f tranm = icp.getFinalTransformation();
//	std::cout << "Transform matrix:"<< std::endl << tranm << std::endl;
//	std::ofstream trans("trans_icp.txt");
//	trans<<tranm<<std::endl;
//
//	PointCloudPtr finalpoints (new PointCloud);
//	//pcl::io::loadPCDFile ("pose_object_align_icp.pcd", *icpfinalpoints);
//	pcl::io::loadPCDFile ("final_pose_object_align.pcd", *finalpoints);
//	std::cerr<<"Show the icp result:"<<std::endl;
//
//	Eigen::Matrix4f t_final = initial_Transform * tranm;
//	std::cout << "Final Transform matrix:"<< std::endl << t_final << std::endl;
//	std::ofstream transf("trans_final.txt");
//	transf<<t_final<<std::endl;
//	return(finalpoints);
//}

void Matching::checkFinalScene(PointCloudPtr scene, PointCloudPtr model, Eigen::Matrix4f t_final)
{
	MRSreg::visualize::Visualizer viz;
	PointCloudPtr icpfinalpoints (new PointCloud);
	pcl::io::loadPCDFile ("pose_object_align_icp.pcd", *icpfinalpoints);
	PointCloudPtr finals_aligned_model(new PointCloud);
	pcl::transformPointCloud(*model,*finals_aligned_model,t_final);
	pcl::io::savePCDFile ("icp_aligned_model.pcd", *finals_aligned_model);
	PointCloud complete_aligned_model_scene = *finals_aligned_model;
	complete_aligned_model_scene += *scene;
	pcl::io::savePCDFile ("complete_final_aligned_model_scene.pcd", complete_aligned_model_scene);

	PointCloudPtr complete_scene (new PointCloud);
	ROS_INFO("======================================================================");
	ROS_INFO(" CLOUD VIEWER  please check that the complete transformation works");
	ROS_INFO("======================================================================");
	pcl::io::loadPCDFile ("complete_final_aligned_model_scene.pcd", *complete_scene);
	viz.viewSingleCloud(complete_scene);

	std::cout << "camera to object_icp Final Transformation matrix:"<< std::endl << t_final << std::endl;
	std::ofstream transf("cam2obj_final_Transformation.txt");
	transf<<t_final<<std::endl;
	ROS_INFO("======================================================================");
	ROS_INFO(" CLOUD VIEWER  FINAL ALIGNED");
	ROS_INFO("======================================================================");
	viz.viewMultiClouds(icpfinalpoints,scene);
}

void Matching::printTransformation(Eigen::Matrix4f target_transformation)
{
	Eigen::Matrix3f finalrotation = target_transformation.block<3, 3>(0, 0);
	Eigen::Vector3f finaltranslation = target_transformation.block<3, 1>(0, 3);

	std::cout << "ICP(parent) to camera frame transformation matrix:" << std::endl << std::endl;
	printf("\t\t    | %6.3f %6.3f %6.3f | \n", finalrotation(0, 0), finalrotation(0, 1), finalrotation(0, 2));
	printf("\t\tR = | %6.3f %6.3f %6.3f | \n", finalrotation(1, 0), finalrotation(1, 1), finalrotation(1, 2));
	printf("\t\t    | %6.3f %6.3f %6.3f | \n\n", finalrotation(2, 0), finalrotation(2, 1), finalrotation(2, 2));
	printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", finaltranslation(0), finaltranslation(1), finaltranslation(2));

	Eigen::Matrix4f Tmm = target_transformation.inverse();
	tf::Vector3 origin;
	tf::Matrix3x3 tf3d;
	tf::Quaternion tfqt;
	tf::Quaternion itfqt;

	ROS_INFO("Translation:");
	origin.setValue(
		static_cast<double>(Tmm(0,3)),static_cast<double>(Tmm(1,3)),static_cast<double>(Tmm(2,3)));

	ROS_INFO("Rotation:");
	tf3d.setValue(
		static_cast<double>(Tmm(0,0)), static_cast<double>(Tmm(0,1)), static_cast<double>(Tmm(0,2)),
		static_cast<double>(Tmm(1,0)), static_cast<double>(Tmm(1,1)), static_cast<double>(Tmm(1,2)),
		static_cast<double>(Tmm(2,0)), static_cast<double>(Tmm(2,1)), static_cast<double>(Tmm(2,2)));

	tf3d.getRotation(tfqt);
	tf3d.transpose().getRotation(itfqt);
	std::cout << tfqt;

	ROS_INFO("Translation:");
	printf("\t\tt = < %0.6f, %0.6f, %0.6f>\n", origin.x(), origin.y(), origin.z());
	ROS_INFO("object to camera quaternion values:");
	printf("Camera(parent) to robot = < %0.6f, %0.6f, %0.6f, %0.6f>\n", tfqt.x(), tfqt.y(), tfqt.z(), tfqt.w());
	ROS_INFO("titfqt camera  to object quaternion values:");
	printf("Robot(parent) to camera = < %0.6f, %0.6f, %0.6f, %0.6f>\n", itfqt.x(), itfqt.y(), itfqt.z(), itfqt.w());
}

