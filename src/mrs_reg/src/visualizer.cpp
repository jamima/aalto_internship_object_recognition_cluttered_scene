/*
 * visualizer.cpp
 *
 *  Created on: Jul 4, 2016
 *      Author: jaakko
 */

#include "visualizer.h"

using namespace MRSreg::visualize;

PointCloudPtr Visualizer::chooseModels()
{
	PointCloudPtr models (new PointCloud);
	int modelno;
	std::cout << "Choose Model No: 0 - 4 \n"
			<< "0 - D_redtable_unscaled.pcd(default)\n"
			<< "1 - A_bluechair_unscaled.pcd\n"
			<< "2 - B_pinkchair_unscaled_new.pcd\n"
			<< "3 - E_redchair_unscaled.pcd\n"
			<< "4 - G_woodchair_unscaled.pcd\n"<< std::endl;

	std::cin >> modelno;

	switch (modelno){
		case 0:
			pcl::io::loadPCDFile<PointT>("D_redtable_unscaled.pcd",*models);
			break;
		case 1:
			pcl::io::loadPCDFile<PointT>("A_bluechair_unscaled.pcd",*models);
			break;
		case 2:
			pcl::io::loadPCDFile<PointT>("B_pinkchair_unscaled_new.pcd",*models);
			break;
		case 3:
			pcl::io::loadPCDFile<PointT>("E_redchair_unscaled.pcd",*models);
			break;
		case 4:
			pcl::io::loadPCDFile<PointT>("G_woodchair_unscaled.pcd",*models);
			break;
		default:
			pcl::io::loadPCDFile<PointT>("D_redtable_unscaled.pcd",*models);
	}
	return models;
}

int Visualizer::viewSingleCloud(PointCloudPtr input_cloud) const
{
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("Normal Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> colorpose(input_cloud, 255, 0, 0);
	viewer->addPointCloud(input_cloud, colorpose, "input_cloud");
	while (!viewer->wasStopped()){ viewer->spinOnce ();}
	return 0;
}
int Visualizer::viewMultiClouds(PointCloudPtr scene_cloud, PointCloudPtr model_cloud) const
{
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("Normal Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> colorpose(model_cloud, 255, 0, 0);
	viewer->addPointCloud(model_cloud, colorpose, "model_cloud");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> colorposes(scene_cloud, 155, 0, 0);
	viewer->addPointCloud(scene_cloud, colorposes, "scene_cloud");
	while (!viewer->wasStopped()){ viewer->spinOnce ();}
	return 0;
}
int Visualizer::viewSingleNormal (PointCloudPtr input_cloud, SurfaceNormalsPtr cloud_normal) const
{
	std::cerr<<"Show the normals...(Close the pop-out viewer to continue)"<<std::endl;
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("Normal Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> colorpose(input_cloud, 255, 0, 0);
	viewer->addPointCloud(input_cloud, colorpose, "input_cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"input_cloud");
	viewer->addPointCloudNormals<PointT, NormalT> (input_cloud, cloud_normal, 15, 0.05,"normals");
	while (!viewer->wasStopped()){ viewer->spinOnce ();}
	return 0;
}
int Visualizer::viewMultiNormals (PointCloudPtr scene_cloud, SurfaceNormalsPtr scene_normal,
		PointCloudPtr model_cloud, SurfaceNormalsPtr model_normal) const
{
	ROS_INFO("scene and model point cloud and their normals after filtering");
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("Normal Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> colorposes(scene_cloud, 215, 0, 0);
	viewer->addPointCloud(scene_cloud, colorposes, "scene");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> colorpose(model_cloud, 125, 0, 0);
	viewer->addPointCloud(model_cloud, colorpose, "model");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"scene");
	viewer->addPointCloudNormals<PointT, NormalT> (scene_cloud,scene_normal, 10, 0.05, "scene_cloud");
	viewer->addPointCloudNormals<PointT, NormalT> (model_cloud,model_normal, 10, 0.05, "object_cloud");
	while (!viewer->wasStopped()){ viewer->spinOnce ();}
	return 0;
}
