/*
 * filtering.cpp
 *
 *  Created on: Jul 4, 2016
 *      Author: jaakko
 */

#include "filtering.h"

using namespace MRSreg::filter;

// http://pointclouds.org/documentation/tutorials/range_image_creation.php
//		pcl::RangeImage::Ptr Filtering::getRangeImage(PointCloudPtr input_cloud)
//		{
//			float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
//			float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
//			float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
//			Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
//			pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//			float noiseLevel = 0.00;
//			float minRange = 0.0f;
//			int borderSize = 1;
//			pcl::RangeImage::Ptr rangeImage;
//			rangeImage->createFromPointCloud(input_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
//			                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
//			return rangeImage;
//		}

PointCloudPtr Filtering::filterPassthroughZ(PointCloudPtr Input_Cloud, CloudFile filter_save)
{
	// Setting the pointers
	PointCloudPtr Output_Cloud (new PointCloud);
	pcl::PassThrough<PointT> limit;

	// Setting the parameters
	limit.setInputCloud (Input_Cloud);
	limit.setFilterFieldName ("z");
	limit.setFilterLimits (0.0, 1.4);
	limit.setKeepOrganized (true);
	//pass.setFilterLimitsNegative (true);

	// Executing and saving
	limit.filter (*Output_Cloud);
	pcl::io::savePCDFileASCII (filter_save, *Output_Cloud);
	ROS_INFO("Pass Through Filter:: Kinect scene filtered");
	//viewSingleCloud(Output_Cloud);
	return(Output_Cloud);
}

PointCloudPtr Filtering::filterPassthroughY(PointCloudPtr Input_Cloud, CloudFile filter_save)
{
	// Setting the pointers
	PointCloudPtr Output_Cloud (new PointCloud);
	pcl::PassThrough<PointT> limit;

	// Setting the parameters
	limit.setInputCloud (Input_Cloud);
	limit.setFilterFieldName ("y");
	limit.setFilterLimits (-0.9, 0.25);
	limit.setKeepOrganized (true);
	//pass.setFilterLimitsNegative (true);

	// Executing and saving
	limit.filter (*Output_Cloud);
	pcl::io::savePCDFileASCII (filter_save, *Output_Cloud);
	ROS_INFO("Pass Through Filter:: Kinect scene filtered");
	//viewSingleCloud(Output_Cloud);
	return(Output_Cloud);
}

PointCloudPtr Filtering::filterPassthroughX(PointCloudPtr Input_Cloud, CloudFile filter_save)
{
	// Setting the pointers
	PointCloudPtr Output_Cloud (new PointCloud);
	pcl::PassThrough<PointT> limit;

	// Setting the parameters
	limit.setInputCloud (Input_Cloud);
	//limit.setFilterFieldName ("z");
	//limit.setFilterLimits (0.0, 1.5);
	limit.setFilterFieldName ("x");
	limit.setFilterLimits (-0.5, 0.5);
	limit.setKeepOrganized (true);
	//pass.setFilterLimitsNegative (true);

	// Executing and saving
	limit.filter (*Output_Cloud);
	pcl::io::savePCDFileASCII (filter_save, *Output_Cloud);
	ROS_INFO("Pass Through Filter:: Kinect scene filtered");
	//viewSingleCloud(Output_Cloud);
	return(Output_Cloud);
}

// Voxel Filter to Downsample
PointCloudPtr Filtering::filterVoxel(PointCloudPtr Input_Cloud, CloudFile filter_save)
{
	// Setting the pointers
	PointCloudPtr Output_Cloud (new PointCloud);
	pcl::VoxelGrid<PointT> Downsample;

	// Setting the parameters
	Downsample.setInputCloud (Input_Cloud);
	Downsample.setLeafSize (0.004f, 0.004f, 0.004f); //previous (0.001f, 0.001f, 0.001f)

	// Execute, print and save
	Downsample.filter(*Output_Cloud);
	std::cerr<<"Dowsampled to "<< Output_Cloud->points.size()<<" points"<< std::endl;
	pcl::io::savePCDFile (filter_save, *Output_Cloud);
	//ROS_INFO("Downsampled Points Shown... close viewer to proceed");
	//viewSingleCloud(Output_Cloud);
	return(Output_Cloud);
}

PointCloudPtr Filtering::filterStatistical(PointCloudPtr Input_Cloud,CloudFile filter_save)
{
	// Setting the pointers
	PointCloudPtr Output_Clouds (new PointCloud);
	pcl::StatisticalOutlierRemoval<PointT> stat(true);

	// Setting the parameters
	stat.setInputCloud (Input_Cloud);
	stat.setMeanK(50);//choose a suitable parameter manually//50-30
	stat.setStddevMulThresh(0.3);//choose a suitable parameter manually//0.5-0.8

	// Executing and saving
	stat.filter(*Output_Clouds);
	pcl::io::savePCDFile (filter_save, *Output_Clouds);
	//ROS_INFO("Statistical_Filter Points after remove Shown... close viewer to proceed");
	//viewSingleCloud(Output_Clouds);
	return(Output_Clouds);
}

PointCloudPtr Filtering::filterRadial(PointCloudPtr Input_Cloud,CloudFile filter_save)
{
	// Setting the pointers
	PointCloudPtr Output_Cloud (new PointCloud);
	pcl::RadiusOutlierRemoval<PointT> radial(true);

	// Setting the parameters
	radial.setInputCloud(Input_Cloud);
	radial.setMinNeighborsInRadius(30);//choose a suitable parameter manually
	radial.setRadiusSearch(0.09);//choose a suitable parameter manually

	// Executing and saving
	radial.filter(*Output_Cloud);
	pcl::io::savePCDFile (filter_save, *Output_Cloud);
	//ROS_INFO("radial_Filter Points after remove Shown... close viewer to proceed");
	//viewSingleCloud(Output_Cloud);
	return(Output_Cloud);
}

PointCloudPtr Filtering::filterMLS (PointCloudPtr Input_Cloud,CloudFile filter_save)
{
	// Setting the pointers
	pcl::PointCloud<PointNT> mls_points;
	PointCloudPtr Output_Cloud (new PointCloud);
	KdSearch::Ptr tree (new KdSearch);
	pcl::MovingLeastSquares<PointT, PointNT> mls;

	// Setting the parameters
	mls.setComputeNormals (false);
	mls.setInputCloud (Input_Cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.01);

	// Executing and saving
	mls.process (mls_points);
	ROS_INFO("Finely smoothed... close viewer to proceed");
	pcl::copyPointCloud(*Input_Cloud,*Output_Cloud);
	pcl::copyPointCloud(mls_points,*Output_Cloud);
	//viewSingleCloud(Output_Cloud);
	pcl::io::savePCDFile (filter_save, *Output_Cloud);
	return(Output_Cloud);
}
