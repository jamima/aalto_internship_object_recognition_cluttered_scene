/*
 * features.cpp
 *
 *  Created on: Jul 4, 2016
 *      Author: jaakko
 */


#include "features.h"

using namespace MRSreg::feature;

// NORMAL ESTIMATION
SurfaceNormalsPtr Features::estimateNormals(PointCloudPtr input_cloud, float radius = 0.01, CloudFile NormalSave ="normal_save.pcd")
{
	// Setting the pointers
	PointCloudPtr Output_Cloud (new PointCloud);
	pcl::NormalEstimation<PointT, NormalT> normal_estimation;
	SurfaceNormalsPtr input_normals (new SurfaceNormals);
	pcl::search::Search<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

	// Setting the parameters
	normal_estimation.setSearchMethod(tree);
	normal_estimation.setInputCloud(input_cloud);
	normal_estimation.setRadiusSearch(radius); // 0.1


	// Executing and saving
	normal_estimation.compute(*input_normals);
	pcl::copyPointCloud(*input_normals,*Output_Cloud);
	pcl::io::savePCDFile (NormalSave, *Output_Cloud);
	ROS_INFO("Normals estimated");
	return(input_normals);
}

SurfaceNormalsPtr Features::estimateClusterNormals (PointCloudPtr input_cloud, float radius, CloudFile NormalSave = "normal_save.pcd")
{
		//Compute centroid
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*input_cloud, centroid);
		float vx,vy,vz;
		vx = centroid[0];
		vy = centroid[1];
		vz = centroid[2];

		PointCloudPtr Output_Cloud (new PointCloud);
		pcl::NormalEstimation<PointT, NormalT> normals;
		normals.setInputCloud(input_cloud);
		normals.setViewPoint(vx,vy,vz);
		normals.setRadiusSearch(0.1);
		SurfaceNormalsPtr input_normals (new SurfaceNormals);
		normals.compute( *input_normals );

		SurfaceNormals::iterator it;
		int count = 0;
		for(it = input_normals->begin(); it < input_normals->end(); it++){
			Eigen::Vector4f n = input_normals->points[count].getNormalVector4fMap();
			pcl::Normal normal(-n[0],-n[1],-n[2]);
			input_normals->points[count] = normal;
			count++;
		}

		pcl::copyPointCloud(*input_normals,*Output_Cloud);
		pcl::io::savePCDFile (NormalSave, *Output_Cloud);
		return(input_normals);
}

// KEYPOINT EXTRACTION
//PointCloudPtr Features::convertCloud(PointCloudPtr input_cloud)
//{
//	PointCloudPtr converted_cloud;
//	pcl::copyPointCloud(*input_cloud, *converted_cloud);
//	return converted_cloud;
//}

PointCloudPtr Features::detectKeypoints (PointCloudPtr & points, SurfaceNormalsPtr & normals,
                 float min_scale = 0.01, int nr_octaves = 3,
				 int nr_scales_per_octave = 4, float min_contrast = 0.001)
{
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
  sift_detect.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
  sift_detect.setMinimumContrast (min_contrast);
  sift_detect.setInputCloud (points);
  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
  sift_detect.compute (keypoints_temp);
  PointCloudPtr keypoints (new PointCloud);
  pcl::copyPointCloud (keypoints_temp, *keypoints);

  return (keypoints);
}

/* ISS from http://www.pointclouds.org/blog/gsoc12/gballin/iss.php
 * Note:Modified types
 * In order to run the ISSKeypoint3D in this modality, the user must set:
 * -	the salient radius (strict positive)
 * -	the non maxima suppression radius (strict positive)
 * -	the border radius (equal to 0.0, as the default value)
 * while he can chose to set or not all the other parameters.
 * The following code snippet allows to run the ISS 3D detector
 * without the border estimation.:
 */
PointCloudPtr Features::extractISSNoBorder(PointCloudPtr model)
{
	//
	//  ISS3D parameters
	//
	double iss_salient_radius_;
	double iss_non_max_radius_;
	double iss_gamma_21_ (0.975);
	double iss_gamma_32_ (0.975);
	double iss_min_neighbors_ (5);
	int iss_threads_ (4);

	PointCloudPtr model_keypoints (new PointCloud ());
	KdSearch::Ptr tree (new KdSearch ());

	double model_resolution;

	// Compute model_resolution

	iss_salient_radius_ = 6 * model_resolution;
	iss_non_max_radius_ = 4 * model_resolution;

	//
	// Compute keypoints
	//
	pcl::ISSKeypoint3D<PointT, PointT> iss_detector;

	iss_detector.setSearchMethod (tree);
	iss_detector.setSalientRadius (iss_salient_radius_);
	iss_detector.setNonMaxRadius (iss_non_max_radius_);
	iss_detector.setThreshold21 (iss_gamma_21_);
	iss_detector.setThreshold32 (iss_gamma_32_);
	iss_detector.setMinNeighbors (iss_min_neighbors_);
	iss_detector.setNumberOfThreads (iss_threads_);
	iss_detector.setInputCloud(model);
	iss_detector.compute (*model_keypoints);
	return model_keypoints;
}

/* ISS version with border estimation
 * 	In order to run the ISSKeypoint3D in this modality, the user must set:
 *	-	the salient radius (strict positive)
 *	-	the non maxima suppression radius (strict positive)
 *	-	the border radius (strict positive)
 *	-	the normal radius (strict positive)
 *	while he can chose to set or not all the other parameters.
 *	The following code snippet allows to run the ISS 3D detector
 *	with the border estimation.:
 */
PointCloudPtr Features::extractISSWithBorder(PointCloudPtr model)
{
	//
	//  ISS3D parameters
	//
	double iss_salient_radius_;
	double iss_non_max_radius_;
	double iss_normal_radius_;
	double iss_border_radius_;
	double iss_gamma_21_ (0.975);
	double iss_gamma_32_ (0.975);
	double iss_min_neighbors_ (5);
	int iss_threads_ (4);


	PointCloudPtr model_keypoints (new PointCloud ());
	KdSearch::Ptr tree (new KdSearch ());

	// Fill in the model cloud

	double model_resolution;

	// Compute model_resolution

	iss_salient_radius_ = 6 * model_resolution;
	iss_non_max_radius_ = 4 * model_resolution;
	iss_normal_radius_ = 4 * model_resolution;
	iss_border_radius_ = 1 * model_resolution;

	//
	// Compute keypoints
	//
	pcl::ISSKeypoint3D<PointT, PointT> iss_detector;

	iss_detector.setSearchMethod (tree);
	iss_detector.setSalientRadius (iss_salient_radius_);
	iss_detector.setNonMaxRadius (iss_non_max_radius_);

	iss_detector.setNormalRadius (iss_normal_radius_);
	iss_detector.setBorderRadius (iss_border_radius_);

	iss_detector.setThreshold21 (iss_gamma_21_);
	iss_detector.setThreshold32 (iss_gamma_32_);
	iss_detector.setMinNeighbors (iss_min_neighbors_);
	iss_detector.setNumberOfThreads (iss_threads_);
	iss_detector.setInputCloud (model);
	iss_detector.compute (*model_keypoints);
	return model_keypoints;
}

PointCloudPtr Features::extractSIFT(PointCloudPtr cloud_xyz)
{
	  std::cout << "points: " << cloud_xyz->points.size () <<std::endl;

	  // Parameters for sift computation
	  const float min_scale = 0.01;
	  const int n_octaves = 3;
	  const int n_scales_per_octave = 4;
	  const float min_contrast = 0.001;

	  // Estimate the normals of the cloud_xyz
	  pcl::NormalEstimation<PointT, PointNT> ne;
	  pcl::PointCloud<PointNT>::Ptr cloud_normals (new pcl::PointCloud<PointNT>);
	  KdSearch::Ptr tree_n(new KdSearch());

	  ne.setInputCloud(cloud_xyz);
	  ne.setSearchMethod(tree_n);
	  ne.setRadiusSearch(0.2);
	  ne.compute(*cloud_normals);

	  // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
	  for(size_t i = 0; i<cloud_normals->points.size(); ++i)
	  {
		cloud_normals->points[i].x = cloud_xyz->points[i].x;
		cloud_normals->points[i].y = cloud_xyz->points[i].y;
		cloud_normals->points[i].z = cloud_xyz->points[i].z;
	  }

	  // Estimate the sift interest points using normals values from xyz as the Intensity variants
	  pcl::SIFTKeypoint<PointNT, PointT> sift;
	  PointCloudPtr result;
	  pcl::search::KdTree<PointNT>::Ptr tree(new pcl::search::KdTree<PointNT> ());
	  sift.setSearchMethod(tree);
	  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	  sift.setMinimumContrast(min_contrast);
	  sift.setInputCloud(cloud_normals);
	  sift.compute(*result);
	  std::cout << "No of SIFT points in the result are " << result->points.size () << std::endl;
	  return result;
}

//PointCloudPtr Features::extractUniformSampling(PointCloudPtr cloud)
//{
//	pcl::PointCloud<int> indices;
//	pcl::UniformSampling<PointT> uniform_sampling;
//	PointCloudPtr cloud_keypoints (new PointCloud ());
//	uniform_sampling.setInputCloud (cloud);
//	uniform_sampling.setRadiusSearch (0.05f);
//	uniform_sampling.compute(indices);
//	std::cout << "Uniform Sampling done, total points: " << cloud->size () << "; Selected Keypoints: " << cloud_keypoints->size () << std::endl;
//	return cloud_keypoints;
//}


// FEATURE DESCRIPTORS
// FPFH 33: Fast
FPFH_Descriptor::Ptr Features::extractFPFH (PointCloudPtr input_cloud, SurfaceNormalsPtr Input_Normal, CloudFile FeatureSave = "cloud_local_feature1.pcd")
{
	// Setting the pointers
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr Input_features (new pcl::PointCloud<pcl::FPFHSignature33> ());
	pcl::FPFHEstimation<PointT, NormalT, pcl::FPFHSignature33> descriptors;

	// Setting the parameters
	descriptors.setRadiusSearch(0.01); //previous (0.09) //0.01
	KdSearch::Ptr tree(new KdSearch);
	descriptors.setSearchMethod(tree);
	descriptors.setInputCloud(input_cloud);
	descriptors.setInputNormals(Input_Normal);

	// Executing and saving
	descriptors.compute (*Input_features);
	ROS_INFO("FPFH Features extracted");
	pcl::io::savePCDFileASCII (FeatureSave, *Input_features);
	return(Input_features);
}

// PFH 125: Includes more neighbor connections than FPFH
PFH_Descriptor::Ptr Features::extractPFH (PointCloudPtr input_cloud, SurfaceNormalsPtr Input_Normal, CloudFile FeatureSave = "cloud_local_feature1.pcd")
{
	// Setting the pointers
	PFH_Descriptor::Ptr Input_features (new PFH_Descriptor);
	pcl::PFHEstimation<PointT, NormalT, pcl::PFHSignature125> descriptorss;

	// Setting the parameters
	KdSearch::Ptr tree (new KdSearch);
	descriptorss.setRadiusSearch (0.01); //previous (0.09)
	descriptorss.setSearchMethod (tree);
	descriptorss.setInputCloud (input_cloud);
	descriptorss.setInputNormals (Input_Normal);

	// Executing and saving
	descriptorss.compute (*Input_features);
	ROS_INFO("PFH Features extracted");
	pcl::io::savePCDFileASCII (FeatureSave, *Input_features);
	return(Input_features);
}

// SHOT 352: Good choice if the looking for a bit more power
SHOT_Descriptor::Ptr Features::extractSHOT (PointCloudPtr input_cloud, SurfaceNormalsPtr Input_Normal, CloudFile FeatureSave = "cloud_local_feature1.pcd")
{
	// Setting the pointers
	SHOT_Descriptor::Ptr Input_features (new SHOT_Descriptor);
	pcl::SHOTEstimation<PointT, NormalT, pcl::SHOT352> shotdescriptors;

	// Setting the parameters
	KdSearch::Ptr tree (new KdSearch);
	shotdescriptors.setRadiusSearch (0.01); //previous (0.09)
	// shotdescriptors.setSearchMethod (tree);
	shotdescriptors.setInputCloud (input_cloud);
	shotdescriptors.setInputNormals (Input_Normal);

	// Executing and saving
	shotdescriptors.compute (*Input_features);
	ROS_INFO("SHOT Features extracted");
	pcl::io::savePCDFileASCII (FeatureSave, *Input_features);
	return(Input_features);
}

// DSC 1980: Very powerful, but might be too slow
DSC_Descriptor::Ptr Features::extractDSC (PointCloudPtr input_cloud, SurfaceNormalsPtr Input_Normal, CloudFile FeatureSave = "cloud_local_feature1.pcd")
{
	// Setting the pointers
	DSC_Descriptor::Ptr Input_features (new DSC_Descriptor);
	pcl::ShapeContext3DEstimation<PointT, NormalT, pcl::ShapeContext1980>  sc3d;
	KdSearch::Ptr tree (new KdSearch);

	// Setting the parameters
	sc3d.setSearchMethod (tree);
	sc3d.setInputCloud (input_cloud);
	sc3d.setInputNormals (Input_Normal);
	sc3d.setRadiusSearch(0.05);
	// The minimal radius value for the search sphere, to avoid being too sensitive
	// in bins close to the center of the sphere.
	sc3d.setMinimalRadius(0.05 / 10.0);
	// Radius used to compute the local point density for the neighbors
	// (the density is the number of points within that radius).
	sc3d.setPointDensityRadius(0.05 / 5.0);

	// Executing and saving
	sc3d.compute (*Input_features);
	ROS_INFO("DSC Features extracted");
	pcl::io::savePCDFileASCII (FeatureSave, *Input_features);
	return(Input_features);
}

LocalDescriptorsPtr Features::computeLocalDescriptors (PointCloudPtr & points, SurfaceNormalsPtr & normals,
                         const PointCloudPtr & keypoints, float feature_radius = 0.05, CloudFile feature_save = "cloud_local_feature1.pcd")
{
  pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
  fpfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  fpfh_estimation.setRadiusSearch (feature_radius);
  fpfh_estimation.setSearchSurface (points);
  fpfh_estimation.setInputNormals (normals);
  fpfh_estimation.setInputCloud (keypoints);
  LocalDescriptorsPtr local_descriptors (new LocalDescriptors);
  fpfh_estimation.compute (*local_descriptors);
  pcl::io::savePCDFileASCII (feature_save, *local_descriptors);
  return (local_descriptors);
}

GlobalDescriptorsPtr Features::computeGlobalDescriptor (PointCloudPtr & points, SurfaceNormalsPtr & normals, CloudFile FeatureSave = "cloud_global_feature1.pcd")
{
  pcl::VFHEstimation<PointT, NormalT, GlobalDescriptorT> vfh_estimation;
  vfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  vfh_estimation.setInputCloud (points);
  vfh_estimation.setInputNormals (normals);
  GlobalDescriptorsPtr global_descriptor (new GlobalDescriptors);
  vfh_estimation.compute (*global_descriptor);

  return (global_descriptor);
}

ObjectFeatures computeFeatures (PointCloudPtr input_cloud)
{
  ObjectFeatures obj;
  Features feat;
  obj.points = input_cloud;
  obj.normals = feat.estimateNormals(input_cloud, 0.05);
  obj.keypoints = feat.detectKeypoints (input_cloud, obj.normals, 0.005, 10, 8, 1.5);
  obj.local_descriptors = feat.computeLocalDescriptors (input_cloud, obj.normals, obj.keypoints, 0.1);
  obj.global_descriptors = feat.computeGlobalDescriptor (input_cloud, obj.normals);

  return (obj);
}
