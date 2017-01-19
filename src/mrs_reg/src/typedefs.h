/*
 * Typedefs.h
 *
 *  Created on: Jul 1, 2016
 *      Author: jaakko
 */

#ifndef MRS_REG_SRC_TYPEDEFS_H_
#define MRS_REG_SRC_TYPEDEFS_H_
#include <pcl/search/kdtree.h>

//typedef pcl::PointNormal PointNT;
//typedef pcl::PointCloud<PointNT> PointCloudNT;
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloudDP;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudDRP;
//typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudDRPA;
//typedef pcl::PointCloud<pcl::PointWithScale> PointCloudScaled;
typedef pcl::PointCloud<pcl::Normal> PointCloudN;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFH_Descriptor;
typedef pcl::PointCloud<pcl::PFHSignature125> PFH_Descriptor;
typedef pcl::PointCloud<pcl::SHOT352> SHOT_Descriptor;
typedef pcl::PointCloud<pcl::ShapeContext1980> DSC_Descriptor;
typedef std::string CloudFile;

// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGBA points
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
typedef pcl::search::KdTree<PointT> KdSearch;

// Define "SurfaceNormals" to be a pcl::PointCloud of pcl::Normal points
typedef pcl::Normal NormalT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<NormalT> SurfaceNormals;
typedef pcl::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;

// Define "LocalDescriptors" to be a pcl::PointCloud of pcl::FPFHSignature33 points
typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT> LocalDescriptors;
typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
typedef pcl::PointCloud<LocalDescriptorT>::ConstPtr LocalDescriptorsConstPtr;

// Define "GlobalDescriptors" to be a pcl::PointCloud of pcl::VFHSignature308 points
typedef pcl::VFHSignature308 GlobalDescriptorT;
typedef pcl::PointCloud<GlobalDescriptorT> GlobalDescriptors;
typedef pcl::PointCloud<GlobalDescriptorT>::Ptr GlobalDescriptorsPtr;
typedef pcl::PointCloud<GlobalDescriptorT>::ConstPtr GlobalDescriptorsConstPtr;



#endif /* MRS_REG_SRC_TYPEDEFS_H_ */
