/*
 * features.h
 *
 *  Created on: Jul 1, 2016
 *      Author: jaakko
 */

#ifndef MRS_REG_SRC_FEATURES_H_
#define MRS_REG_SRC_FEATURES_H_
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/concatenate.h>
//#include <pcl/common/copy_point.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "typedefs.h"

namespace MRSreg {
	namespace feature {
	// source https://github.com/PointCloudLibrary/pcl/tree/master/doc/tutorials/content/sources/iccv2011
	struct ObjectFeatures
	{
	  PointCloudPtr points;
	  SurfaceNormalsPtr normals;
	  PointCloudPtr keypoints;
	  LocalDescriptorsPtr local_descriptors;
	  GlobalDescriptorsPtr global_descriptors;
	};
	class Features {
	public:
		Features() = default;
		virtual ~Features() = default;
		ObjectFeatures computeFeatures (PointCloudPtr input_cloud);
		SurfaceNormalsPtr estimateNormals (PointCloudPtr input_cloud, float radius, CloudFile);
		SurfaceNormalsPtr estimateClusterNormals (PointCloudPtr input_cloud, float radius, CloudFile);
//		PointCloudPtr convertCloud(PointCloudPtr input_cloud);
		PointCloudPtr detectKeypoints (PointCloudPtr& points, SurfaceNormalsPtr& normals,
				float min_scale, int nr_octaves,
				int nr_scales_per_octave, float min_contrast);
		PointCloudPtr extractISSNoBorder(PointCloudPtr input_cloud);
		PointCloudPtr extractISSWithBorder(PointCloudPtr input_cloud);
		PointCloudPtr extractSIFT(PointCloudPtr input_cloud);
//			PointCloudPtr extractUniformSampling(PointCloudPtr input_cloud);
		FPFH_Descriptor::Ptr extractFPFH (PointCloudPtr, SurfaceNormalsPtr, CloudFile);
		PFH_Descriptor::Ptr extractPFH (PointCloudPtr, SurfaceNormalsPtr, CloudFile);
		SHOT_Descriptor::Ptr extractSHOT (PointCloudPtr, SurfaceNormalsPtr, CloudFile);
		DSC_Descriptor::Ptr extractDSC (PointCloudPtr, SurfaceNormalsPtr, CloudFile);
		LocalDescriptorsPtr computeLocalDescriptors (PointCloudPtr& points, SurfaceNormalsPtr& normals,
			const PointCloudPtr& keypoints, float feature_radius, CloudFile feature_save);
		GlobalDescriptorsPtr computeGlobalDescriptor (PointCloudPtr& points, SurfaceNormalsPtr& normals, CloudFile feature_save);

	};

	} /* namespace feature */
} /* namespace MRSreg */

#endif /* MRS_REG_SRC_FEATURES_H_ */
