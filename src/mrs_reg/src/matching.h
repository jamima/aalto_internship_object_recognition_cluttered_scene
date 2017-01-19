/*
 * matching.h
 *
 *  Created on: Jul 1, 2016
 *      Author: jaakko
 */

#ifndef MRS_REG_SRC_MATCHING_H_
#define MRS_REG_SRC_MATCHING_H_
//#define PIOVER180 (3.14159265359/180.0) // for fromEuler()
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "visualizer.h"
#include "typedefs.h"

namespace MRSreg {
	namespace match {

		class Matching {
		public:
			Matching() = default;
			virtual ~Matching() = default;
			Eigen::Matrix4f matchInitial (PointCloudPtr scene, LocalDescriptorsPtr scene_features,
					PointCloudPtr model, LocalDescriptorsPtr model_features, PointCloudPtr aligned_model);
//			PointCloudPtr matchInitialPCD (PointCloudPtr,LocalDescriptorsPtr,PointCloudPtr,LocalDescriptorsPtr);
			Eigen::Matrix4f matchFinal (PointCloudPtr new_aligned_model, PointCloudPtr scene,
					Eigen::Matrix4f initial_Transform, PointCloudPtr icp_aligned_model);
//			PointCloudPtr matchFinalPCD (PointCloudPtr, PointCloudPtr, Eigen::Matrix4f);
			void checkFinalScene(PointCloudPtr, PointCloudPtr,Eigen::Matrix4f);
			void printTransformation(Eigen::Matrix4f transformation);

		};
	} /* namespace match */
} /* namespace MRSreg */

#endif /* MRS_REG_SRC_MATCHING_H_ */
