/*
 * Visualizer.h
 *
 *  Created on: Jul 1, 2016
 *      Author: jaakko
 */

#ifndef MRS_REG_SRC_VISUALIZER_H_
#define MRS_REG_SRC_VISUALIZER_H_
#include <pcl/common/pca.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_lib_io.h>
#include "typedefs.h"


namespace MRSreg {
	namespace visualize {
		class Visualizer {
		public:
			Visualizer() = default;
			virtual ~Visualizer() = default;
			int viewSingleCloud(PointCloudPtr input_cloud) const;
			int viewMultiClouds(PointCloudPtr scene_cloud, PointCloudPtr model_cloud) const;
			int viewSingleNormal(PointCloudPtr input_cloud, SurfaceNormalsPtr cloud_normal) const;
			int viewMultiNormals(PointCloudPtr scene_cloud, SurfaceNormalsPtr scene_normal, PointCloudPtr model_cloud, SurfaceNormalsPtr model_normal) const;
			PointCloudPtr chooseModels();
		};

	} /* namespace visualize */
} /* namespace MRSreg */

#endif /* MRS_REG_SRC_VISUALIZER_H_ */
