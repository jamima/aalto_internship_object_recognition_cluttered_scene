/*
 * segmentation.h
 *
 *  Created on: Jul 4, 2016
 *      Author: jaakko
 */

#ifndef MRS_REG_SRC_SEGMENTATION_H_
#define MRS_REG_SRC_SEGMENTATION_H_
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sys/stat.h>
#include "typedefs.h"


namespace MRSreg {
	namespace segment {

	class Segmentation {
	public:
		Segmentation() = default;
		virtual ~Segmentation()  = default;
		int segmentEuclidean(PointCloudPtr input_cloud);
		int segmentRegionGrow(PointCloudPtr input_cloud, SurfaceNormalsPtr input_normals);
		PointCloudPtr chooseBestCluster();
		bool segmentPlaneSAC(PointCloudPtr input_cloud, PointCloudPtr output_cloud);

	};
	} /* namespace segment */
} /* namespace MRSreg */

#endif /* MRS_REG_SRC_SEGMENTATION_H_ */
