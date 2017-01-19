/*
 * Filtering.h
 *
 *  Created on: Jul 1, 2016
 *      Author: jaakko
 */

#ifndef MRS_REG_SRC_FILTERING_H_
#define MRS_REG_SRC_FILTERING_H_
//#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "typedefs.h"


namespace MRSreg {
	namespace filter {

		class Filtering {
		public:
			Filtering() = default;
			virtual ~Filtering() = default;
//			pcl::RangeImage::Ptr getRangeImage(PointCloudPtr input_cloud);
			PointCloudPtr filterPassthroughZ(PointCloudPtr input_cloud, CloudFile savefile);
			PointCloudPtr filterPassthroughY(PointCloudPtr input_cloud, CloudFile savefile);
			PointCloudPtr filterPassthroughX(PointCloudPtr input_cloud, CloudFile savefile);
			PointCloudPtr filterVoxel(PointCloudPtr input_cloud, CloudFile savefile);
			PointCloudPtr filterStatistical(PointCloudPtr input_cloud, CloudFile savefile);
			PointCloudPtr filterRadial(PointCloudPtr input_cloud, CloudFile savefile);
			PointCloudPtr filterMLS (PointCloudPtr input_cloud, CloudFile savefile);

		};
	} /* namespace filter */
} /* namespace MRSreg */

#endif /* MRS_REG_SRC_FILTERING_H_ */
