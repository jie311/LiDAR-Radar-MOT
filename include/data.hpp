#ifndef DATA_H_
#define DATA_H_

#include <iostream>
#include <cmath>
#include <chrono>
#include <unordered_set>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/impl/point_types.hpp>

typedef struct {

	float centroid_x;                       // Local centroid (with respect to the "base_link" frame)
	float centroid_y;
	float centroid_z;
	float global_centroid_x;                // Global centroid (with respect to the "map" frame)
	float global_centroid_y;
	float global_centroid_z;
    float w;
    float h;
    float d;
	double r;
	double g;
	double b;
    double a;
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	// string type;
	int object_id;
    double time;

} Object;

#endif /* DATA_H_ */