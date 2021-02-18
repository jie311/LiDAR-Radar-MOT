#ifndef OBJECT_HPP_
#define OBJECT_HPP_

typedef struct {
    float centroid_x; // Local centroid (with respect to the "base_link" frame)
    float centroid_y;
    float centroid_z;
    float global_centroid_x; // Global centroid (with respect to the "map" frame)
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
    std::string type;
    int object_id;
    double time;
} Object;

#endif