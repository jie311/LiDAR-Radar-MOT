#include "../include/lidar.hpp"

/****************************************************
 *  XYZ and angle cloud filtering
 * 
 *  I- Non filetered cloud
 *  O- Filtered cloud by XYZ and angle
 * **************************************************/

pcl::PointCloud<pcl::PointXYZ> CloudFiltering (pcl::PointCloud<pcl::PointXYZ>::Ptr nonFilteredCloud) {

    pcl::PointCloud<pcl::PointXYZ> auxFilteredCloud;
    
    // -- XYZ filtering
    for (int i = 0; i < nonFilteredCloud->points.size(); i++) {

        pcl::PointXYZ Point = nonFilteredCloud->points[i];

        // If the point is above the sidewalk -> push back
        if (Point.z > -1.90) {
            auxFilteredCloud.points.push_back(Point);
        }
    }

    // -- Angle filtering
    pcl::PointCloud<pcl::PointXYZ> FilteredCloud;
    float FieldOfView = 80;
    double FovMin = -(FieldOfView/2)*(3.14/180);;
    double FovMax = (FieldOfView/2)*(3.14/180);

    for (int i = 0; i < auxFilteredCloud.points.size(); i++) {

        pcl::PointXYZ Point = auxFilteredCloud.points[i];
        double PointAngle = atan2(Point.y, Point.x);

        // If the point is between Fov/2 and -Fov/2
        if (PointAngle < FovMax && PointAngle > FovMin) {
            FilteredCloud.points.push_back(Point);
        }

    }

    return FilteredCloud;

}

/****************************************************
 *  Plane segmentation with RANSAC-3D algorithm
 *  (needs subsequent cloud separation)
 * 
 *  I- Cloud, MaxIterations, Threshold
 *  O- Pair of clouds (plane and obstacles)
 * **************************************************/

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> PlaneSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, int MaxIterations, float Threshold) {

    auto StartTime = std::chrono::steady_clock::now();

    std::unordered_set<int> InlierPoints;
    while (MaxIterations--) {

        // 1. Choose three random points
        std::unordered_set<int> TargetPoints;
        while(TargetPoints.size() < 3) {
            TargetPoints.insert(rand()%Cloud->size());
        }

        // 2. Parameters required for the plane equation
        //      ax + by + cz + d = 0 
        // 2.a. Points characterization
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = TargetPoints.begin();
        x1 = Cloud->points[*itr].x;
        y1 = Cloud->points[*itr].y;
        z1 = Cloud->points[*itr].z;
        itr++;
        x2 = Cloud->points[*itr].x;
        y2 = Cloud->points[*itr].y;
        z2 = Cloud->points[*itr].z;
        itr++;
        x3 = Cloud->points[*itr].x;
        y3 = Cloud->points[*itr].y;
        z3 = Cloud->points[*itr].z;
        
        // 2.b. Plane characterization
        float a, b, c, d, den;
        a = ((y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y2));
        b = ((z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z2));
        c = ((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x2));
        d = - (a * x1 + b * y1 + c * z1);
        den = sqrt(a * a + b * b + c * c);

        // 3. For all the points in the cloud, estimate distance to
        //    the plane
        for (int i = 0; i<Cloud->points.size(); i++) {
            
            // If the point is already an inlier, continue
            if (TargetPoints.count(i) > 0) {    continue;   }

            // Distance from a point to the plane
            pcl::PointXYZ Point = Cloud->points[i];
            float xi = Point.x;
            float yi = Point.y;
            float zi = Point.z;
            float Dist = fabs(a * xi + b * yi + c * zi)/den;

            // If the dist < threshold -> point is an inlier
            if (Dist < Threshold) {
                TargetPoints.insert(i);
            }
            
            // 4. Store the results of the plane that has more points
            if (TargetPoints.size() > InlierPoints.size()) {
                InlierPoints = TargetPoints;
            }
            
        }

    }

    // 5. Creating two new point clouds, one with obstacles and other with plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Divide the floor and the rest in 2 different point clouds
    for (int i = 0; i < (int) Cloud->points.size(); i++) {

        pcl::PointXYZ Point = Cloud->points[i];
        if (InlierPoints.count(i)) {
            planeCloud->points.push_back(Point);
        } else {
            obstCloud->points.push_back(Point);
        }

    }

    auto EndTime = std::chrono::steady_clock::now();
    auto ElapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime);
    std::cout << "Plane segmentation (Ransac3D) took: " << ElapsedTime.count() << " ms" << std::endl;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;

}

 /****************************************************
 *  Clustering extraction
 * 
 *  I- Original cloud, Inliers to split
 *  O- New cloud with only inliers
 * **************************************************/
/*
 void ClusteringExtraction (pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float Tolerance, int MinSize, int MaxSize, std::vector<Object>* outputObjects, int* numOutputObjects) {

    // -- KD-tree object definition
    pcl::search::KdTree<pcl::PointXYZ>::Ptr Tree (new pcl::search::KdTree<pcl::PointXYZ>);
    Tree->setInputCloud(Cloud);

    // -- Configuration of the search method of extraction
    std::vector<pcl::PointIndices> ClusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> EC;
    EC.setClusterTolerance(Tolerance);
    EC.setMinClusterSize(MinSize);
    EC.setMaxClusterSize(MaxSize);
    EC.setSearchMethod(Tree);
    EC.setInputCloud(Cloud);
    EC.extract(ClusterIndices);

    // -- Clusters storage
    for (std::vector<pcl::PointIndices>::const_iterator it = ClusterIndices.begin (); it != ClusterIndices.end (); ++it) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr CloudCluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
            CloudCluster->push_back ((*Cloud)[*pit]);
        }
        
        CloudCluster->width = CloudCluster->size ();
        CloudCluster->height = 1;
        CloudCluster->is_dense = true;

        // Initialize point cloud vertices
        float xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY, zmin = INFINITY, zmax = -INFINITY;
        float xcen = -INFINITY, ycen = -INFINITY, zcen = -INFINITY;

        // Cloud vertices search
        for (int i = 0; i < CloudCluster->points.size(); i++){

            if (CloudCluster->points[i].x < xmin) {xmin = CloudCluster->points[i].x;}
            if (CloudCluster->points[i].x > xmax) {xmax = CloudCluster->points[i].x;}
            if (CloudCluster->points[i].y < ymin) {ymin = CloudCluster->points[i].y;}
            if (CloudCluster->points[i].y > ymax) {ymax = CloudCluster->points[i].y;}
            if (CloudCluster->points[i].z < zmin) {zmin = CloudCluster->points[i].z;}
            if (CloudCluster->points[i].z > zmax) {zmax = CloudCluster->points[i].z;}

        }

        // Calculus of the centroid
        xcen = (xmax + xmin) / 2.0;
        ycen = (ymax + ymin) / 2.0;
        zcen = (zmax + zmin) / 2.0;

        // Creation of an object type
        Object object;

        object.x_min = xmin;
        object.x_max = xmax;
        object.y_min = ymin;
        object.y_max = ymax;
        object.z_min = zmin;
        object.z_max = zmax;

        object.d = xmax - xmin;
        object.w = ymax - ymin;
        object.h = zmax - zmin;

        object.centroid_x = xcen;
        object.centroid_y = ycen;
        object.centroid_z = zcen;

        object.type = "none";
        object.cloud = CloudCluster;

        outputObjects->push_back(object);
        *numOutputObjects = *numOutputObjects + 1;
    }

}


void ClusteringExtraction (pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float Tolerance, int MinSize, int MaxSize,
                             std::vector<Object>* outputObjects, int* numOutputObjects) {

    std:vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    KdTree* tree = new KdTree;


}
*/

void ClusterHelper (int idx, pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, std::vector<int>& Cluster,
                     std::vector<bool>& Processed, KdTree* Tree, float DistanceTol) {

    Processed[idx] = true;
    Cluster.push_back(idx);
    std::vector<int> nearest = Tree->search(Cloud->points[idx], DistanceTol);

    for (auto id : nearest) {
        if(!Processed[id]) {     ClusterHelper(id, Cloud, Cluster, Processed, Tree, DistanceTol)     }
    }

}


