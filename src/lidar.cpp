#include "lidar.hpp"

/****************************************************
 *  Constructor & Deconstructor
 * 
 *  I- -
 *  O- -
 * **************************************************/
LidarFramework::LidarFramework(void) {}

LidarFramework::~LidarFramework(void) {}

/****************************************************
 *  Hello World function
 * 
 *  I- -
 *  O- -
 * **************************************************/
void LidarFramework::HelloWorld() {

    std::cout << "lidar says hello world" << std::endl;

}

/****************************************************
 *  XYZ and angle cloud filtering
 * 
 *  I- Non filetered cloud
 *  O- Filtered cloud by XYZ and angle
 * **************************************************/

pcl::PointCloud<pcl::PointXYZ> LidarFramework::CloudFiltering (pcl::PointCloud<pcl::PointXYZ>::Ptr nonFilteredCloud) {

    pcl::PointCloud<pcl::PointXYZ> auxFilteredCloud;
    
    // XYZ filter

    for (int i = 0; i < nonFilteredCloud->points.size(); i++) {

        pcl::PointXYZ Point = nonFilteredCloud->points[i];

        // If the point is above the sidewalk -> push back
        if (Point.z > -1.90) {
            auxFilteredCloud.points.push_back(Point);
        }
    }

    // Angle filter

    pcl::PointCloud<pcl::PointXYZ> FilteredCloud;
    float FieldOfView = 80;
    double FovMin = -(FieldOfView/2)*(3.14/180);;
    double FovMax = (FieldOfView/2)*(3.14/180);

    for (int i = 0; i < auxFilteredCloud->points.size(); i++) {

        pcl::PointXYZ Point = auxFilteredCloud->points[i];
        double PointAngle = atan2(Point.y, Point.x);

        // If the point is between Fov/2 and -Fov/2
        if (PointAngle < FovMax && PointAngle > FovMin) {
            FilteredCloud.points.push_back(Point);
        }

    }

    return FilteredCloud;

}


/****************************************************
 *  Cloud clustering
 * 
 *  I- Cloud, MaxIterations, Threshold
 *  O- InlierPoints
 * **************************************************/

void LidarFramework::CloudClustering (pcl::PointCloud<PointXYZ>::Ptr Cloud, float Tolerance, int MinSize, int MaxSize) {

    // -- KD-tree definition
    pcl::search::KdTree<pcl::PointXYZ>::Ptr Tree (new pcl::search::KdTree<pcl::PointXYZ>);
    Tree->setInputCloud(Cloud);

    // -- Euclidean cluster extraction object
    std::vector<pcl::PointIndices> ClusterIndices;
    pcl::EuclideanClusterExtraction<PointT> EC;
    EC.setClusterTolerance(Tolerance);
    EC.setMinClusterSize(MinSize);
    EC.setMaxClusterSize(MaxSize);
    EC.setSearchMethod(Tree);
    EC.setInputCloud(Cloud);
    EC.extract(ClusterIndices);


    // -- Clusters storage




}

/****************************************************
 *  Ransac3D algorithm (plane fitting algorithm)
 * 
 *  I- Cloud, MaxIterations, Threshold
 *  O- InlierPoints
 * **************************************************/

std::unordered_set<int> LidarFramework::Ransac3d (pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, int MaxIterations, float Threshold) {

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
            if (TargetPoints.count(i) > 0) {
                continue;
            }

            // Distance from a point to the plane
            pcl::PointXYZ Point = Cloud->points[i];
            float xi = Point.x;
            float yi = Point.y;
            float zi = Point.z;
            float Dist = fabs(a * xi + b * yi + c * zi)/den;

            // 4. If the dist < threshold -> point is an inlier
            if (Dist < Threshold) {
                TargetPoints.insert(i);
            }
            
            if (TargetPoints.size() > InlierPoints.size()) {
                InlierPoints = TargetPoints;
            }
            
        }

    }

    auto EndTime = std::chrono::steady_clock::now();
    auto ElapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime);
    std::cout << "Plane segmentation (Ransac3D) took: " << ElapsedTime.count() << " ms" << std::endl;

    return InlierPoints;

}


/****************************************************
 *  Angle filtering
 * 
 *  I- Non filetered cloud
 *  O- Filtered cloud by angle
 * **************************************************/

/*
pcl::PointCloud<pcl::PointXYZ> LidarFramework::AngleFilter (pcl::PointCloud<pcl::PointXYZ>::Ptr NonFilteredCloud) {

    pcl::PointCloud<pcl::PointXYZ> FilteredCloud;
    float FieldOfView = 80;
    double FovMin = -(FieldOfView/2)*(3.14/180);;
    double FovMax = (FieldOfView/2)*(3.14/180);

    for (int i = 0; i < NonFilteredCloud->points.size(); i++) {

        pcl::PointXYZ Point = NonFilteredCloud->points[i];
        double PointAngle = atan2(Point.y, Point.x);

        // If the point is between Fov/2 and -Fov/2
        if (PointAngle < FovMax && PointAngle > FovMin) {
            FilteredCloud.points.push_back(Point);
        }

    }

    return FilteredCloud;

}
*/