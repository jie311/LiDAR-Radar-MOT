#ifndef LIDAR_H_
#define LIDAR_H_

#include <iostream>
#include <unordered_set>

    class LidarFramework {

        private:

            // --- XyzFilter ---

            // --- AngleFilter ---

            // --- Ransac3d ---
            int MaxIterations;
            float Threshold;
            pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud;

        public:

            // Constructor
            LidarFramework();

            // De-constructor
            ~LidarFramework();

            void HelloWorld();
            std::unordered_set<int> Ransac3d (pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, int MaxIterations, float Threshold);
                       
    };


#endif /* LIDAR_H_*/