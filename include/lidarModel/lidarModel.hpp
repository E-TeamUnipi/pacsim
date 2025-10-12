#ifndef LIDAR_MODEL_HPP
#define LIDAR_MODEL_HPP

#include "logger.hpp"
#include "types.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <random>


#define TOTAL_RAY 72000
#define POINTS_PER_ARCH 450
#define X_CONE_DIM 0.228
#define Y_CONE_DIM 0.228
#define Z_CONE_DIM 0.325
#define RADIUS X_CONE_DIM / 2.0
#define LIDAR_Z 0.5

class lidarModel
{
private:
public:
    lidarModel(/* args */);
    ~lidarModel();
    void test(std::shared_ptr<Logger> logger);
    void generatePointCloud(LandmarkList landmarks, std::shared_ptr<Logger> logger);
    void fillOcclusionsArray(double* array, LandmarkList landmarks);
    void generateFloorPoints(double* occlusions, pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    double getConeFlattedSurface(Landmark landmark);
    uint32_t sampleOnCone(double surface, double distance);
    std::tuple<double, double, double> samplePointOnCone(double pos_x, double pos_y, double pos_z, double distance);
    void printConePositions(LandmarkList landmarks, std::shared_ptr<Logger> logger);
};




#endif // LIDAR_MODEL_HPP