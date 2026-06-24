#ifndef LIDAR_MODEL_HPP
#define LIDAR_MODEL_HPP

#include "logger.hpp"
#include "types.hpp"
#include "configParser.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <random>
#include <memory>
#include "sensorModels/perceptionSensor.hpp"


// #define TOTAL_RAY 72000
// #define POINTS_PER_ARCH 450
#define X_CONE_DIM 0.228
#define Y_CONE_DIM 0.228
#define Z_CONE_DIM 0.325
#define RADIUS X_CONE_DIM / 2.0

class lidarModel
{
public:
    lidarModel(/* args */);

    ~lidarModel();

    void readConfig(ConfigElement& config);

    void test(std::shared_ptr<Logger> logger);

    pcl::PointCloud<pcl::PointXYZRGB> generatePointCloud(LandmarkList landmarks, std::shared_ptr<Logger> logger);

    bool generateSegment(LandmarkList landmarks, pcl::PointCloud<pcl::PointXYZRGB>& out_cloud, std::shared_ptr<Logger> logger);

    void setPerceptionSensor(std::shared_ptr<PerceptionSensor> ps) { perceptionSensor = ps; }

    bool RunTick(double simTime, LandmarkList& trackAsLMList, Eigen::Vector3d t, Eigen::Vector3d rEulerAngles, pcl::PointCloud<pcl::PointXYZRGB>& out_cloud, std::shared_ptr<Logger> logger);


    void fillOcclusionsArray(double* array, LandmarkList landmarks, int start_idx, int end_idx);

    void generateFloorPoints(double* occlusions, pcl::PointCloud<pcl::PointXYZRGB>& cloud, int start_idx, int end_idx);

    double getConeFlattedSurface();

    uint32_t sampleOnCone(double surface, double distance);

    std::tuple<double, double, double> samplePointOnCone(double pos_x, double pos_y, double pos_z, double distance);

    void printConePositions(LandmarkList landmarks, std::shared_ptr<Logger> logger);

    uint32_t getTotalRay() { return total_ray; }

    double getLidarX() const { return lidar_x; }

    double getLidarY() const { return lidar_y; }

    double getLidarZ() const { return lidar_z; }

    double getRate() const { return rate; }

    int countChannelsFast(double D);

    uint16_t getNumSegments() const { return num_segments; }

    std::string getPerceptionSensorName() const { return perception_sensor_name; }

private:
    bool isConeInSegment(const Landmark& lm, double phi_start, double phi_end);
    void applyNoise(pcl::PointCloud<pcl::PointXYZRGB>& cloud);

    uint32_t total_ray;
    uint16_t points_per_arch;
    uint16_t num_channel;
    uint16_t num_segments;
    uint16_t current_segment;
    double min_angle_horizontal;
    double max_angle_horizontal;
    double lidar_x;
    double lidar_y;
    double lidar_z;
    double angular_uncertainty;
    double rate;
    std::string perception_sensor_name;
    std::shared_ptr<PerceptionSensor> perceptionSensor;

    pcl::PointCloud<pcl::PointXYZRGB> accumulated_cloud;
    double lastLidarSegmentTime = 0.0;
};




#endif // LIDAR_MODEL_HPP
