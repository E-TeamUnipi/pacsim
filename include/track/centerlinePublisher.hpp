#ifndef CENTERLINE_PUBLISHER_HPP
#define CENTERLINE_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "types.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class CenterlinePublisher
{
public:
    void initialize(std::shared_ptr<rclcpp::Node> node);
    void setTrack(const Track& track, const std::string& frameId, double time);
    void publishRaw(double time);
    void publishFront(double time, const Eigen::Vector3d& trans, const Eigen::Vector3d& rot);

private:
    LandmarkList centerlineRawMapFrame;
    bool hasCenterlineRaw = false;
    bool hasLastClosestIdx = false;
    std::size_t lastClosestIdx = 0;

    LandmarkList centerlineSmoothedMapFrame;
    bool hasCenterlineSmoothed = false;
    bool hasLastClosestIdxSmoothed = false;
    std::size_t lastClosestIdxSmoothed = 0;

    std::string mapFrame = "map";
    std::size_t rawPublishCallCounter = 0;
    std::size_t frontPublishCallCounter = 0;
    std::size_t frontPublishCallCounterSmoothed = 0;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centerlineRawVizPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centerlineRawFrontVizPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centerlineSmoothedVizPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centerlineSmoothedFrontVizPub;
};

#endif /* CENTERLINE_PUBLISHER_HPP */
