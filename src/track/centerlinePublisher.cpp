#include "track/centerlinePublisher.hpp"

#include "ros2Helpers.hpp"
#include "transform.hpp"

#include <limits>

namespace
{
visualization_msgs::msg::MarkerArray buildCenterlineMarkerArray(
    const LandmarkList& centerline, const std::string& frame, double time, int markerId)
{
    visualization_msgs::msg::MarkerArray markerArray;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = rclcpp::Time(static_cast<uint64_t>(time * 1e9));
    marker.ns = "pacsim/centerline";
    marker.id = markerId;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    for (const auto& lm : centerline.list)
    {
        geometry_msgs::msg::Point p;
        p.x = lm.position.x();
        p.y = lm.position.y();
        p.z = lm.position.z();
        marker.points.push_back(p);
    }

    markerArray.markers.push_back(marker);
    return markerArray;
}
} // namespace

void CenterlinePublisher::initialize(std::shared_ptr<rclcpp::Node> node)
{
    auto latchedQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    centerlineRawVizPub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/pacsim/track/centerline_raw/viz", latchedQos);
    centerlineRawFrontVizPub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/pacsim/track/centerline_raw_front/viz", latchedQos);
}

void CenterlinePublisher::setTrack(const Track& track, const std::string& frameId, double time)
{
    mapFrame = frameId;
    centerlineRawMapFrame.list = track.centerline_raw;
    centerlineRawMapFrame.frame_id = frameId;
    centerlineRawMapFrame.timestamp = time;

    hasCenterlineRaw = !centerlineRawMapFrame.list.empty();

    if (hasCenterlineRaw)
    {
        centerlineRawVizPub->publish(buildCenterlineMarkerArray(centerlineRawMapFrame, frameId, time, 0));
    }
}

void CenterlinePublisher::publishRaw(double time)
{
    if (!hasCenterlineRaw)
    {
        return;
    }

    centerlineRawMapFrame.timestamp = time;
    centerlineRawVizPub->publish(buildCenterlineMarkerArray(centerlineRawMapFrame, mapFrame, time, 0));
}

void CenterlinePublisher::publishFront(double time, const Eigen::Vector3d& trans, const Eigen::Vector3d& rot)
{
    if (!hasCenterlineRaw)
    {
        return;
    }

    centerlineRawMapFrame.timestamp = time;

    const auto& centerlinePoints = centerlineRawMapFrame.list;
    const std::size_t pointCount = centerlinePoints.size();
    if (pointCount == 0)
    {
        return;
    }

    std::size_t closestIdx = 0;
    double closestDistSq = std::numeric_limits<double>::infinity();
    for (std::size_t idx = 0; idx < pointCount; ++idx)
    {
        const double distSq = (centerlinePoints[idx].position - trans).squaredNorm();
        if (distSq < closestDistSq)
        {
            closestDistSq = distSq;
            closestIdx = idx;
        }
    }

    constexpr double lookaheadDistanceM = 10.0;
    constexpr std::size_t maxFrontPoints = 300;

    LandmarkList centerlineFrontMapFrame;
    centerlineFrontMapFrame.frame_id = mapFrame;
    centerlineFrontMapFrame.timestamp = time;
    centerlineFrontMapFrame.list.reserve(std::min(pointCount, maxFrontPoints));

    centerlineFrontMapFrame.list.push_back(centerlinePoints[closestIdx]);
    std::size_t currentIdx = closestIdx;
    double accumulatedDistance = 0.0;

    for (std::size_t step = 0; step + 1 < pointCount && centerlineFrontMapFrame.list.size() < maxFrontPoints; ++step)
    {
        const std::size_t nextIdx = (currentIdx + 1) % pointCount;
        const double segmentDistance = (centerlinePoints[nextIdx].position - centerlinePoints[currentIdx].position).norm();

        if (accumulatedDistance + segmentDistance > lookaheadDistanceM)
        {
            break;
        }

        accumulatedDistance += segmentDistance;
        centerlineFrontMapFrame.list.push_back(centerlinePoints[nextIdx]);
        currentIdx = nextIdx;

        if (currentIdx == closestIdx)
        {
            break;
        }
    }

    LandmarkList centerlineFrontCarFrame = transformLmList(centerlineFrontMapFrame, trans, rot);
    centerlineFrontCarFrame.frame_id = "car";
    centerlineFrontCarFrame.timestamp = time;

    centerlineRawFrontVizPub->publish(buildCenterlineMarkerArray(centerlineFrontCarFrame, "car", time, 1));
}
