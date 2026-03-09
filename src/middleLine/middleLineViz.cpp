#include "middleLine/middleLineViz.hpp"
#include "rclcpp/rclcpp.hpp"

visualization_msgs::msg::MarkerArray createGatesMarkerArray(
    const std::vector<Gate>& gates, const std::string& frame, double timestamp)
{
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
    marker.ns = "middle_line_gates";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.08; // line width
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;
    marker.pose.orientation.w = 1.0;

    for (const auto& gate : gates)
    {
        geometry_msgs::msg::Point p1, p2;
        p1.x = gate.cone1.x();
        p1.y = gate.cone1.y();
        p1.z = gate.cone1.z();
        p2.x = gate.cone2.x();
        p2.y = gate.cone2.y();
        p2.z = gate.cone2.z();
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    arr.markers.push_back(marker);
    return arr;
}

visualization_msgs::msg::MarkerArray createDelaunayMarkerArray(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges, const std::string& frame,
    double timestamp)
{
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
    marker.ns = "delaunay_edges";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.03; // thin lines
    marker.color.r = 0.6f;
    marker.color.g = 0.6f;
    marker.color.b = 0.6f;
    marker.color.a = 0.35f;
    marker.pose.orientation.w = 1.0;

    for (const auto& edge : edges)
    {
        geometry_msgs::msg::Point p1, p2;
        p1.x = edge.first.x();
        p1.y = edge.first.y();
        p1.z = edge.first.z();
        p2.x = edge.second.x();
        p2.y = edge.second.y();
        p2.z = edge.second.z();
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    arr.markers.push_back(marker);
    return arr;
}
