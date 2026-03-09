#ifndef MIDDLELINEVIZ_HPP
#define MIDDLELINEVIZ_HPP

#include "middleLine/middleLineComputer.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <string>

/**
 * Build a MarkerArray visualizing the selected gates (green lines between cone pairs).
 */
visualization_msgs::msg::MarkerArray createGatesMarkerArray(
    const std::vector<Gate>& gates, const std::string& frame, double timestamp);

/**
 * Build a MarkerArray visualizing all Delaunay triangulation edges (thin grey lines).
 */
visualization_msgs::msg::MarkerArray createDelaunayMarkerArray(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges, const std::string& frame,
    double timestamp);

#endif /* MIDDLELINEVIZ_HPP */
