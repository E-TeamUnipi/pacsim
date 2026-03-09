#ifndef MIDDLELINECOMPUTER_HPP
#define MIDDLELINECOMPUTER_HPP

#include "types.hpp"
#include <Eigen/Core>
#include <vector>

/**
 * Tuning parameters for the middle-line computation.
 * Defaults match temp/test_planner_FSG_FSE.py (BlindPathPlanner).
 */
struct MiddleLineParams
{
    double minGateWidth = 3.0;   // Minimum distance between two cones to form a gate
    double maxGateWidth = 5.0;   // Maximum distance between two cones to form a gate
    double searchRadius = 12.0;  // How far ahead to look for the next gate
    double maxOrthoError = 0.90; // Reject edges too parallel to the driving direction
    double directionInertia = 0.5; // Blending weight: 0.5 * old_dir + 0.5 * new_dir (more reactive)
    int maxSteps = 300;          // Maximum number of gates to traverse
};

/**
 * A gate: the two cone endpoints that form a valid crossing.
 */
struct Gate
{
    Eigen::Vector3d cone1;
    Eigen::Vector3d cone2;
};

/**
 * Full result of the middle-line computation, including debug data.
 */
struct MiddleLineResult
{
    std::vector<Eigen::Vector3d> midpoints;                        // Ordered gate midpoints (the middle line)
    std::vector<Gate> gates;                                       // Selected gates (cone pairs)
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> delaunayEdges; // All Delaunay triangulation edges
};

/**
 * Computes the middle line of a track using 2D Delaunay triangulation
 * and a greedy gate-selection walk.
 *
 * This is a C++ port of the algorithm in temp/PathPlanner.py.
 * All left and right cones are merged into one point cloud (color-blind),
 * triangulated, then a greedy walk selects valid "gates" (triangle edges
 * that cross the track) and records their midpoints.
 *
 * @param track           The loaded track (left_lane + right_lane used)
 * @param startPosition   Car start position (x, y, z)
 * @param startYaw        Car start yaw angle in radians
 * @param params          Algorithm tuning parameters
 * @return MiddleLineResult with midpoints, gates, and Delaunay edges
 */
MiddleLineResult computeMiddleLine(const Track& track, const Eigen::Vector3d& startPosition,
    double startYaw, const MiddleLineParams& params = MiddleLineParams());

#endif /* MIDDLELINECOMPUTER_HPP */
