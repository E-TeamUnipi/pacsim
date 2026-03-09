/**
 * Middle-line computation for Formula Student tracks.
 *
 * Port of the algorithm from temp/PathPlanner.py:
 *   1. Collect all cones (left + right) as 2D points
 *   2. Compute 2D Delaunay triangulation (using delaunator-cpp)
 *   3. Extract unique edges from the triangulation
 *   4. Greedy walk from the start position, selecting the closest valid
 *      gate (edge that crosses the track) at each step
 *   5. Return the ordered gate midpoints as the middle line
 */

#include "middleLine/middleLineComputer.hpp"
#include "external/delaunator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

// ============================================================================
// Middle Line Computation (greedy gate walk)
// ============================================================================

MiddleLineResult computeMiddleLine(
    const Track& track, const Eigen::Vector3d& startPosition, double startYaw, const MiddleLineParams& params)
{
    MiddleLineResult result;

    // 1. Collect all cones as 2D points (merge left + right, colour-blind)
    std::vector<Eigen::Vector2d> cones2D;
    std::vector<double> coneZ; // preserve Z for output

    for (const auto& lm : track.left_lane)
    {
        cones2D.push_back(Eigen::Vector2d(lm.position.x(), lm.position.y()));
        coneZ.push_back(lm.position.z());
    }
    for (const auto& lm : track.right_lane)
    {
        cones2D.push_back(Eigen::Vector2d(lm.position.x(), lm.position.y()));
        coneZ.push_back(lm.position.z());
    }

    if (cones2D.size() < 4)
        return result;

    // 2. Delaunay triangulation using delaunator-cpp
    //    delaunator expects coords as flat [x0, y0, x1, y1, ...]
    std::vector<double> coords;
    coords.reserve(cones2D.size() * 2);
    for (const auto& c : cones2D)
    {
        coords.push_back(c.x());
        coords.push_back(c.y());
    }
    delaunator::Delaunator d(coords);

    // 3. Extract unique edges from triangulation (sorted index pairs, like Python)
    std::set<std::pair<int, int>> edgesSet;
    for (std::size_t i = 0; i < d.triangles.size(); i += 3)
    {
        int v[3] = { static_cast<int>(d.triangles[i]),
                     static_cast<int>(d.triangles[i + 1]),
                     static_cast<int>(d.triangles[i + 2]) };
        std::sort(v, v + 3);
        edgesSet.insert({ v[0], v[1] });
        edgesSet.insert({ v[1], v[2] });
        edgesSet.insert({ v[0], v[2] });
    }

    // Store all Delaunay edges for debug visualization
    for (const auto& edge : edgesSet)
    {
        double z1 = coneZ[edge.first];
        double z2 = coneZ[edge.second];
        result.delaunayEdges.push_back({
            Eigen::Vector3d(cones2D[edge.first].x(), cones2D[edge.first].y(), z1),
            Eigen::Vector3d(cones2D[edge.second].x(), cones2D[edge.second].y(), z2) });
    }

    // 4. Greedy walk: from start position, repeatedly pick the closest valid gate
    Eigen::Vector2d currentPos(startPosition.x(), startPosition.y());
    Eigen::Vector2d currentDir(std::cos(startYaw), std::sin(startYaw));

    std::set<std::pair<int, int>> visitedEdges;

    for (int step = 0; step < params.maxSteps; step++)
    {
        Eigen::Vector2d bestMidpoint = Eigen::Vector2d::Zero();
        std::pair<int, int> bestEdge = { -1, -1 };
        double minDist = std::numeric_limits<double>::infinity();
        bool found = false;

        for (const auto& edge : edgesSet)
        {
            if (visitedEdges.count(edge))
                continue;

            const Eigen::Vector2d& p1 = cones2D[edge.first];
            const Eigen::Vector2d& p2 = cones2D[edge.second];

            // A. Gate geometry: edge length must be within [min, max]
            Eigen::Vector2d edgeVec = p2 - p1;
            double edgeLen = edgeVec.norm();
            if (edgeLen <= params.minGateWidth || edgeLen >= params.maxGateWidth)
                continue;

            Eigen::Vector2d midpoint = (p1 + p2) / 2.0;

            // B. Distance check
            Eigen::Vector2d vecToMid = midpoint - currentPos;
            double distToMid = vecToMid.norm();
            if (distToMid > params.searchRadius)
                continue;

            // C. Cone of vision: only consider gates ahead
            if (distToMid > 0.1)
            {
                Eigen::Vector2d dirToMid = vecToMid / distToMid;
                if (dirToMid.dot(currentDir) < 0.0)
                    continue;
            }

            // D. Orthogonality: reject edges too parallel to driving direction
            Eigen::Vector2d edgeDirNorm = edgeVec / edgeLen;
            double orthoFactor = std::abs(edgeDirNorm.dot(currentDir));
            if (orthoFactor > params.maxOrthoError)
                continue;

            // E. Select the closest valid gate
            if (distToMid < minDist)
            {
                minDist = distToMid;
                bestMidpoint = midpoint;
                bestEdge = edge;
                found = true;
            }
        }

        if (!found)
            break;

        // Record midpoint (average the Z values of the two cones)
        double z = (coneZ[bestEdge.first] + coneZ[bestEdge.second]) / 2.0;
        result.midpoints.push_back(Eigen::Vector3d(bestMidpoint.x(), bestMidpoint.y(), z));

        // Record gate
        Gate gate;
        gate.cone1 = Eigen::Vector3d(cones2D[bestEdge.first].x(), cones2D[bestEdge.first].y(), coneZ[bestEdge.first]);
        gate.cone2
            = Eigen::Vector3d(cones2D[bestEdge.second].x(), cones2D[bestEdge.second].y(), coneZ[bestEdge.second]);
        result.gates.push_back(gate);

        visitedEdges.insert(bestEdge);

        // Update direction with inertia (smooths the virtual exploration)
        Eigen::Vector2d newDir = bestMidpoint - currentPos;
        double normNew = newDir.norm();
        if (normNew > 0.1)
        {
            newDir /= normNew;
            currentDir = params.directionInertia * currentDir + (1.0 - params.directionInertia) * newDir;
            currentDir.normalize();
        }

        currentPos = bestMidpoint;
    }

    return result;
}
