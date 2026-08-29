#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

struct PathfinderRealPoint {
    float x = 0.f;
    float y = 0.f;
};

struct PathfinderRealShape {
    std::array<PathfinderRealPoint, 8> points {};
    uint8_t pointCount = 0;
    bool hazard = false;
    bool solid = false;
    float minX = 0.f;
    float maxX = 0.f;
    float minY = 0.f;
    float maxY = 0.f;
};

struct PathfinderRealGeometry {
    // Shapes are copied out of Geometry Dash's own spatial collision sections.
    // A shape can appear in several X buckets so queries only touch nearby edges.
    std::unordered_map<int, std::vector<PathfinderRealShape>> sections;
    std::size_t shapeCount = 0;
    float minX = 0.f;
    float maxX = 0.f;
};

std::shared_ptr<PathfinderRealGeometry const> getPathfinderRealGeometry();
void clearPathfinderRealGeometry();

// Returns the minimum distance from an axis-aligned player hitbox to a real GD
// collision polygon. <= 0 means the rectangle overlaps a solid/hazard edge.
float pathfinderRealGeometryClearance(
    PathfinderRealGeometry const& geometry,
    float left,
    float bottom,
    float right,
    float top
);
