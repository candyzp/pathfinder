#include "real_geometry.hpp"

#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_set>

using namespace geode::prelude;

namespace {

std::mutex g_geometryMutex;
std::shared_ptr<PathfinderRealGeometry const> g_geometry;
constexpr float kBucketWidth = 100.f;

float cross(PathfinderRealPoint a, PathfinderRealPoint b, PathfinderRealPoint c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

float pointSegmentDistance(
    PathfinderRealPoint p,
    PathfinderRealPoint a,
    PathfinderRealPoint b
) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float len2 = dx * dx + dy * dy;
    if (len2 <= 1e-8f) {
        float px = p.x - a.x;
        float py = p.y - a.y;
        return std::sqrt(px * px + py * py);
    }
    float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
    t = std::clamp(t, 0.f, 1.f);
    float qx = a.x + t * dx;
    float qy = a.y + t * dy;
    float px = p.x - qx;
    float py = p.y - qy;
    return std::sqrt(px * px + py * py);
}

bool segmentsIntersect(
    PathfinderRealPoint a,
    PathfinderRealPoint b,
    PathfinderRealPoint c,
    PathfinderRealPoint d
) {
    float c1 = cross(a, b, c);
    float c2 = cross(a, b, d);
    float c3 = cross(c, d, a);
    float c4 = cross(c, d, b);
    constexpr float eps = 1e-5f;

    if (((c1 > eps && c2 < -eps) || (c1 < -eps && c2 > eps)) &&
        ((c3 > eps && c4 < -eps) || (c3 < -eps && c4 > eps))) {
        return true;
    }

    auto onSegment = [](PathfinderRealPoint p, PathfinderRealPoint q, PathfinderRealPoint r) {
        constexpr float e = 1e-5f;
        return q.x >= std::min(p.x, r.x) - e && q.x <= std::max(p.x, r.x) + e &&
               q.y >= std::min(p.y, r.y) - e && q.y <= std::max(p.y, r.y) + e;
    };

    if (std::abs(c1) <= eps && onSegment(a, c, b)) return true;
    if (std::abs(c2) <= eps && onSegment(a, d, b)) return true;
    if (std::abs(c3) <= eps && onSegment(c, a, d)) return true;
    if (std::abs(c4) <= eps && onSegment(c, b, d)) return true;
    return false;
}

bool pointInConvexPolygon(
    PathfinderRealPoint p,
    PathfinderRealShape const& shape
) {
    if (shape.pointCount < 3)
        return false;

    float sign = 0.f;
    for (uint8_t i = 0; i < shape.pointCount; ++i) {
        auto a = shape.points[i];
        auto b = shape.points[(i + 1) % shape.pointCount];
        float value = cross(a, b, p);
        if (std::abs(value) <= 1e-5f)
            continue;
        if (sign == 0.f)
            sign = value;
        else if ((sign > 0.f) != (value > 0.f))
            return false;
    }
    return true;
}

float shapeRectClearance(
    PathfinderRealShape const& shape,
    float left,
    float bottom,
    float right,
    float top
) {
    if (right < shape.minX) return shape.minX - right;
    if (left > shape.maxX) return left - shape.maxX;
    if (top < shape.minY) return shape.minY - top;
    if (bottom > shape.maxY) return bottom - shape.maxY;

    std::array<PathfinderRealPoint, 4> rect {{
        {left, bottom},
        {right, bottom},
        {right, top},
        {left, top}
    }};

    for (auto const& p : rect) {
        if (pointInConvexPolygon(p, shape))
            return 0.f;
    }

    for (uint8_t i = 0; i < shape.pointCount; ++i) {
        auto const& p = shape.points[i];
        if (p.x >= left && p.x <= right && p.y >= bottom && p.y <= top)
            return 0.f;
    }

    float best = std::numeric_limits<float>::infinity();
    for (uint8_t i = 0; i < shape.pointCount; ++i) {
        auto a = shape.points[i];
        auto b = shape.points[(i + 1) % shape.pointCount];

        for (size_t r = 0; r < rect.size(); ++r) {
            auto c = rect[r];
            auto d = rect[(r + 1) % rect.size()];
            if (segmentsIntersect(a, b, c, d))
                return 0.f;

            best = std::min(best, pointSegmentDistance(a, c, d));
            best = std::min(best, pointSegmentDistance(b, c, d));
            best = std::min(best, pointSegmentDistance(c, a, b));
            best = std::min(best, pointSegmentDistance(d, a, b));
        }
    }
    return best;
}

void finishBounds(PathfinderRealShape& shape) {
    if (shape.pointCount == 0)
        return;
    shape.minX = shape.maxX = shape.points[0].x;
    shape.minY = shape.maxY = shape.points[0].y;
    for (uint8_t i = 1; i < shape.pointCount; ++i) {
        shape.minX = std::min(shape.minX, shape.points[i].x);
        shape.maxX = std::max(shape.maxX, shape.points[i].x);
        shape.minY = std::min(shape.minY, shape.points[i].y);
        shape.maxY = std::max(shape.maxY, shape.points[i].y);
    }
}

CCRect stableObjectRect(GameObject* obj) {
    if (!obj)
        return {};
    if (!obj->m_isObjectRectDirty)
        return obj->m_objectRect;

    bool dirty = obj->m_isObjectRectDirty;
    bool boxOffset = obj->m_boxOffsetCalculated;
    auto rect = obj->getObjectRect();
    obj->m_isObjectRectDirty = dirty;
    obj->m_boxOffsetCalculated = boxOffset;
    return rect;
}

bool collisionShapeType(GameObject* obj, bool& hazard, bool& solid) {
    if (!obj)
        return false;

    hazard = false;
    solid = false;

    switch (obj->m_objectType) {
        case GameObjectType::Hazard:
        case GameObjectType::AnimatedHazard:
            hazard = true;
            return true;
        case GameObjectType::Solid:
        case GameObjectType::Slope:
            if (!obj->m_isPassable) {
                solid = true;
                return true;
            }
            return false;
        default:
            return false;
    }
}

PathfinderRealShape shapeFromObject(GameObject* obj, bool hazard, bool solid) {
    PathfinderRealShape shape;
    shape.hazard = hazard;
    shape.solid = solid;

    if (obj->m_objectRadius > 0.f) {
        float radius = obj->m_objectRadius * std::max(std::abs(obj->m_scaleX), std::abs(obj->m_scaleY));
        constexpr uint8_t count = 8;
        shape.pointCount = count;
        constexpr float pi = 3.14159265358979323846f;
        for (uint8_t i = 0; i < count; ++i) {
            float a = (2.f * pi * static_cast<float>(i)) / static_cast<float>(count);
            shape.points[i] = {
                static_cast<float>(obj->m_positionX) + std::cos(a) * radius,
                static_cast<float>(obj->m_positionY) + std::sin(a) * radius
            };
        }
        finishBounds(shape);
        return shape;
    }

    if (obj->m_orientedBox) {
        shape.pointCount = 4;
        for (uint8_t i = 0; i < 4; ++i) {
            auto p = obj->m_orientedBox->m_corners[i];
            shape.points[i] = {p.x, p.y};
        }
        finishBounds(shape);
        return shape;
    }

    auto r = stableObjectRect(obj);
    if (obj->m_objectType == GameObjectType::Slope) {
        PathfinderRealPoint br {r.getMaxX(), r.getMinY()};
        PathfinderRealPoint tl {r.getMinX(), r.getMaxY()};
        PathfinderRealPoint bl {r.getMinX(), r.getMinY()};
        PathfinderRealPoint tr {r.getMaxX(), r.getMaxY()};

        shape.pointCount = 3;
        switch (obj->m_slopeDirection) {
            case 0:
            case 7:
                shape.points[0] = br;
                shape.points[1] = tr;
                shape.points[2] = bl;
                break;
            case 3:
            case 6:
                shape.points[0] = tr;
                shape.points[1] = tl;
                shape.points[2] = bl;
                break;
            case 1:
            case 5:
                shape.points[0] = br;
                shape.points[1] = tr;
                shape.points[2] = tl;
                break;
            default:
                shape.points[0] = br;
                shape.points[1] = tl;
                shape.points[2] = bl;
                break;
        }
        finishBounds(shape);
        return shape;
    }

    shape.pointCount = 4;
    shape.points[0] = {r.getMinX(), r.getMinY()};
    shape.points[1] = {r.getMaxX(), r.getMinY()};
    shape.points[2] = {r.getMaxX(), r.getMaxY()};
    shape.points[3] = {r.getMinX(), r.getMaxY()};
    finishBounds(shape);
    return shape;
}

std::shared_ptr<PathfinderRealGeometry const> captureGeometry(PlayLayer* game) {
    if (!game)
        return {};

    auto snapshot = std::make_shared<PathfinderRealGeometry>();
    std::unordered_set<GameObject*> seen;

    int xCount = static_cast<int>(game->m_sections.size());
    for (int i = 0; i < xCount; ++i) {
        auto column = game->m_sections[i];
        if (!column)
            continue;

        int yCount = static_cast<int>(column->size());
        for (int j = 0; j < yCount; ++j) {
            auto cell = column->at(j);
            if (!cell)
                continue;

            if (i >= static_cast<int>(game->m_sectionSizes.size()) || !game->m_sectionSizes[i])
                continue;
            auto sizes = game->m_sectionSizes[i];
            if (j >= static_cast<int>(sizes->size()))
                continue;

            int objectCount = sizes->at(j);
            for (int k = 0; k < objectCount; ++k) {
                auto* obj = cell->at(k);
                if (!obj || !seen.insert(obj).second)
                    continue;
                if (obj == game->m_player1CollisionBlock ||
                    obj == game->m_player2CollisionBlock ||
                    obj == game->m_anticheatSpike)
                    continue;
                if (obj->m_isDecoration || obj->m_isDecoration2 || obj->m_unk3ee)
                    continue;

                bool hazard = false;
                bool solid = false;
                if (!collisionShapeType(obj, hazard, solid))
                    continue;

                auto shape = shapeFromObject(obj, hazard, solid);
                if (shape.pointCount < 3 || !std::isfinite(shape.minX) || !std::isfinite(shape.maxX))
                    continue;

                int firstBucket = static_cast<int>(std::floor(shape.minX / kBucketWidth));
                int lastBucket = static_cast<int>(std::floor(shape.maxX / kBucketWidth));
                firstBucket = std::max(firstBucket, -200000);
                lastBucket = std::min(lastBucket, 200000);
                for (int bucket = firstBucket; bucket <= lastBucket; ++bucket)
                    snapshot->sections[bucket].push_back(shape);

                if (snapshot->shapeCount == 0) {
                    snapshot->minX = shape.minX;
                    snapshot->maxX = shape.maxX;
                } else {
                    snapshot->minX = std::min(snapshot->minX, shape.minX);
                    snapshot->maxX = std::max(snapshot->maxX, shape.maxX);
                }
                ++snapshot->shapeCount;
            }
        }
    }

    if (snapshot->shapeCount == 0)
        return {};
    return snapshot;
}

void publishGeometry(PlayLayer* game) {
    auto captured = captureGeometry(game);
    std::lock_guard lock(g_geometryMutex);
    g_geometry = std::move(captured);
    if (g_geometry) {
        log::info(
            "Pathfinder real-geometry scanner captured {} GD collision shapes from X {:.1f} to {:.1f}",
            g_geometry->shapeCount,
            g_geometry->minX,
            g_geometry->maxX
        );
    }
}

} // namespace

std::shared_ptr<PathfinderRealGeometry const> getPathfinderRealGeometry() {
    std::lock_guard lock(g_geometryMutex);
    return g_geometry;
}

void clearPathfinderRealGeometry() {
    std::lock_guard lock(g_geometryMutex);
    g_geometry.reset();
}

float pathfinderRealGeometryClearance(
    PathfinderRealGeometry const& geometry,
    float left,
    float bottom,
    float right,
    float top
) {
    int firstBucket = static_cast<int>(std::floor(left / kBucketWidth)) - 1;
    int lastBucket = static_cast<int>(std::floor(right / kBucketWidth)) + 1;
    float best = std::numeric_limits<float>::infinity();

    for (int bucket = firstBucket; bucket <= lastBucket; ++bucket) {
        auto it = geometry.sections.find(bucket);
        if (it == geometry.sections.end())
            continue;
        for (auto const& shape : it->second) {
            float clearance = shapeRectClearance(shape, left, bottom, right, top);
            best = std::min(best, clearance);
            if (best <= 0.f)
                return 0.f;
        }
    }
    return best;
}

struct PathfinderRealGeometryPlayLayer : geode::Modify<PathfinderRealGeometryPlayLayer, PlayLayer> {
    void createObjectsFromSetupFinished() {
        PlayLayer::createObjectsFromSetupFinished();
        publishGeometry(this);
    }

    void onQuit() {
        clearPathfinderRealGeometry();
        PlayLayer::onQuit();
    }
};
