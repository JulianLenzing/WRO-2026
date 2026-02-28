#ifndef LINE_QUALITY_LINE_H
#define LINE_QUALITY_LINE_H

#include <algorithm>
#include <optional>  // for std::optional
#include <cmath>

#include "Vec2f.h"

inline bool inRange(float t) { return t >= 0.0f && t <= 1.0f; }

class Line {
public:
    Vec2f start;
    Vec2f end;

    Line() = default;
    Line(const Vec2f& s, const Vec2f& e) : start(s), end(e) {}

    bool operator==(const Line& other) const { return start == other.start && end == other.end; }
    bool operator!=(const Line& other) const { return !(*this == other); }

    [[nodiscard]] Vec2f direction() const { return end - start; }

    // ---- Intersection of INFINITE lines ----
    static std::optional<Vec2f> intersectionInfinite(const Line& a, const Line& b) {
        float x1 = a.start.x, y1 = a.start.y;
        float x2 = a.end.x,   y2 = a.end.y;
        float x3 = b.start.x, y3 = b.start.y;
        float x4 = b.end.x,   y4 = b.end.y;

        float denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);
        if (std::fabs(denom) < 1e-6f)
            return std::nullopt; // Parallel or coincident

        float px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denom;
        float py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denom;

        return Vec2f(px, py);
    }

    // ---- Intersection of finite SEGMENTS ----
    static std::optional<Vec2f> intersectionSegment(const Line& a, const Line& b) {
        float x1 = a.start.x, y1 = a.start.y;
        float x2 = a.end.x,   y2 = a.end.y;
        float x3 = b.start.x, y3 = b.start.y;
        float x4 = b.end.x,   y4 = b.end.y;

        float denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);
        if (std::fabs(denom) < 1e-6f)
            return std::nullopt; // Parallel or coincident

        float t = ((x1 - x3)*(y3 - y4) - (y1 - y3)*(x3 - x4)) / denom;
        float u = ((x1 - x3)*(y1 - y2) - (y1 - y3)*(x1 - x2)) / denom;

        if (!inRange(t) || !inRange(u))
            return std::nullopt; // Intersection outside segments

        float px = x1 + t * (x2 - x1);
        float py = y1 + t * (y2 - y1);

        return Vec2f(px, py);
    }

    [[nodiscard]] Vec2f normal() const {
        // direction vector of THIS line
        Vec2f dir = end - start;

        // perpendicular: (x, y) → (−y, x)
        Vec2f n(-dir.y, dir.x);

        // normalize
        n.normalize();
        return n;
    }

    [[nodiscard]] Vec2f closestPointOnSegment(const Vec2f& P) const {
        Vec2f AB = end - start;
        Vec2f AP = P - start;

        float t = AP.dot(AB) / AB.lengthSquared();
        t = std::clamp(t, 0.0f, 1.0f);  // clamp to segment

        return start + AB * t;
    }

    [[nodiscard]] Vec2f closestPointOnInfinite(const Vec2f& P) const {
        Vec2f AB = end - start;   // direction of the line
        Vec2f AP = P - start;     // vector from line start to P

        float t = AP.dot(AB) / AB.lengthSquared();
        // No clamping here — infinite line

        return start + AB * t;    // point along infinite line
    }
};

#endif //LINE_QUALITY_LINE_H