#include "slam.h"

using namespace std;

void generateLandmarks(vector<Line>& lms) {
    lms.emplace_back(Vec2f(0,0), Vec2f(0, 3));
    lms.emplace_back(Vec2f(0, 3), Vec2f(3, 3));
    lms.emplace_back(Vec2f(3, 3), Vec2f(3, 0));
    lms.emplace_back(Vec2f(3, 0), Vec2f(0, 0));
    lms.emplace_back(Vec2f(1, 1), Vec2f(1, 2));
    lms.emplace_back(Vec2f(1, 2), Vec2f(2, 2));
    lms.emplace_back(Vec2f(2, 2), Vec2f(2, 1));
    lms.emplace_back(Vec2f(2, 1), Vec2f(1, 1));
}

static float angleWeight(const Line& a, const Line& b)
{
    Vec2f A = a.direction();
    Vec2f B = b.direction();

    float lenA = A.length();
    float lenB = B.length();

    if (lenA == 0.0f || lenB == 0.0f)
        return 0.0f;

    // 2D cross product magnitude (scalar)
    float cross = A.x * B.y - A.y * B.x;

    // normalized → |sin θ|
    return std::fabs(cross) / (lenA * lenB);
}

std::optional<Vec2f> weightedAngleAverageSegmentIntersections(const std::vector<Line>& lines)
{
    double totalWeight = 0.0;
    Vec2f weightedSum(0, 0);

    const size_t n = lines.size();

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {

            auto p = Line::intersectionInfinite(lines[i], lines[j]);
            if (!p.has_value())
                continue;

            float w = angleWeight(lines[i], lines[j]);
            if (w <= 1e-8f)
                continue;

            weightedSum.x += p->x * w;
            weightedSum.y += p->y * w;
            totalWeight += w;
        }
    }

    if (totalWeight == 0.0)
        return std::nullopt;

    float inv = 1.0f / static_cast<float>(totalWeight);
    return Vec2f(weightedSum.x * inv, weightedSum.y * inv);
}

LidarPoint vec2fToLidarPoint(const Vec2f& point) {
    float distance = std::sqrt(point.x * point.x + point.y * point.y);
    float angle = std::atan2(point.y, point.x); // atan2 returns radians

    return {angle, distance};
}

void generateTestPoints(
    vector<LidarPoint>& lidarPoints,
    const Vec2f& pos,
    const vector<Line>& lms, const float& angleNoiseStdDeg, const float& distanceNoiseStd, const int& rayCount)
{
    const float maxRayDistance   = 1000.0f;   // length of ray

    //static std::default_random_engine rng(std::random_device{}());
    static std::default_random_engine rng(5555);
    std::normal_distribution<float> angleNoiseRad(0.0f, angleNoiseStdDeg * (M_PI / 180.0f));
    std::normal_distribution<float> distanceNoise(0.0f, distanceNoiseStd);

    for (int i = 0; i < rayCount; i++) {

        float baseAngle = float(i) * (2.0f * M_PI / rayCount);
        float noisyAngle = baseAngle + angleNoiseRad(rng);

        // Ray starting at pos, going outwards
        Line ray(pos, Vec2f(pos.x + cosf(noisyAngle) * maxRayDistance,
                            pos.y + sinf(noisyAngle) * maxRayDistance));

        vector<intersectionIndexPair> intersections;

        // Check all map segments
        for (int j = 0; j < lms.size(); j++) {
            optional<Vec2f> p = Line::intersectionSegment(ray, lms[j]);
            if (p.has_value()) {
                intersections.push_back(intersectionIndexPair(j, p.value()));
            }
        }

        if (!intersections.empty()) {
            // Find closest intersection:
            float lowestDistance = (intersections[0].point - ray.start).lengthSquared();
            intersectionIndexPair closest = intersections[0];

            for (int k = 1; k < intersections.size(); k++) {
                float d = (intersections[k].point - ray.start).lengthSquared();
                if (d < lowestDistance) {
                    lowestDistance = d;
                    closest = intersections[k];
                }
            }

            // Convert hit point to relative vector
            Vec2f hitVec = closest.point - pos;

            // Convert to lidar polar for noise application
            LidarPoint lp = vec2fToLidarPoint(hitVec);

            // Add distance noise
            lp.distance += distanceNoise(rng);
            if (lp.distance < 0.15f) lp.distance = 0.15f;

            // Add angle noise
            lp.angle += angleNoiseRad(rng);

            lidarPoints.push_back(lp);
        }
    }
}

bool isPointUseable(LidarPoint lp, float minDistance, float maxDistance, const Vec2f& xRange, const Vec2f& yRange, const vector<Line>& lms, size_t& landmark) {
    bool isPointUseable = true;
    landmark = -1;
    
    // Minimum and maximum distance check
    if(lp.distance <= minDistance || lp.distance >= maxDistance) return false;
    
    // Direction check
    Vec2f dir = lp.getDirection();
    constexpr int size = 4;
    Vec2f maxPos[size];
    maxPos[0] = Vec2f(xRange.x,yRange.x);
    maxPos[1] = Vec2f(xRange.x,yRange.y);
    maxPos[2] = Vec2f(xRange.y,yRange.y);
    maxPos[3] = Vec2f(xRange.y,yRange.x);

    int correspondingIndex[size];
    for (int i = 0; i < sizeof(maxPos) / sizeof(Vec2f); i++) {
        Line line(maxPos[i], Vec2f(maxPos[i].x + dir.x * 1000, maxPos[i].y + dir.y * 1000));
        //displayLines.push_back(line);
        vector<intersectionIndexPair> intersections;
        for (int j = 0; j < lms.size(); j++) {
            optional<Vec2f> p = Line::intersectionSegment(line, lms[j]);
            if (p) {
                intersections.push_back(intersectionIndexPair(j, p.value()));
            }
        }
        if (!intersections.empty()) {
            float lowestDistance = (intersections[0].point - line.start).lengthSquared();
            intersectionIndexPair closestIntersection = intersections[0];
            for (int i = 1; i < intersections.size(); i++) {
                float distance = (intersections[i].point - line.start).lengthSquared();
                if (distance < lowestDistance) {
                    lowestDistance = distance;
                    closestIntersection = intersections[i];
                }
            }
            correspondingIndex[i] = closestIntersection.index;
            //cout << "Point on Landmark: " << closestIntersection.index << endl;
        }
        else {
            correspondingIndex[i] = -1;
            isPointUseable = false; // If a line has no intersection this point is not useable
            //cout << "Function - isPointUseable: No intersection found" << endl;
        }
    }
    for (int i = 1; i < size; i++) {
        if (correspondingIndex[i-1] != correspondingIndex[i]) {
            isPointUseable = false;
        }
    }
    if (isPointUseable) landmark = correspondingIndex[0];
    return isPointUseable;
}

optional<Vec2f> processLidarPoints(const vector<LidarPoint>& lidarPoints, const vector<Line>& lms, Vec2f estimatedPosition, Vec2f xRange, Vec2f yRange) {
    vector<Line> parallels;
    for (auto lp : lidarPoints) {
        Vec2f tmp(estimatedPosition + lp.getDirection() * lp.distance);
        size_t lmIndex = -1;
        bool useable = isPointUseable(lp, 0.3, 3.65, xRange, yRange, lms, lmIndex);
        if (useable) {
            dpd.appendPoint(lp.point() + estimatedPosition, BLUE, USEABLE_LIDAR_POINT_POINT);
            //printf("%f, %f\n", lp.point().x, lp.point().y);
            Vec2f normal = lms[lmIndex].normal();
            Vec2f normalOpposite = normal * -1.0f;

            float perpendicularDistance = lp.distance * fabs(lp.getDirection().dot(normal));
            Line parallel1 = lms[lmIndex];
            parallel1.start += normal * perpendicularDistance;
            parallel1.end += normal * perpendicularDistance;

            Line parallel2 = lms[lmIndex];
            parallel2.start += normalOpposite * perpendicularDistance;
            parallel2.end += normalOpposite * perpendicularDistance;

            Vec2f point = lms[lmIndex].closestPointOnInfinite(estimatedPosition);
            float fit1 = Vec2f::pointingTowards(point, normal, estimatedPosition);
            float fit2 = Vec2f::pointingTowards(point, normalOpposite, estimatedPosition);
            Line parallel;
            if (fit1 > fit2) parallel = parallel1;
            else parallel = parallel2;
            dpd.appendLine(parallel, GREEN, SLAM_DEBUG_LINE);
            parallels.push_back(parallel);
        }
        else dpd.appendPoint(lp.point() + estimatedPosition, GRAY, UNSUEABLE_LIDAR_POINT_POINT);
    }
    optional<Vec2f> deltaPosition = weightedAngleAverageSegmentIntersections(parallels);
    return deltaPosition;
}
