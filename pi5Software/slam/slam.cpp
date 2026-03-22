#include "slam.h"
#include "LidarPoint.h"
#include "Landmarks.h"

#define MIN_POINT_DISTANCE 0.3f
#define MAX_POINT_DISTANCE 3.65f

using namespace std;

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

bool isPointUseable(LidarPoint& lp, float minDistance, float maxDistance, const Vec2f& xRange, const Vec2f& yRange, const vector<Line>& lms) {
    bool isPointUseable = true;
    
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
            //cout << "No intersection found" << endl;
        }
    }
    for (int i = 1; i < size; i++) {
        if (correspondingIndex[i-1] != correspondingIndex[i]) {
            isPointUseable = false;
            //printf("Inconsistent corresponding landmark indices: %d and %d on index %d\n", correspondingIndex[i-1], correspondingIndex[i], i);
        }
    }
    if (isPointUseable) lp.lmIndex = correspondingIndex[0];
    //printf("Index: %d\n", lp.lmIndex);
    return isPointUseable;
}

int getUsablePoints(LidarScan scan, const Vec2f& xRange, const Vec2f& yRange, const Landmarks& landmarks, LidarScan& useableScan) {
    int usablePointCount = 0;
    for (LidarPoint& lp : scan.scan) {
        if(isPointUseable(lp, MIN_POINT_DISTANCE, MAX_POINT_DISTANCE, xRange, yRange, landmarks.lines)) {
            useableScan.scan.push_back(lp);
            //printf("Usable Lidar Point - Angle: %f, Distance: %f, LmIndex: %d\n", lp.angle, lp.distance, lp.lmIndex);
            usablePointCount++;
        }
    }
    return usablePointCount;
}

optional<Vec2f> lidarEstimatePosition(const LidarScan& scan, const Landmarks& landmarks, const Vec2f& estimatedPosition) {
    vector<Line> parallels;
    for (auto lp : scan.scan) {
        //printf("Lidar Point - Angle: %f, Distance: %f, LmIndex: %d\n", lp.angle, lp.distance, lp.lmIndex);
        if(lp.lmIndex == -1) continue; // Skip if no corresponding landmark (should not happen if all points are useable)

        Vec2f normal = landmarks.lines[lp.lmIndex].normal();
        Vec2f normalOpposite = normal * -1.0f;

        float perpendicularDistance = lp.distance * fabs(lp.getDirection().dot(normal));
        Line parallel1 = landmarks.lines[lp.lmIndex];
        parallel1.start += normal * perpendicularDistance;
        parallel1.end += normal * perpendicularDistance;

        Line parallel2 = landmarks.lines[lp.lmIndex];
        parallel2.start += normalOpposite * perpendicularDistance;
        parallel2.end += normalOpposite * perpendicularDistance;

        Vec2f point = landmarks.lines[lp.lmIndex].closestPointOnInfinite(estimatedPosition);
        float fit1 = Vec2f::pointingTowards(point, normal, estimatedPosition);
        float fit2 = Vec2f::pointingTowards(point, normalOpposite, estimatedPosition);
        Line parallel;
        if (fit1 > fit2) parallel = parallel1;
        else parallel = parallel2;
        dpd.appendLine(parallel, GREEN, SLAM_DEBUG_LINE);
        //printf("Parallel line: start(%f, %f) end(%f, %f)\n", parallel.start.x, parallel.start.y, parallel.end.x, parallel.end.y);
        parallels.push_back(parallel);
    }
    optional<Vec2f> deltaPosition = weightedAngleAverageSegmentIntersections(parallels);
    return deltaPosition;
}
