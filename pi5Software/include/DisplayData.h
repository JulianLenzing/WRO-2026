#ifndef LINE_QUALITY_DISPLAYDATA_H
#define LINE_QUALITY_DISPLAYDATA_H

#include <array>
#include <vector>

#include "RGBA.h"
#include "Line.h"
#include "Vec2f.h"

#define WHITE       RGBA(1, 1, 1, 1)
#define BLACK       RGBA(0, 0, 0, 1)
#define GRAY        RGBA(0.5f, 0.5f, 0.5f, 1)

#define RED         RGBA(1, 0, 0, 1)
#define GREEN       RGBA(0, 1, 0, 1)
#define BLUE        RGBA(0, 0, 1, 1)

#define YELLOW      RGBA(1, 1, 0, 1)
#define CYAN        RGBA(0, 1, 1, 1)
#define MAGENTA     RGBA(1, 0, 1, 1)

#define ORANGE      RGBA(1, 0.5f, 0, 1)
#define PURPLE      RGBA(0.5f, 0, 0.5f, 1)
#define PINK        RGBA(1, 0.4f, 0.7f, 1)
#define BROWN       RGBA(0.4f, 0.26f, 0.13f, 1)

#define LIGHT_RED       RGBA(1, 0.6f, 0.6f, 1)
#define LIGHT_GREEN     RGBA(0.6f, 1, 0.6f, 1)
#define LIGHT_BLUE      RGBA(0.6f, 0.6f, 1, 1)
#define LIGHT_GRAY      RGBA(0.8f, 0.8f, 0.8f, 1)

#define DARK_RED        RGBA(0.5f, 0, 0, 1)
#define DARK_GREEN      RGBA(0, 0.5f, 0, 1)
#define DARK_BLUE       RGBA(0, 0, 0.5f, 1)
#define DARK_GRAY       RGBA(0.2f, 0.2f, 0.2f, 1)

using namespace std;

enum DisplayPointType {
    STANDARD_POINT,
    LANDMARK_POINT,
    UNSUEABLE_LIDAR_POINT_POINT,
    USEABLE_LIDAR_POINT_POINT,
    ESTIMATED_POSITION_POINT,
    NEW_ESTIMATED_POSITION_POINT,
    SLAM_DEBUG_POINT,
    DISPLAY_POINT_TYPE_COUNT
};
constexpr std::array<const char*, static_cast<size_t>(DisplayPointType::DISPLAY_POINT_TYPE_COUNT)> DisplayPointTypeNames = {
    "STANDARD_POINT",
    "LANDMARK_POINT",
    "UNSUEABLE_LIDAR_POINT_POINT",
    "USEABLE_LIDAR_POINT_POINT",
    "ESTIMATED_POSITION_POINT",
    "NEW_ESTIMATED_POSITION_POINT",
    "SLAM_DEBUG_POINT",
};

enum DisplayLineType {
    STANDARD_LINE = 0,
    LANDMARK_LINE,
    LIDAR_POINT_LINE,
    SLAM_DEBUG_LINE,
    DISPLAY_LINE_TYPE_COUNT
};

class DisplayLine {
public:
    Line line{};
    RGBA startColor{};
    RGBA endColor{};
    DisplayLineType type{};
    bool visible = true;
};

class DisplayPoint {
public:
    Vec2f point{};
    RGBA color{};
    DisplayPointType type{};
    bool visible = true;
};

class Visibility {
public:
    bool showPointType[DISPLAY_POINT_TYPE_COUNT] = { true };
    bool showLineType[DISPLAY_LINE_TYPE_COUNT]   = { true };

    bool showAllPoints = true;
    bool showAllLines  = true;

    Visibility() {
        for (bool& b : showPointType) b = true;
        for (bool& b : showLineType) b = true;
    }

    void setPointVisibility(const DisplayPointType type, const bool state = true) {showPointType[type] = state;}
    void setLineVisibility(const DisplayLineType type, const bool state = true) {showLineType[type] = state;}
    void setAllPoints(const bool state = true) {showAllPoints = state;}
    void setAllLines(const bool state = true) {showAllLines = state;}
};

class DisplayData {
    public:
    vector<DisplayLine> displayLines{};
    vector<DisplayPoint> displayPoints{};

    public:
    void appendPoint(const Vec2f p, const RGBA c = RGBA(), const DisplayPointType t = DisplayPointType::STANDARD_POINT) {
        displayPoints.push_back(DisplayPoint{p, c, t});
    }

    void appendPoints(const vector<Vec2f>& points, const RGBA c = RGBA(), const DisplayPointType t = DisplayPointType::STANDARD_POINT) {
        for (auto p : points) appendPoint(p, c, t);
    }

    void appendLine(const Line l, const RGBA c1 = RGBA(), const DisplayLineType t = DisplayLineType::STANDARD_LINE) {
        const RGBA c2 = c1;
        displayLines.push_back(DisplayLine(l, c1, c2, t));
    }

    void appendLine(const Line l, const RGBA c1, const RGBA c2, const DisplayLineType t = DisplayLineType::STANDARD_LINE) {
        displayLines.push_back(DisplayLine(l, c1, c2, t));
    }

    void appendLines(const vector<Line>& lines, const RGBA c = RGBA(), const DisplayLineType t = DisplayLineType::STANDARD_LINE) {
        for (const auto& l : lines) {
            appendLine(l, c, t);
        }
    }

    void removePoint() {
        displayPoints.erase(displayPoints.end());
    }

    void removeLine() {
        displayLines.erase(displayLines.end());
    }

    void clear() {
        displayLines.clear();
        displayPoints.clear();
    }

    void updateVisibility(Visibility v = Visibility()) {
        for (auto& p : displayPoints)
            p.visible = v.showAllPoints && v.showPointType[p.type];

        for (auto& l : displayLines)
            l.visible = v.showAllLines && v.showLineType[l.type];
    }

    [[nodiscard]] vector<Line> getDisplayLines() const {
        vector<Line> lines;
        for (const auto& DLine : displayLines) {
            if (DLine.visible) lines.push_back(DLine.line);
        }
        return lines;
    }

    [[nodiscard]] vector<RGBA> getDisplayLinesColors() const {
        vector<RGBA> colors;
        for (const auto& DLine : displayLines) {
            if (DLine.visible) {
                colors.push_back(DLine.startColor);
                colors.push_back(DLine.endColor);
            }
        }
        return colors;
    }

    [[nodiscard]] vector<Vec2f> getDisplayPoints() const {
        vector<Vec2f> points;
        for (const auto& DPoint : displayPoints) {
            if (DPoint.visible) points.push_back(DPoint.point);
        }
        return points;
    }

    [[nodiscard]] vector<RGBA> getDisplayPointsColors() const {
        vector<RGBA> colors;
        for (const auto& DPoint : displayPoints) {
            if (DPoint.visible) colors.push_back(DPoint.color);
        }
        return colors;
    }
};

extern DisplayData dpd;

#endif //LINE_QUALITY_DISPLAYDATA_H