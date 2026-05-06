#pragma once

#include <optional>
#include <vector>

#include "Vec2f.h"
#include "Obstacle.h"
#include "LidarPoint.h"
#include <opencv2/opencv.hpp>

#include "DisplayData.h"

#define OBSTACLE_DETECTION_RADIUS 0.1f
#define HORIZONTAL_CAMERA_FOV 0.925023722f

class ObstacleDetection
{
public:
    ObstacleDetection()
    {
        std::vector<Obstacle> basePoints = {
            {Obstacle(Vec2f(1.0f, 0.4f), 4)},
            {Obstacle(Vec2f(1.0f, 0.6f), 1)},
            {Obstacle(Vec2f(1.5f, 0.4f), 5)},
            {Obstacle(Vec2f(1.5f, 0.6f), 2)},
            {Obstacle(Vec2f(2.0f, 0.4f), 6)},
            {Obstacle(Vec2f(2.0f, 0.6f), 3)}
        };

        Vec2f pivot(1.5f, 1.5f);
        for (const auto& p : basePoints) {
            Obstacle current = p;

            for (int i = 0; i < 4; ++i) {
                possibleObstacles.emplace_back(current);
                current.position = rotate90(current.position, pivot);
            }
        }
    }

    ~ObstacleDetection() {}

    void feedScan(LidarScan scan, Vec2f estimatedPosition)
    {
        float radiusSquared = OBSTACLE_DETECTION_RADIUS * OBSTACLE_DETECTION_RADIUS;
        for (LidarPoint point : scan.scan)
        {
            for (Obstacle& possibleObstacle : possibleObstacles)
            {
                if (isInRadiusSquared(point.point() + estimatedPosition, possibleObstacle.position, radiusSquared))
                {
                    if (possibleObstacle.count < 999999) possibleObstacle.count++; // Safety against overflow
                    dpd.appendPoint(point.point() + estimatedPosition, WHITE);
                }
            }
        }
    }

    void feedImage(cv::Mat image, std::vector<Obstacle> filteredObstacles, Vec2f position, float heading)
    {
        enum OBSTACLE_COLOR obstacleColor = OBSTACLE_COLOUR_UNKNOWN;
        filterColors(image, obstacleColor);
        if (obstacleColor != OBSTACLE_COLOUR_UNKNOWN)
        {
            Obstacle* closest = nullptr;
            float shortestSquaredDistance = 16;
            for (Obstacle& obstacle : filteredObstacles)
            {
                if (obstacle.isValid())
                {
                    Vec2f rel(obstacle.position - position);
                    float angle = acosf((rel.x * cosf(heading) + rel.y * sinf(heading)) / rel.length());
                    if (rel.lengthSquared() < shortestSquaredDistance && fabs(angle) < HORIZONTAL_CAMERA_FOV / 2.0f) // The closest obstacle infront of the robot
                    {
                        shortestSquaredDistance = rel.lengthSquared();
                        closest = &obstacle;
                    }
                }
            }
            if (!closest) return;

            for (Obstacle& ownObstacle : possibleObstacles) 
            {
                if(ownObstacle.position == closest->position)
                {
                    if (fabs(ownObstacle.colorCount) < 99999) // Overflow protection
                    {
                        if (obstacleColor == OBSTACLE_COLOUR_RED) ownObstacle.colorCount++;
                        else ownObstacle.colorCount--;
                    }
                    break;
                }
            }
        }
    }

    bool filterColors(cv::Mat display, enum OBSTACLE_COLOR& obstacleColor) {
        cv::Mat hsv;
        cv::cvtColor(display, hsv, cv::COLOR_BGR2HSV);

        // ========================
        // GREEN MASK
        // ========================
        cv::Mat greenMask;
        #ifndef SIMULATION
        cv::Scalar greenLower(40, 80, 60);
        cv::Scalar greenUpper(65, 255, 255);
        #endif
        #ifdef SIMULATION
        cv::Scalar greenLower(40, 50, 50);
        cv::Scalar greenUpper(90, 255, 255);
        #endif
        cv::inRange(hsv, greenLower, greenUpper, greenMask);

        // ========================
        // RED MASK (two ranges)
        // ========================
        cv::Mat redMask1, redMask2, redMask;

        cv::Scalar redLower1(0, 160, 110);
        cv::Scalar redUpper1(5, 255, 255);

        cv::Scalar redLower2(155, 130, 110);
        cv::Scalar redUpper2(180, 255, 255);

        cv::inRange(hsv, redLower1, redUpper1, redMask1);
        cv::inRange(hsv, redLower2, redUpper2, redMask2);

        redMask = redMask1 | redMask2;

        // ========================
        // OPTIONAL: CLEAN NOISE
        // ========================
        cv::erode(greenMask, greenMask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(greenMask, greenMask, cv::Mat(), cv::Point(-1, -1), 2);

        cv::erode(redMask, redMask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(redMask, redMask, cv::Mat(), cv::Point(-1, -1), 2);

        // ========================
        // FIND CONTOURS
        // ========================
        std::vector<std::vector<cv::Point>> greenContours, redContours;

        cv::findContours(greenMask, greenContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(redMask, redContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // ========================
        // FIND LARGEST OBJECTS
        // ========================
        double maxGreenArea = 0;
        cv::Rect bestGreenRect;

        for (const auto& cnt : greenContours) {
            double area = cv::contourArea(cnt);
            if (area > 500 && area > maxGreenArea) {  // ignore small noise
                maxGreenArea = area;
                bestGreenRect = cv::boundingRect(cnt);
            }
        }

        double maxRedArea = 0;
        cv::Rect bestRedRect;

        for (const auto& cnt : redContours) {
            double area = cv::contourArea(cnt);
            if (area > 500 && area > maxRedArea) {
                maxRedArea = area;
                bestRedRect = cv::boundingRect(cnt);
            }
        }

        // ========================
        // DRAW RESULTS
        // ========================
        if (maxGreenArea > 0) {
            cv::rectangle(display, bestGreenRect, cv::Scalar(312, 100, 100), 2);
        }

        if (maxRedArea > 0) {
            cv::rectangle(display, bestRedRect, cv::Scalar(312, 100, 100), 2);
        }

        // ========================
        // PRINT WHICH IS CLOSER
        // ========================
        if (maxGreenArea > 0 || maxRedArea > 0) {
            if (maxGreenArea > maxRedArea) {
                std::cout << "Green object is closer\n";
                obstacleColor = OBSTACLE_COLOUR_GREEN;
            } else if (maxRedArea > maxGreenArea) {
                std::cout << "Red object is closer\n";
                obstacleColor = OBSTACLE_COLOUR_RED;
            } else {
                std::cout << "Both are at similar distance\n";
                cv::imshow("Camera", display);
                cv::waitKey(1);
                return 0;
            }
        } else {
            std::cout << "No objects detected\n";
            cv::imshow("Camera", display);
            cv::waitKey(1);
            return 0;
        }

        // ========================
        // SHOW OUTPUT
        // ========================
        cv::imshow("Camera", display);
        cv::waitKey(1);
        //cv::imshow("Green Mask", greenMask);
        //cv::imshow("Red Mask", redMask);
        return 1;
    }

    void getObstacles(std::vector<Obstacle>& obstacles)
    {
        for(Obstacle o : possibleObstacles) {
            if(o.isValid()) obstacles.push_back(o);
        }        
    }

    std::vector<Obstacle> possibleObstacles;

protected:
    bool isInRadiusSquared(Vec2f positionA, Vec2f positionB, float radiusSquared)
    {
        float distanceSquared = Vec2f(positionB-positionA).lengthSquared();
        if (distanceSquared < radiusSquared) return true;
        return false;
    }

    Vec2f rotate90(const Vec2f& p, const Vec2f& center) {
        float x = p.x - center.x;
        float y = p.y - center.y;

        float rx = -y;
        float ry = x;

        return Vec2f(rx + center.x, ry + center.y);
    }
};
