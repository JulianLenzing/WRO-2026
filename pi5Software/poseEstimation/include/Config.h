#ifndef LINE_QUALITY_CONFIG_H
#define LINE_QUALITY_CONFIG_H

#include <iostream>
#include <ostream>
#include <string>
#include <unordered_map>

int square(int x);

class Config {
public:
    // predefined config variables
    int rayCount = 50;
    float angleNoiseStdDeg = 2.0f;
    float distanceNoiseStd = 0.2f;

    explicit Config()= default;

    explicit Config(const std::unordered_map<std::string, std::string>& entries) {
        applyConfig(entries);
    }

    [[nodiscard]] std::unordered_map<std::string, std::string> toMap() const {
        return {
            { "RayCount",         std::to_string(rayCount) },
            { "AngleNoiseStdDeg", std::to_string(angleNoiseStdDeg) },
            { "DistanceNoiseStd", std::to_string(distanceNoiseStd) }
        };
    }

    // Fills the members using the map from ConfigReader
    void applyConfig(const std::unordered_map<std::string, std::string>& entries) {
        for (const auto& [tag, value] : entries) {
            {
                /* Examples for different types
                if (tag == "ServerIP") {
                    ServerIP = value;
                }
                else if (tag == "Port") {
                    Port = std::stoi(value);
                }
                else if (tag == "UseSSL") {
                    UseSSL = (value == "true" || value == "1");
                }
                else if (tag == "Timeout") {
                    Timeout = std::stod(value);
                }
                else {
                    std::cerr << "Unknown tag: " << tag << std::endl;
                }
                */
            }
            if (tag == "RayCount") rayCount = std::stoi(value);
            else if (tag == "AngleNoiseStdDeg") angleNoiseStdDeg = std::stof(value);
            else if (tag == "DistanceNoiseStd") distanceNoiseStd = std::stof(value);
            else std::cerr << "Unknown tag: " << tag << std::endl;
        }
    }
};

#endif //LINE_QUALITY_CONFIG_H