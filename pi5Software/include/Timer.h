#pragma once

#include <chrono>

class TimerMillis {
private:
    std::chrono::steady_clock::time_point startTime;
    std::chrono::milliseconds duration;

public:
    explicit TimerMillis(const int& durationMs) : duration(durationMs) {
        startTime = std::chrono::steady_clock::now();
    }

    void reset() {
        startTime = std::chrono::steady_clock::now();
    }

    void setDuration(int durationMs) {
        duration = std::chrono::milliseconds(durationMs);
    }

    [[nodiscard]] bool isExpired() const {
        auto now = std::chrono::steady_clock::now();
        return now - startTime >= duration;
    }

    [[nodiscard]] int remainingMs() const {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime);
        int remaining = static_cast<int>(duration.count() - elapsed.count());
        return remaining > 0 ? remaining : 0;
    }

    [[nodiscard]] std::chrono::milliseconds passedMs() const {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime);
        return elapsed;
    }
};

class TimerMicros {
private:
    std::chrono::steady_clock::time_point startTime;
    std::chrono::microseconds duration;

public:
    // Constructor: set the timer duration in microseconds
    explicit TimerMicros(const int& durationUs) : duration(durationUs) {
        startTime = std::chrono::steady_clock::now();
    }

    // Reset the timer to start counting from now
    void reset() {
        startTime = std::chrono::steady_clock::now();
    }

    void setDuration(int durationUs) {
        duration = std::chrono::microseconds(durationUs);
    }

    // Check if the timer has expired
    [[nodiscard]] bool isExpired() const {
        auto now = std::chrono::steady_clock::now();
        return now - startTime >= duration;
    }

    [[nodiscard]] int remainingUs() const {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
        int remaining = static_cast<int>(duration.count() - elapsed.count());
        return remaining > 0 ? remaining : 0;
    }
};