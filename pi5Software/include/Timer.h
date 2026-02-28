#ifndef LINE_QUALITY_TIMER_H
#define LINE_QUALITY_TIMER_H

#include <chrono>

class Timer {
private:
    std::chrono::steady_clock::time_point startTime;
    std::chrono::microseconds duration;

public:
    // Constructor: set the timer duration in microseconds
    explicit Timer(const int& durationUs) : duration(durationUs) {
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

    // Optional: get remaining time in microseconds
    [[nodiscard]] int remainingUs() const {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
        int remaining = static_cast<int>(duration.count() - elapsed.count());
        return remaining > 0 ? remaining : 0;
    }
};

#endif //LINE_QUALITY_TIMER_H