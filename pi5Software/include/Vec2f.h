#ifndef LINE_QUALITY_VECTORS_H
#define LINE_QUALITY_VECTORS_H

#include <cmath>

struct Vec2f {
    float x, y;

    explicit Vec2f(float x = 0, float y = 0) : x(x), y(y) {}

    [[nodiscard]] float dot(const Vec2f& other) const {return x * other.x + y * other.y;}
    [[nodiscard]] float length() const { return sqrtf(x * x + y * y);}
    [[nodiscard]] float lengthSquared() const { return x * x + y * y; }

    void normalize() {
        const float len = std::sqrt(x * x + y * y);
        if (len == 0.0f) return;
        x /= len;
        y /= len;
    }

    [[nodiscard]] Vec2f normalized() const {
        float len = std::sqrt(x * x + y * y);
        if (len == 0.0f) return Vec2f();
        return Vec2f(x / len, y / len);
    }


    static float pointingTowards(const Vec2f& origin, const Vec2f& vector, const Vec2f& point) {
        const Vec2f vNorm = vector.normalized();
        const Vec2f dir = (point - origin).normalized();

        return vNorm.dot(dir); // [-1, 1]
    }

    Vec2f operator+(const Vec2f& other) const { return Vec2f(x + other.x, y + other.y); }
    Vec2f operator-(const Vec2f& other) const { return Vec2f(x - other.x, y - other.y); }
    Vec2f operator*(float scalar) const { return Vec2f(x * scalar, y * scalar); }
    Vec2f operator/(float scalar) const { return Vec2f(x / scalar, y / scalar); }

    Vec2f& operator+=(const Vec2f& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2f& operator-=(const Vec2f& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    bool operator==(const Vec2f& other) const { return x == other.x && y == other.y; }
    bool operator!=(const Vec2f& other) const { return !(*this == other); }
};

#endif //LINE_QUALITY_VECTORS_H