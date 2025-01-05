#include "../include/vector2.h"

namespace physics {

Vector2::Vector2() : x(0), y(0) {}

Vector2::Vector2(float x, float y) : x(x), y(y) {}

Vector2 Vector2::operator+(const Vector2& v) const {
    return Vector2(x + v.x, y + v.y);
}

Vector2 Vector2::operator-(const Vector2& v) const {
    return Vector2(x - v.x, y - v.y);
}

Vector2 Vector2::operator*(float scalar) const {
    return Vector2(x * scalar, y * scalar);
}

Vector2 Vector2::operator/(float scalar) const {
    if (scalar == 0) return Vector2(0, 0);
    return Vector2(x / scalar, y / scalar);
}

Vector2& Vector2::operator+=(const Vector2& v) {
    x += v.x;
    y += v.y;
    return *this;
}

Vector2& Vector2::operator-=(const Vector2& v) {
    x -= v.x;
    y -= v.y;
    return *this;
}

Vector2& Vector2::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
}

float Vector2::dot(const Vector2& v) const {
    return x * v.x + y * v.y;
}

float Vector2::cross(const Vector2& v) const {
    return x * v.y - y * v.x;
}

float Vector2::length() const {
    return std::sqrt(x * x + y * y);
}

float Vector2::lengthSquared() const {
    return x * x + y * y;
}

Vector2 Vector2::normalized() const {
    float len = length();
    if (len == 0) return Vector2(0, 0);
    return Vector2(x / len, y / len);
}

void Vector2::normalize() {
    float len = length();
    if (len == 0) return;
    x /= len;
    y /= len;
}

Vector2 Vector2::rotated(float radians) const {
    float cos_a = std::cos(radians);
    float sin_a = std::sin(radians);
    return Vector2(x * cos_a - y * sin_a, x * sin_a + y * cos_a);
}

void Vector2::rotate(float radians) {
    float cos_a = std::cos(radians);
    float sin_a = std::sin(radians);
    float new_x = x * cos_a - y * sin_a;
    float new_y = x * sin_a + y * cos_a;
    x = new_x;
    y = new_y;
}

void Vector2::print() const {
    std::cout << "Vector2(" << x << ", " << y << ")" << std::endl;
}

float Vector2::distance(const Vector2& a, const Vector2& b) {
    return (a - b).length();
}

Vector2 Vector2::lerp(const Vector2& a, const Vector2& b, float t) {
    return a + (b - a) * t;
}

}
