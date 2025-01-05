#pragma once

#include <cmath>
#include <iostream>

namespace physics {

class Vector2 {
public:
    float x, y;

    // Constructors
    Vector2();
    Vector2(float x, float y);

    // Operators
    Vector2 operator+(const Vector2& v) const;
    Vector2 operator-(const Vector2& v) const;
    Vector2 operator*(float scalar) const;
    Vector2 operator/(float scalar) const;
    Vector2& operator+=(const Vector2& v);
    Vector2& operator-=(const Vector2& v);
    Vector2& operator*=(float scalar);

    // Vector operations
    float dot(const Vector2& v) const;
    float cross(const Vector2& v) const;  // Returns scalar (z-component in 3D)
    float length() const;
    float lengthSquared() const;
    Vector2 normalized() const;
    void normalize();

    // Rotation
    Vector2 rotated(float radians) const;
    void rotate(float radians);

    // Utility
    void print() const;
    static float distance(const Vector2& a, const Vector2& b);
    static Vector2 lerp(const Vector2& a, const Vector2& b, float t);
};

}
