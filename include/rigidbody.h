#pragma once

#include "vector2.h"

namespace physics {

class RigidBody {
public:
    // Position and rotation
    Vector2 position;
    float rotation;  // In radians

    // Velocity and angular velocity
    Vector2 velocity;
    float angularVelocity;

    // Forces and torque
    Vector2 force;
    float torque;

    // Physical properties
    float mass;
    float inverseMass;
    float momentOfInertia;
    float inverseInertia;
    float restitution;  // Bounciness (0-1)
    float friction;     // Friction coefficient

    // Body type
    enum class BodyType { Static, Dynamic };
    BodyType type;

    // Shape (for simplicity, we'll use circles and rectangles)
    enum class ShapeType { Circle, Rectangle };
    ShapeType shapeType;

    // Shape parameters
    float radius;      // For circles
    Vector2 size;      // For rectangles (width, height)

    // Constructor
    RigidBody(BodyType type = BodyType::Dynamic);

    // Physics methods
    void applyForce(const Vector2& f);
    void applyImpulse(const Vector2& impulse, const Vector2& contactPoint = Vector2(0, 0));
    void applyTorque(float t);
    void applyAngularImpulse(float impulse);

    // Integration (Euler method)
    void integrateVelocity(float dt);
    void integratePosition(float dt);

    // Set mass (updates inverse mass and moment of inertia)
    void setMass(float m);

    // Local to world transformation
    Vector2 toWorldPoint(const Vector2& localPoint) const;
    Vector2 toLocalPoint(const Vector2& worldPoint) const;

    // Velocity at point
    Vector2 getVelocityAtPoint(const Vector2& worldPoint) const;

    // Clear forces
    void clearForces();

    // Check if static
    bool isStatic() const { return type == BodyType::Static; }
};

}
