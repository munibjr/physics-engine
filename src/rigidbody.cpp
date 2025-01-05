#include "../include/rigidbody.h"
#include <cmath>

namespace physics {

RigidBody::RigidBody(BodyType type)
    : position(0, 0), rotation(0), velocity(0, 0), angularVelocity(0),
      force(0, 0), torque(0), mass(1.0f), inverseMass(1.0f),
      momentOfInertia(1.0f), inverseInertia(1.0f), restitution(0.5f),
      friction(0.3f), type(type), shapeType(ShapeType::Circle),
      radius(0.5f), size(1.0f, 1.0f) {
    if (type == BodyType::Static) {
        inverseMass = 0;
        inverseInertia = 0;
    }
}

void RigidBody::applyForce(const Vector2& f) {
    if (type == BodyType::Static) return;
    force += f;
}

void RigidBody::applyImpulse(const Vector2& impulse, const Vector2& contactPoint) {
    if (type == BodyType::Static) return;
    velocity += impulse * inverseMass;
    
    if (contactPoint.lengthSquared() > 0) {
        float crossProduct = contactPoint.cross(impulse);
        angularVelocity += crossProduct * inverseInertia;
    }
}

void RigidBody::applyTorque(float t) {
    if (type == BodyType::Static) return;
    torque += t;
}

void RigidBody::applyAngularImpulse(float impulse) {
    if (type == BodyType::Static) return;
    angularVelocity += impulse * inverseInertia;
}

void RigidBody::integrateVelocity(float dt) {
    if (type == BodyType::Static) return;
    
    // F = ma, so a = F/m
    Vector2 acceleration = force * inverseMass;
    velocity += acceleration * dt;
    
    // Damping (reduces velocity over time)
    velocity *= 0.99f;
    
    // Torque and angular velocity
    float angularAcceleration = torque * inverseInertia;
    angularVelocity += angularAcceleration * dt;
    angularVelocity *= 0.99f;
}

void RigidBody::integratePosition(float dt) {
    if (type == BodyType::Static) return;
    
    position += velocity * dt;
    rotation += angularVelocity * dt;
}

void RigidBody::setMass(float m) {
    mass = m;
    if (m > 0) {
        inverseMass = 1.0f / m;
        
        // Calculate moment of inertia based on shape
        if (shapeType == ShapeType::Circle) {
            momentOfInertia = 0.5f * m * radius * radius;
        } else {
            // Rectangle: I = (1/12) * m * (width^2 + height^2)
            momentOfInertia = (1.0f / 12.0f) * m * (size.x * size.x + size.y * size.y);
        }
        inverseInertia = 1.0f / momentOfInertia;
    } else {
        inverseMass = 0;
        momentOfInertia = 0;
        inverseInertia = 0;
    }
}

Vector2 RigidBody::toWorldPoint(const Vector2& localPoint) const {
    Vector2 rotated = localPoint.rotated(rotation);
    return position + rotated;
}

Vector2 RigidBody::toLocalPoint(const Vector2& worldPoint) const {
    Vector2 relative = worldPoint - position;
    return relative.rotated(-rotation);
}

Vector2 RigidBody::getVelocityAtPoint(const Vector2& worldPoint) const {
    Vector2 r = worldPoint - position;
    Vector2 tangential = Vector2(-r.y, r.x) * angularVelocity;
    return velocity + tangential;
}

void RigidBody::clearForces() {
    force = Vector2(0, 0);
    torque = 0;
}

}
