#include "../include/collision.h"
#include "../include/rigidbody.h"
#include <cmath>
#include <algorithm>

namespace physics {

bool Collision::checkAABB(const RigidBody* a, const RigidBody* b) {
    float aLeft = a->position.x - a->size.x / 2;
    float aRight = a->position.x + a->size.x / 2;
    float aTop = a->position.y + a->size.y / 2;
    float aBottom = a->position.y - a->size.y / 2;

    float bLeft = b->position.x - b->size.x / 2;
    float bRight = b->position.x + b->size.x / 2;
    float bTop = b->position.y + b->size.y / 2;
    float bBottom = b->position.y - b->size.y / 2;

    return !(aRight < bLeft || aLeft > bRight || aTop < bBottom || aBottom > bTop);
}

bool Collision::checkCircle(const RigidBody* a, const RigidBody* b, CollisionManifold& manifold) {
    if (a->shapeType != RigidBody::ShapeType::Circle || b->shapeType != RigidBody::ShapeType::Circle) {
        return false;
    }

    Vector2 delta = b->position - a->position;
    float dist = delta.length();
    float minDist = a->radius + b->radius;

    if (dist >= minDist) {
        return false;
    }

    manifold.bodyA = (RigidBody*)a;
    manifold.bodyB = (RigidBody*)b;
    manifold.penetration = minDist - dist;
    
    if (dist > 0.0001f) {
        manifold.normal = delta.normalized();
    } else {
        manifold.normal = Vector2(1, 0);
    }

    manifold.contactPoint1 = a->position + manifold.normal * a->radius;
    manifold.contactCount = 1;
    manifold.restitution = std::min(a->restitution, b->restitution);

    return true;
}

bool Collision::checkAABBCircle(const RigidBody* aabb, const RigidBody* circle, CollisionManifold& manifold) {
    // Find closest point on AABB to circle center
    Vector2 closest = circle->position;
    
    float aabbLeft = aabb->position.x - aabb->size.x / 2;
    float aabbRight = aabb->position.x + aabb->size.x / 2;
    float aabbTop = aabb->position.y + aabb->size.y / 2;
    float aabbBottom = aabb->position.y - aabb->size.y / 2;

    closest.x = std::max(aabbLeft, std::min(closest.x, aabbRight));
    closest.y = std::max(aabbBottom, std::min(closest.y, aabbTop));

    Vector2 delta = circle->position - closest;
    float dist = delta.length();

    if (dist >= circle->radius) {
        return false;
    }

    manifold.bodyA = (RigidBody*)aabb;
    manifold.bodyB = (RigidBody*)circle;
    manifold.penetration = circle->radius - dist;
    
    if (dist > 0.0001f) {
        manifold.normal = delta.normalized();
    } else {
        manifold.normal = Vector2(1, 0);
    }

    manifold.contactPoint1 = closest;
    manifold.contactCount = 1;
    manifold.restitution = std::min(aabb->restitution, circle->restitution);

    return true;
}

bool Collision::checkSAT(const RigidBody* a, const RigidBody* b, CollisionManifold& manifold) {
    // Simplified SAT for axis-aligned and rotated rectangles
    // For now, fall back to AABB check if both are axis-aligned
    if (std::abs(a->rotation) < 0.0001f && std::abs(b->rotation) < 0.0001f) {
        if (checkAABB(a, b)) {
            // Create a simple manifold
            manifold.bodyA = (RigidBody*)a;
            manifold.bodyB = (RigidBody*)b;
            manifold.normal = (b->position - a->position).normalized();
            manifold.penetration = 0.1f;  // Simplified
            manifold.contactCount = 1;
            manifold.restitution = std::min(a->restitution, b->restitution);
            return true;
        }
    }
    return false;
}

bool Collision::checkCollision(RigidBody* a, RigidBody* b, CollisionManifold& manifold) {
    if (a->shapeType == RigidBody::ShapeType::Circle && b->shapeType == RigidBody::ShapeType::Circle) {
        return checkCircle(a, b, manifold);
    } else if (a->shapeType == RigidBody::ShapeType::Rectangle && b->shapeType == RigidBody::ShapeType::Circle) {
        return checkAABBCircle(a, b, manifold);
    } else if (a->shapeType == RigidBody::ShapeType::Circle && b->shapeType == RigidBody::ShapeType::Rectangle) {
        if (checkAABBCircle(b, a, manifold)) {
            manifold.normal = manifold.normal * -1.0f;
            std::swap(manifold.bodyA, manifold.bodyB);
            return true;
        }
        return false;
    } else {
        return checkSAT(a, b, manifold);
    }
}

void Collision::resolveCollision(CollisionManifold& manifold) {
    RigidBody* a = manifold.bodyA;
    RigidBody* b = manifold.bodyB;

    if (a->type == RigidBody::BodyType::Static && b->type == RigidBody::BodyType::Static) {
        return;
    }

    // Resolve penetration (separation)
    if (manifold.penetration > 0) {
        Vector2 correction = manifold.normal * manifold.penetration;
        if (a->type != RigidBody::BodyType::Static && b->type != RigidBody::BodyType::Static) {
            a->position -= correction * 0.5f;
            b->position += correction * 0.5f;
        } else if (a->type != RigidBody::BodyType::Static) {
            a->position -= correction;
        } else {
            b->position += correction;
        }
    }

    // Resolve velocity (impulse)
    Vector2 relativeVelocity = b->velocity - a->velocity;
    float velocityAlongNormal = relativeVelocity.dot(manifold.normal);

    if (velocityAlongNormal > 0) {
        return;  // Velocities are separating
    }

    float rCross = manifold.contactPoint1.cross(manifold.normal);
    float restitution = manifold.restitution;
    
    float impulseMagnitude = -(1.0f + restitution) * velocityAlongNormal;
    impulseMagnitude /= a->inverseMass + b->inverseMass + rCross * rCross * (a->inverseInertia + b->inverseInertia);

    Vector2 impulse = manifold.normal * impulseMagnitude;
    
    a->applyImpulse(impulse * -1.0f, manifold.contactPoint1 - a->position);
    b->applyImpulse(impulse, manifold.contactPoint1 - b->position);
}

}
