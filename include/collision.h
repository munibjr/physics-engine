#pragma once

#include "vector2.h"

namespace physics {

class RigidBody;

// Collision manifold stores collision information
struct CollisionManifold {
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector2 normal;  // From A to B
    float penetration;
    Vector2 contactPoint1;
    Vector2 contactPoint2;
    int contactCount;
    float restitution;

    CollisionManifold() : bodyA(nullptr), bodyB(nullptr), penetration(0), contactCount(0), restitution(0) {}
};

// Collision detection class
class Collision {
public:
    // AABB collision detection
    static bool checkAABB(const RigidBody* a, const RigidBody* b);

    // Circle collision detection
    static bool checkCircle(const RigidBody* a, const RigidBody* b, CollisionManifold& manifold);

    // AABB vs Circle
    static bool checkAABBCircle(const RigidBody* aabb, const RigidBody* circle, CollisionManifold& manifold);

    // SAT (Separating Axis Theorem) for rotated rectangles
    static bool checkSAT(const RigidBody* a, const RigidBody* b, CollisionManifold& manifold);

    // General collision check (dispatches to appropriate test)
    static bool checkCollision(RigidBody* a, RigidBody* b, CollisionManifold& manifold);

    // Collision response (resolve penetration and impulse)
    static void resolveCollision(CollisionManifold& manifold);

private:
    // Helper functions for SAT
    static Vector2 getProjection(const RigidBody* body, const Vector2& axis);
    static bool projectionsOverlap(const Vector2& proj1, const Vector2& proj2, float& penetration);
    static Vector2 getNormal(const RigidBody* body, int edgeIndex);
};

}
