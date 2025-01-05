#pragma once

#include "vector2.h"
#include "rigidbody.h"
#include "collision.h"
#include <vector>

namespace physics {

class World {
public:
    World(Vector2 gravity = Vector2(0, -9.81f));
    ~World();

    // Body management
    RigidBody* createBody(RigidBody::BodyType type = RigidBody::BodyType::Dynamic);
    void destroyBody(RigidBody* body);
    void clear();

    // Simulation
    void step(float dt, int iterations = 1);

    // Access
    const std::vector<RigidBody*>& getBodies() const { return bodies; }
    const std::vector<CollisionManifold>& getCollisions() const { return collisions; }
    Vector2 getGravity() const { return gravity; }
    void setGravity(const Vector2& g) { gravity = g; }

    // Debug
    void printStats() const;

private:
    std::vector<RigidBody*> bodies;
    std::vector<CollisionManifold> collisions;
    Vector2 gravity;

    void applyForces(float dt);
    void integrateVelocities(float dt);
    void broadPhaseCollisionDetection();
    void narrowPhaseCollisionDetection();
    void resolveCollisions();
    void integratePositions(float dt);
    void clearForces();
};

}
