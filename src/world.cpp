#include "../include/world.h"
#include <iostream>

namespace physics {

World::World(Vector2 gravity) : gravity(gravity) {}

World::~World() {
    clear();
}

RigidBody* World::createBody(RigidBody::BodyType type) {
    RigidBody* body = new RigidBody(type);
    bodies.push_back(body);
    return body;
}

void World::destroyBody(RigidBody* body) {
    auto it = std::find(bodies.begin(), bodies.end(), body);
    if (it != bodies.end()) {
        bodies.erase(it);
        delete body;
    }
}

void World::clear() {
    for (auto body : bodies) {
        delete body;
    }
    bodies.clear();
    collisions.clear();
}

void World::step(float dt, int iterations) {
    applyForces(dt);
    integrateVelocities(dt);
    
    for (int i = 0; i < iterations; ++i) {
        broadPhaseCollisionDetection();
        narrowPhaseCollisionDetection();
        resolveCollisions();
    }
    
    integratePositions(dt);
    clearForces();
}

void World::applyForces(float dt) {
    for (auto body : bodies) {
        if (body->type == RigidBody::BodyType::Static) continue;
        
        // Apply gravity
        body->applyForce(gravity * body->mass);
    }
}

void World::integrateVelocities(float dt) {
    for (auto body : bodies) {
        body->integrateVelocity(dt);
    }
}

void World::broadPhaseCollisionDetection() {
    // Simple broad phase: check all pairs
    // In production, use spatial partitioning (BVH, quad-tree)
    collisions.clear();
    
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            RigidBody* a = bodies[i];
            RigidBody* b = bodies[j];
            
            // Quick AABB check first
            float dx = a->position.x - b->position.x;
            float dy = a->position.y - b->position.y;
            float maxDist = 5.0f;  // Tunable
            
            if (dx * dx + dy * dy < maxDist * maxDist) {
                CollisionManifold manifold;
                if (Collision::checkCollision(a, b, manifold)) {
                    collisions.push_back(manifold);
                }
            }
        }
    }
}

void World::narrowPhaseCollisionDetection() {
    // Already done in broadPhase for simplicity
}

void World::resolveCollisions() {
    for (auto& manifold : collisions) {
        Collision::resolveCollision(manifold);
    }
}

void World::integratePositions(float dt) {
    for (auto body : bodies) {
        body->integratePosition(dt);
    }
}

void World::clearForces() {
    for (auto body : bodies) {
        body->clearForces();
    }
}

void World::printStats() const {
    std::cout << "World Stats:" << std::endl;
    std::cout << "  Bodies: " << bodies.size() << std::endl;
    std::cout << "  Collisions: " << collisions.size() << std::endl;
    std::cout << "  Gravity: "; gravity.print();
}

}
