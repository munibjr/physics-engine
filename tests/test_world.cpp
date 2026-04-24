#include "../include/world.h"
#include <cassert>
#include <iostream>
#include <cmath>

using namespace physics;

void testGravity() {
    std::cout << "Testing gravity..." << std::endl;
    
    World world(Vector2(0, -9.81f));
    
    RigidBody* body = world.createBody(RigidBody::BodyType::Dynamic);
    body->position = Vector2(0, 10);
    body->setMass(1.0f);
    
    world.step(0.1f);
    
    // Velocity should increase downward due to gravity
    assert(body->velocity.y < 0);
    assert(std::abs(body->velocity.y - (-0.981f)) < 0.1f);
    
    std::cout << "✓ Gravity test passed" << std::endl;
}

void testStaticBodies() {
    std::cout << "Testing static bodies..." << std::endl;

    World world(Vector2(0, -9.81f));

    RigidBody* staticBody = world.createBody(RigidBody::BodyType::Static);
    staticBody->position = Vector2(0, 0);

    assert(staticBody->inverseMass == 0);
    assert(staticBody->isStatic());

    std::cout << "✓ Static body test passed" << std::endl;
}

void testForceApplication() {
    std::cout << "Testing force application..." << std::endl;
    
    World world(Vector2(0, 0));
    
    RigidBody* body = world.createBody(RigidBody::BodyType::Dynamic);
    body->position = Vector2(0, 0);
    body->setMass(1.0f);
    
    // Apply force in X direction
    body->applyForce(Vector2(10, 0));
    world.step(0.1f);
    
    // Velocity should be in X direction
    assert(body->velocity.x > 0);
    
    std::cout << "✓ Force application test passed" << std::endl;
}

void testBodyCreationDeletion() {
    std::cout << "Testing body creation/deletion..." << std::endl;
    
    World world(Vector2(0, -9.81f));
    
    assert(world.getBodies().size() == 0);
    
    RigidBody* body1 = world.createBody();
    assert(world.getBodies().size() == 1);
    
    RigidBody* body2 = world.createBody();
    assert(world.getBodies().size() == 2);
    
    world.destroyBody(body1);
    assert(world.getBodies().size() == 1);
    
    world.clear();
    assert(world.getBodies().size() == 0);
    
    std::cout << "✓ Body creation/deletion test passed" << std::endl;
}

void testCollisionDetection() {
    std::cout << "Testing world collision detection..." << std::endl;
    
    World world(Vector2(0, 0));
    
    RigidBody* ball1 = world.createBody(RigidBody::BodyType::Dynamic);
    ball1->position = Vector2(0, 0);
    ball1->shapeType = RigidBody::ShapeType::Circle;
    ball1->radius = 1.0f;
    ball1->setMass(1.0f);
    
    RigidBody* ball2 = world.createBody(RigidBody::BodyType::Dynamic);
    ball2->position = Vector2(1.5f, 0);
    ball2->shapeType = RigidBody::ShapeType::Circle;
    ball2->radius = 1.0f;
    ball2->setMass(1.0f);
    
    world.step(0.016f);
    
    // Should detect collision
    assert(world.getCollisions().size() > 0);
    
    std::cout << "✓ Collision detection test passed" << std::endl;
}

int main() {
    std::cout << "=== World Integration Unit Tests ===" << std::endl;
    
    testGravity();
    testStaticBodies();
    testForceApplication();
    testBodyCreationDeletion();
    testCollisionDetection();
    
    std::cout << "\n✓ All world tests passed!" << std::endl;
    return 0;
}
