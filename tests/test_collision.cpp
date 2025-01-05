#include "../include/world.h"
#include "../include/collision.h"
#include <cassert>
#include <iostream>
#include <cmath>

using namespace physics;

void testCircleCollision() {
    std::cout << "Testing circle-circle collision..." << std::endl;
    
    RigidBody a(RigidBody::BodyType::Dynamic);
    a.position = Vector2(0, 0);
    a.shapeType = RigidBody::ShapeType::Circle;
    a.radius = 1.0f;
    
    RigidBody b(RigidBody::BodyType::Dynamic);
    b.position = Vector2(1.5f, 0);
    b.shapeType = RigidBody::ShapeType::Circle;
    b.radius = 1.0f;
    
    CollisionManifold manifold;
    bool colliding = Collision::checkCircle(&a, &b, manifold);
    
    assert(colliding);
    assert(manifold.penetration > 0);
    assert(std::abs(manifold.normal.x - 1.0f) < 0.001f);
    
    std::cout << "✓ Circle collision test passed" << std::endl;
}

void testCircleNoCollision() {
    std::cout << "Testing circle non-collision..." << std::endl;
    
    RigidBody a(RigidBody::BodyType::Dynamic);
    a.position = Vector2(0, 0);
    a.shapeType = RigidBody::ShapeType::Circle;
    a.radius = 1.0f;
    
    RigidBody b(RigidBody::BodyType::Dynamic);
    b.position = Vector2(5, 0);
    b.shapeType = RigidBody::ShapeType::Circle;
    b.radius = 1.0f;
    
    CollisionManifold manifold;
    bool colliding = Collision::checkCircle(&a, &b, manifold);
    
    assert(!colliding);
    
    std::cout << "✓ Circle non-collision test passed" << std::endl;
}

void testAABBCollision() {
    std::cout << "Testing AABB collision..." << std::endl;
    
    RigidBody a(RigidBody::BodyType::Dynamic);
    a.position = Vector2(0, 0);
    a.shapeType = RigidBody::ShapeType::Rectangle;
    a.size = Vector2(2, 2);
    
    RigidBody b(RigidBody::BodyType::Dynamic);
    b.position = Vector2(1.5f, 0);
    b.shapeType = RigidBody::ShapeType::Rectangle;
    b.size = Vector2(2, 2);
    
    bool colliding = Collision::checkAABB(&a, &b);
    assert(colliding);
    
    std::cout << "✓ AABB collision test passed" << std::endl;
}

void testRestitution() {
    std::cout << "Testing restitution..." << std::endl;
    
    RigidBody a(RigidBody::BodyType::Dynamic);
    a.setMass(1.0f);
    a.velocity = Vector2(1, 0);
    a.restitution = 0.8f;
    
    RigidBody b(RigidBody::BodyType::Static);
    b.position = Vector2(5, 0);
    
    float e = std::min(a.restitution, b.restitution);
    assert(std::abs(e - 0.4f) < 0.001f);  // min(0.8, 0.5)
    
    std::cout << "✓ Restitution test passed" << std::endl;
}

int main() {
    std::cout << "=== Collision Detection Unit Tests ===" << std::endl;
    
    testCircleCollision();
    testCircleNoCollision();
    testAABBCollision();
    testRestitution();
    
    std::cout << "\n✓ All collision tests passed!" << std::endl;
    return 0;
}
