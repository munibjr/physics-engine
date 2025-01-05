#include "../include/world.h"
#include <iostream>

using namespace physics;

int main() {
    std::cout << "=== Pendulum (Constraint) Simulation ===" << std::endl;
    
    World world(Vector2(0, -9.81f));
    
    // Fixed pivot point (static body at top)
    RigidBody* pivot = world.createBody(RigidBody::BodyType::Static);
    pivot->position = Vector2(0, 5);
    pivot->shapeType = RigidBody::ShapeType::Circle;
    pivot->radius = 0.1f;
    
    // Pendulum bob
    RigidBody* bob = world.createBody(RigidBody::BodyType::Dynamic);
    bob->position = Vector2(3, 5);  // Offset from pivot
    bob->setMass(1.0f);
    bob->shapeType = RigidBody::ShapeType::Circle;
    bob->radius = 0.3f;
    bob->restitution = 0.9f;
    
    // Ground (for reference)
    RigidBody* ground = world.createBody(RigidBody::BodyType::Static);
    ground->position = Vector2(0, -5);
    ground->shapeType = RigidBody::ShapeType::Rectangle;
    ground->size = Vector2(20, 1);
    
    // Simulate (note: without explicit constraint solver, bob will drift)
    float dt = 0.016f;
    float time = 0.0f;
    int frame = 0;
    
    std::cout << "Time\t\tBob X\t\tBob Y\t\tDistance to Pivot" << std::endl;
    std::cout << "============================================================" << std::endl;
    
    // Max pendulum length
    float maxLength = Vector2::distance(pivot->position, bob->position);
    
    for (int i = 0; i < 600; ++i) {
        world.step(dt);
        time += dt;
        frame++;
        
        // Simple constraint: keep distance constant
        Vector2 toPivot = pivot->position - bob->position;
        float currentDist = toPivot.length();
        if (currentDist > maxLength) {
            Vector2 correction = toPivot.normalized() * (currentDist - maxLength) * 0.1f;
            bob->position += correction;
        }
        
        if (frame % 30 == 0) {
            float dist = Vector2::distance(pivot->position, bob->position);
            printf("%.2f\t\t%.2f\t\t%.2f\t\t%.2f\n", 
                   time, bob->position.x, bob->position.y, dist);
        }
    }
    
    std::cout << "\n=== Simulation Complete ===" << std::endl;
    
    return 0;
}
