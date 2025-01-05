#include "../include/world.h"
#include <iostream>

using namespace physics;

int main() {
    std::cout << "=== Block Stacking Simulation ===" << std::endl;
    
    World world(Vector2(0, -9.81f));
    
    // Create ground
    RigidBody* ground = world.createBody(RigidBody::BodyType::Static);
    ground->position = Vector2(0, -5);
    ground->shapeType = RigidBody::ShapeType::Rectangle;
    ground->size = Vector2(20, 1);
    ground->restitution = 0.2f;
    
    // Create a stack of blocks
    int stackHeight = 5;
    float blockWidth = 1.0f;
    float blockHeight = 1.0f;
    
    std::cout << "Creating stack of " << stackHeight << " blocks..." << std::endl;
    
    for (int i = 0; i < stackHeight; ++i) {
        RigidBody* block = world.createBody(RigidBody::BodyType::Dynamic);
        block->position = Vector2(0, -4 + i * blockHeight);
        block->setMass(1.0f);
        block->shapeType = RigidBody::ShapeType::Rectangle;
        block->size = Vector2(blockWidth, blockHeight);
        block->restitution = 0.3f;
        block->friction = 0.5f;
    }
    
    // Simulate
    float dt = 0.016f;
    float time = 0.0f;
    int frame = 0;
    
    std::cout << "\nSimulating stack stability...\n" << std::endl;
    
    for (int i = 0; i < 1000; ++i) {
        world.step(dt);
        time += dt;
        frame++;
        
        if (frame % 100 == 0) {
            std::cout << "Time: " << time << "s | Bodies: " << world.getBodies().size() 
                      << " | Collisions: " << world.getCollisions().size() << std::endl;
        }
    }
    
    std::cout << "\n=== Final State ===" << std::endl;
    const auto& bodies = world.getBodies();
    for (size_t i = 1; i < bodies.size(); ++i) {  // Skip ground
        printf("Block %zu: Y=%.2f, Vel Y=%.2f\n", i, bodies[i]->position.y, bodies[i]->velocity.y);
    }
    
    return 0;
}
