#include "../include/world.h"
#include <iostream>

using namespace physics;

int main() {
    std::cout << "=== Ball Drop Simulation ===" << std::endl;
    
    World world(Vector2(0, -9.81f));
    
    // Create a ball
    RigidBody* ball = world.createBody(RigidBody::BodyType::Dynamic);
    ball->position = Vector2(0, 10);
    ball->setMass(1.0f);
    ball->shapeType = RigidBody::ShapeType::Circle;
    ball->radius = 0.5f;
    ball->restitution = 0.7f;
    ball->friction = 0.3f;
    
    // Create ground
    RigidBody* ground = world.createBody(RigidBody::BodyType::Static);
    ground->position = Vector2(0, -5);
    ground->shapeType = RigidBody::ShapeType::Rectangle;
    ground->size = Vector2(20, 1);
    ground->restitution = 0.5f;
    
    // Create walls
    RigidBody* leftWall = world.createBody(RigidBody::BodyType::Static);
    leftWall->position = Vector2(-10, 0);
    leftWall->shapeType = RigidBody::ShapeType::Rectangle;
    leftWall->size = Vector2(1, 20);
    
    RigidBody* rightWall = world.createBody(RigidBody::BodyType::Static);
    rightWall->position = Vector2(10, 0);
    rightWall->shapeType = RigidBody::ShapeType::Rectangle;
    rightWall->size = Vector2(1, 20);
    
    // Simulate
    float dt = 0.016f;  // 60 FPS
    float time = 0.0f;
    int frame = 0;
    
    std::cout << "Time\t\tPos X\t\tPos Y\t\tVel Y\t\tCollisions" << std::endl;
    std::cout << "================================================================" << std::endl;
    
    for (int i = 0; i < 600; ++i) {
        world.step(dt);
        time += dt;
        frame++;
        
        // Print every 30 frames (~0.5 seconds)
        if (frame % 30 == 0) {
            printf("%.2f\t\t%.2f\t\t%.2f\t\t%.2f\t\t%zu\n",
                   time,
                   ball->position.x,
                   ball->position.y,
                   ball->velocity.y,
                   world.getCollisions().size());
        }
    }
    
    std::cout << "\n=== Simulation Complete ===" << std::endl;
    world.printStats();
    
    return 0;
}
