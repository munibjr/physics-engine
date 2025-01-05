#include "../include/world.h"
#include <iostream>
#include <chrono>
#include <vector>

using namespace physics;

double getCurrentTimeMs() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double, std::milli>(duration).count();
}

void benchmarkSimulation(int numBodies) {
    std::cout << "Benchmarking " << numBodies << " bodies..." << std::endl;
    
    World world(Vector2(0, -9.81f));
    
    // Create bodies in a grid
    int gridSize = (int)std::sqrt(numBodies);
    float spacing = 2.5f;
    
    for (int i = 0; i < gridSize; ++i) {
        for (int j = 0; j < gridSize; ++j) {
            RigidBody* body = world.createBody(RigidBody::BodyType::Dynamic);
            body->position = Vector2(i * spacing - gridSize * spacing / 2, j * spacing + 5);
            body->setMass(1.0f);
            body->shapeType = RigidBody::ShapeType::Circle;
            body->radius = 0.4f;
            body->restitution = 0.8f;
        }
    }
    
    // Ground
    RigidBody* ground = world.createBody(RigidBody::BodyType::Static);
    ground->position = Vector2(0, -10);
    ground->shapeType = RigidBody::ShapeType::Rectangle;
    ground->size = Vector2(100, 2);
    
    // Simulate for 5 seconds at 60 FPS
    float dt = 0.016f;
    int frames = 300;
    
    double startTime = getCurrentTimeMs();
    
    for (int frame = 0; frame < frames; ++frame) {
        world.step(dt);
    }
    
    double endTime = getCurrentTimeMs();
    double totalTime = endTime - startTime;
    double timePerFrame = totalTime / frames;
    double fps = 1000.0 / timePerFrame;
    
    std::cout << "  Total time: " << totalTime << " ms" << std::endl;
    std::cout << "  Time/frame: " << timePerFrame << " ms" << std::endl;
    std::cout << "  FPS: " << fps << std::endl;
    std::cout << "  Collisions: " << world.getCollisions().size() << std::endl;
    std::cout << std::endl;
}

void benchmarkCollisionDetection() {
    std::cout << "Benchmarking collision detection..." << std::endl;
    
    World world(Vector2(0, 0));
    
    // Create two overlapping circles
    RigidBody* ball1 = world.createBody(RigidBody::BodyType::Dynamic);
    ball1->position = Vector2(0, 0);
    ball1->shapeType = RigidBody::ShapeType::Circle;
    ball1->radius = 1.0f;
    
    RigidBody* ball2 = world.createBody(RigidBody::BodyType::Dynamic);
    ball2->position = Vector2(1.5f, 0);
    ball2->shapeType = RigidBody::ShapeType::Circle;
    ball2->radius = 1.0f;
    
    // Run many collision checks
    int iterations = 100000;
    
    double startTime = getCurrentTimeMs();
    
    for (int i = 0; i < iterations; ++i) {
        CollisionManifold manifold;
        Collision::checkCollision(ball1, ball2, manifold);
    }
    
    double endTime = getCurrentTimeMs();
    double totalTime = endTime - startTime;
    
    std::cout << "  Iterations: " << iterations << std::endl;
    std::cout << "  Total time: " << totalTime << " ms" << std::endl;
    std::cout << "  Time/check: " << (totalTime / iterations) << " ms" << std::endl;
    std::cout << std::endl;
}

int main() {
    std::cout << "=== Physics Engine Benchmarks ===" << std::endl << std::endl;
    
    std::cout << "Running simulation benchmarks..." << std::endl << std::endl;
    benchmarkSimulation(25);    // 5x5 grid
    benchmarkSimulation(100);   // 10x10 grid
    benchmarkSimulation(400);   // 20x20 grid
    
    std::cout << "Running collision detection benchmarks..." << std::endl << std::endl;
    benchmarkCollisionDetection();
    
    std::cout << "=== Benchmark Complete ===" << std::endl;
    
    return 0;
}
