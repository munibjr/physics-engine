#include "../include/vector2.h"
#include <cmath>
#include <cassert>
#include <iostream>

using namespace physics;

void testVectorOperations() {
    std::cout << "Testing Vector2 operations..." << std::endl;
    
    Vector2 v1(3, 4);
    Vector2 v2(1, 2);
    
    // Length
    assert(std::abs(v1.length() - 5.0f) < 0.001f);
    
    // Dot product
    assert(std::abs(v1.dot(v2) - 11.0f) < 0.001f);
    
    // Cross product (2D)
    assert(std::abs(v1.cross(v2) - 2.0f) < 0.001f);
    
    // Normalization
    Vector2 normalized = v1.normalized();
    assert(std::abs(normalized.length() - 1.0f) < 0.001f);
    
    // Addition
    Vector2 sum = v1 + v2;
    assert(sum.x == 4 && sum.y == 6);
    
    // Subtraction
    Vector2 diff = v1 - v2;
    assert(diff.x == 2 && diff.y == 2);
    
    // Scalar multiplication
    Vector2 scaled = v1 * 2;
    assert(scaled.x == 6 && scaled.y == 8);
    
    // Distance
    float dist = Vector2::distance(v1, v2);
    assert(dist > 0);
    
    std::cout << "✓ Vector2 operations passed" << std::endl;
}

void testVectorRotation() {
    std::cout << "Testing vector rotation..." << std::endl;
    
    Vector2 v(1, 0);
    
    // 90 degree rotation
    Vector2 rotated = v.rotated(3.14159f / 2);
    assert(std::abs(rotated.x) < 0.001f);
    assert(std::abs(rotated.y - 1.0f) < 0.001f);
    
    std::cout << "✓ Vector rotation passed" << std::endl;
}

void testEdgeCases() {
    std::cout << "Testing edge cases..." << std::endl;
    
    // Zero vector
    Vector2 zero(0, 0);
    assert(zero.length() == 0);
    
    // Very small values
    Vector2 small(0.00001f, 0.00001f);
    assert(small.length() > 0);
    
    // Division by zero safety
    Vector2 divided = small / 0;
    assert(divided.x == 0 && divided.y == 0);
    
    std::cout << "✓ Edge cases passed" << std::endl;
}

int main() {
    std::cout << "=== Vector2 Unit Tests ===" << std::endl;
    
    testVectorOperations();
    testVectorRotation();
    testEdgeCases();
    
    std::cout << "\n✓ All Vector2 tests passed!" << std::endl;
    return 0;
}
