#pragma once

#include "vector2.h"
#include "rigidbody.h"
#include <vector>
#include <memory>

namespace physics {

// AABB bounding box
struct AABB {
    Vector2 min, max;

    AABB() : min(0, 0), max(0, 0) {}
    AABB(Vector2 min, Vector2 max) : min(min), max(max) {}

    bool intersects(const AABB& other) const;
    AABB merged(const AABB& other) const;
    Vector2 center() const;
    Vector2 extent() const;
};

// BVH tree node
struct BVHNode {
    AABB aabb;
    RigidBody* body;
    BVHNode* left;
    BVHNode* right;
    int depth;

    BVHNode() : body(nullptr), left(nullptr), right(nullptr), depth(0) {}
};

// Bounding Volume Hierarchy for broad-phase collision detection
class BVH {
public:
    BVH();
    ~BVH();

    void insert(RigidBody* body);
    void build(const std::vector<RigidBody*>& bodies);
    void rebuild(const std::vector<RigidBody*>& bodies);

    // Find potential collisions
    void getPotentialCollisions(std::vector<std::pair<RigidBody*, RigidBody*>>& pairs);

    void clear();
    void debugPrint() const;

private:
    BVHNode* root;

    BVHNode* buildTree(std::vector<RigidBody*>& bodies, int depth);
    void getPotentialCollisionsRecursive(BVHNode* node, std::vector<std::pair<RigidBody*, RigidBody*>>& pairs);
    void deleteTree(BVHNode* node);
    AABB getBodyAABB(RigidBody* body) const;
};

}
