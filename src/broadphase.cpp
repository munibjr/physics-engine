#include "../include/broadphase.h"
#include <algorithm>
#include <iostream>
#include <cfloat>

namespace physics {

bool AABB::intersects(const AABB& other) const {
    return !(max.x < other.min.x || min.x > other.max.x ||
             max.y < other.min.y || min.y > other.max.y);
}

AABB AABB::merged(const AABB& other) const {
    return AABB(
        Vector2(std::min(min.x, other.min.x), std::min(min.y, other.min.y)),
        Vector2(std::max(max.x, other.max.x), std::max(max.y, other.max.y))
    );
}

Vector2 AABB::center() const {
    return (min + max) * 0.5f;
}

Vector2 AABB::extent() const {
    return (max - min) * 0.5f;
}

BVH::BVH() : root(nullptr) {}

BVH::~BVH() {
    clear();
}

void BVH::insert(RigidBody* body) {
    // Simplified: just rebuild
    // Full implementation would do incremental insertion
}

void BVH::build(const std::vector<RigidBody*>& bodies) {
    clear();
    if (bodies.empty()) return;
    
    std::vector<RigidBody*> bodyList(bodies);
    root = buildTree(bodyList, 0);
}

void BVH::rebuild(const std::vector<RigidBody*>& bodies) {
    clear();
    build(bodies);
}

BVHNode* BVH::buildTree(std::vector<RigidBody*>& bodies, int depth) {
    if (bodies.empty()) return nullptr;
    if (bodies.size() == 1) {
        BVHNode* leaf = new BVHNode();
        leaf->body = bodies[0];
        leaf->aabb = getBodyAABB(bodies[0]);
        leaf->depth = depth;
        return leaf;
    }

    // Find best split axis
    Vector2 minBounds(FLT_MAX, FLT_MAX);
    Vector2 maxBounds(-FLT_MAX, -FLT_MAX);
    
    for (auto body : bodies) {
        AABB aabb = getBodyAABB(body);
        minBounds.x = std::min(minBounds.x, aabb.min.x);
        minBounds.y = std::min(minBounds.y, aabb.min.y);
        maxBounds.x = std::max(maxBounds.x, aabb.max.x);
        maxBounds.y = std::max(maxBounds.y, aabb.max.y);
    }

    Vector2 extent = maxBounds - minBounds;
    int axis = extent.x > extent.y ? 0 : 1;  // 0 for x, 1 for y

    // Sort bodies along axis
    std::sort(bodies.begin(), bodies.end(), [axis](const RigidBody* a, const RigidBody* b) -> bool {
        float aVal = axis == 0 ? a->position.x : a->position.y;
        float bVal = axis == 0 ? b->position.x : b->position.y;
        return aVal < bVal;
    });

    // Split in half
    size_t mid = bodies.size() / 2;
    std::vector<RigidBody*> left(bodies.begin(), bodies.begin() + mid);
    std::vector<RigidBody*> right(bodies.begin() + mid, bodies.end());

    BVHNode* node = new BVHNode();
    node->left = buildTree(left, depth + 1);
    node->right = buildTree(right, depth + 1);
    node->depth = depth;

    // Compute AABB as union of children
    if (node->left && node->right) {
        node->aabb = node->left->aabb.merged(node->right->aabb);
    } else if (node->left) {
        node->aabb = node->left->aabb;
    } else if (node->right) {
        node->aabb = node->right->aabb;
    }

    return node;
}

void BVH::getPotentialCollisions(std::vector<std::pair<RigidBody*, RigidBody*>>& pairs) {
    pairs.clear();
    if (root) {
        getPotentialCollisionsRecursive(root, pairs);
    }
}

void BVH::getPotentialCollisionsRecursive(BVHNode* node, std::vector<std::pair<RigidBody*, RigidBody*>>& pairs) {
    if (!node) return;

    if (!node->left && !node->right) {
        // Leaf node
        return;
    }

    if (node->left && node->right && node->left->aabb.intersects(node->right->aabb)) {
        if (!node->left->left && !node->left->right && !node->right->left && !node->right->right) {
            // Both children are leaves
            pairs.push_back({node->left->body, node->right->body});
        } else {
            getPotentialCollisionsRecursive(node->left, pairs);
            getPotentialCollisionsRecursive(node->right, pairs);
        }
    }
}

void BVH::clear() {
    deleteTree(root);
    root = nullptr;
}

void BVH::deleteTree(BVHNode* node) {
    if (!node) return;
    deleteTree(node->left);
    deleteTree(node->right);
    delete node;
}

AABB BVH::getBodyAABB(RigidBody* body) const {
    Vector2 extent;
    if (body->shapeType == RigidBody::ShapeType::Circle) {
        extent = Vector2(body->radius, body->radius);
    } else {
        extent = body->size * 0.5f;
    }
    return AABB(body->position - extent, body->position + extent);
}

void BVH::debugPrint() const {
    if (!root) {
        std::cout << "Empty BVH" << std::endl;
        return;
    }
    std::cout << "BVH tree depth: " << root->depth << std::endl;
}

}
