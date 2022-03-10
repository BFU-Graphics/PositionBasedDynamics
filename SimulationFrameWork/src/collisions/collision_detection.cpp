#include "collision_detection.h"

HINASIM::CollisionDetection::CollisionDetection()
{
    colliders_.reserve(1000);
    tolerance_ = 0.01;
}

HINASIM::CollisionDetection::~CollisionDetection()
{
    for (auto &collider: colliders_)
        delete collider;
    colliders_.clear();
}