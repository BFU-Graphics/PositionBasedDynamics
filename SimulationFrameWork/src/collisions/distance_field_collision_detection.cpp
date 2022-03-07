#include "distance_field_collision_detection.h"

#include "src/objects/rigidbody.h"

HINASIM::DistanceFieldCollisionObject::DistanceFieldCollisionObject(SimObject *object)
{
    assert(object); // ensure object is not nullptr

    object_ = object;
    bvh_.init(object_->V_);
    bvh_.construct();
}

double HINASIM::DistanceFieldCollisionSphere::distance(const Eigen::Vector3d &x, double tolerance)
{
    const Eigen::Vector3d d = x.template cast<double>(); // ??? don't know yet

    const double dist = (d.norm() - radius_);
    return invertSDF_ ? (-dist - tolerance) : (dist - tolerance);
}

// ======================================== Distance Collision Detection ========================================

HINASIM::DistanceFieldCollisionDetection::DistanceFieldCollisionDetection() : CollisionDetection()
{}

void HINASIM::DistanceFieldCollisionDetection::collision_detection()
{
    std::vector<std::pair<unsigned int, unsigned int>> collider_pairs;
    for (int i = 0; i < collision_objects_.size(); ++i)
        for (int j = 0; j < collision_objects_.size(); ++j)
            if (i != j)
                // TODO: self collisions for deformables in the future
                collider_pairs.emplace_back(i, j);
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < collision_objects_.size(); ++i)
    {
        auto *co = collision_objects_[i];

        if (dynamic_cast<DistanceFieldCollisionObject *>(co))
        {
            auto *dfco = dynamic_cast<DistanceFieldCollisionObject *>(co);
            // TODO: update AABB

            // TODO: update BVH
        }
    }


    std::vector<std::vector<ContactData> > contacts_mt; // multi-thread contacts
#ifdef USE_OPENMP
#ifdef _DEBUG
    const unsigned int maxThreads = 1;
#else
    const unsigned int maxThreads = omp_get_max_threads();
#endif
    contacts_mt.resize(maxThreads);
#pragma omp parallel for
#endif
    for (int i = 0; i < collider_pairs.size(); ++i)
    {
        std::pair<unsigned int, unsigned int> &collider_pair = collider_pairs[i];

        // collision detection rb silid
    }
}

void HINASIM::DistanceFieldCollisionDetection::collision_detection_rigid_rigid(RigidBody *rb1, HINASIM::DistanceFieldCollisionObject *co1, RigidBody *rb2, HINASIM::DistanceFieldCollisionObject *co2,
                                                                               std::vector<std::vector<ContactData>> &contacts_mt, double restitution, double friction)
{


}
