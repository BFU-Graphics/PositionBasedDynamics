#include "distance_field_collision_detection.h"

#include "src/objects/objects.h"

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
                collider_pairs.emplace_back(i, j);

    std::vector<std::vector<ContactData> > contacts_mt; // multi-thread contacts
#ifdef USE_OPENMP
#ifdef _DEBUG
    const unsigned int maxThreads = 1;
#else
    const unsigned int maxThreads = omp_get_max_threads();
#endif
    contacts_mt.resize(maxThreads);
#endif
#pragma omp parallel for
    for (int i = 0; i < collision_objects_.size(); ++i)
    {

    }
}
