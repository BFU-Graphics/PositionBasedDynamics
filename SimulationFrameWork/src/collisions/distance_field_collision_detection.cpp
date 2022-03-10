#include "distance_field_collision_detection.h"

#include "src/objects/objects.h"

#ifdef USE_OPENMP
#include "omp.h"
#endif

void HINASIM::DistanceFieldCollisionDetection::collision_detection()
{
    if (colliders_.size() <= 1)
        return;

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < colliders_.size(); ++i)
    {
        if (dynamic_cast<DistanceFieldCollider *>(colliders_[i]))
        {
            auto *dfc = dynamic_cast<DistanceFieldCollider *>(colliders_[i]);
            dfc->update_aabb();
            dfc->update_bvh();
        }
    }

    // Broad Phase collision detection
    std::vector<std::pair<unsigned int, unsigned int>> collider_pairs;
    for (int i = 0; i < colliders_.size() - 1; ++i)
        for (int j = i + 1; j < colliders_.size(); ++j)
            if (AABB::intersection(*(colliders_[i]->aabb_), *(colliders_[j]->aabb_)))
            {
                // TODO: self collisions for deformables in the future
                collider_pairs.emplace_back(i, j);
                collider_pairs.emplace_back(j, i);
            }

    // Narrow Phase collision detection
    std::vector<std::vector<ContactData> > contacts_mt; // multi-thread contacts
#ifdef USE_OPENMP
#ifdef _DEBUG
    contacts_mt.resize(1);
#else
    contacts_mt.resize(omp_get_max_threads());
#endif
#pragma omp parallel for
#endif
    for (int i = 0; i < collider_pairs.size(); ++i)
    {
        auto *collider1 = dynamic_cast<DistanceFieldCollider *>(colliders_[collider_pairs[i].first]);
        auto *collider2 = dynamic_cast<DistanceFieldCollider *>(colliders_[collider_pairs[i].second]);

        if (!collider1 || !collider2)
            continue;


    }
}

HINASIM::CollisionDetection &HINASIM::DistanceFieldCollisionDetection::add_collider_sphere(HINASIM::SimObject *o)
{
    colliders_.emplace_back(new SphereColliderDF(o));
    return *this;
}
