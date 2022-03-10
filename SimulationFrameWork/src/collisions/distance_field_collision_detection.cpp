#include "distance_field_collision_detection.h"

#include "src/objects/rigidbody.h"

#ifdef USE_OPENMP
#include "omp.h"
#endif

HINASIM::CollisionDetection &HINASIM::DistanceFieldCollisionDetection::add_collider_sphere(HINASIM::SimObject *o)
{
    colliders_.emplace_back(new SphereColliderDF(o));
    return *this;
}

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
            dfc->update_bvh(); // NOTE: Rigid Body don't need to update BVH at runtime
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
    std::vector<std::vector<ContactData>> contacts_mt; // multi-thread contacts
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

        auto *body1 = collider1->object_;
        auto *body2 = collider2->object_;

        if (dynamic_cast<RigidBody *>(body1) && dynamic_cast<RigidBody *>(body2))
        {
            auto *rb1 = dynamic_cast<RigidBody *>(body1);
            auto *rb2 = dynamic_cast<RigidBody *>(body2);
            collision_detection_RB_RB(rb1, collider1, rb2, collider2, contacts_mt);
        }
    }

    // generate collision contact constraints
    for (auto &contacts: contacts_mt)
    {
        for (auto &contact: contacts)
        {
            // TODO: generate contacts constraints
        }
    }
}

void HINASIM::DistanceFieldCollisionDetection::collision_detection_RB_RB(HINASIM::RigidBody *rb1, HINASIM::DistanceFieldCollider *co1, HINASIM::RigidBody *rb2, HINASIM::DistanceFieldCollider *co2,
                                                                         std::vector<std::vector<ContactData>> &contacts_mt)
{
    const double restitution = rb1->restitution_ * rb2->restitution_;
    const double friction = rb1->friction_ + rb2->friction_;

    if (rb1->inv_mass_ == 0.0 && rb2->inv_mass_ == 0.0)
    {
        const Eigen::Vector3d &com2 = rb2->position_;

        const Eigen::Matrix3d &R = rb2->transformation_R_;
        const Eigen::Vector3d &v1 = rb2->transformation_v1_;
        const Eigen::Vector3d &v2 = rb2->transformation_v2_;

        const PointCloudBSH *bvh = co1->bvh_;

        // TODO: NOT COMPLETE YET BELOW
        std::function<bool(unsigned int, unsigned int)> predicate = [&](unsigned int node_index, unsigned int depth)
        {
            const BoundingSphere &bs = bvh->hull(node_index);
            const Eigen::Vector3d &sphere_x = bs.x_;
            const Eigen::Vector3d sphere_x_w = rb1->rotation_ * sphere_x + rb1->position_; // TODO: potential bugs

            Eigen::AlignedBox3d box3f;
            box3f.extend(co2->aabb_->aabb_[0]);
            box3f.extend(co2->aabb_->aabb_[1]);
            const double dist = box3f.exteriorDistance(sphere_x_w);

            // Test if center of bounding sphere intersects AABB
            if (dist < bs.r_)
            {
                // Test if distance of center of bounding sphere to collision object is smaller than the radius
                const Eigen::Vector3d x = R * (sphere_x_w - com2) + v1;
                const double dist2 = co2->distance(x, tolerance_);
                if (dist2 == std::numeric_limits<double>::max())
                    return true;
                if (dist2 < bs.r_)
                    return true;
            }
            return false;
        };
    }
}
