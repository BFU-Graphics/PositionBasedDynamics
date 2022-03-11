#include "distance_field_collision_detection.h"

#include "src/objects/rigidbody.h"

#ifdef USE_OPENMP
#include "omp.h"
#endif

HINASIM::DistanceFieldCollisionDetection &HINASIM::DistanceFieldCollisionDetection::add_collider_sphere(HINASIM::SimObject *o)
{
    colliders_.emplace_back(new SphereColliderDF(o));
    return *this;
}

void HINASIM::DistanceFieldCollisionDetection::collision_detection()
{
    if (colliders_.size() <= 1)
        return;

    // clear collision contacts every physics loop
    for (auto contact: contacts_)
        contact->clear();

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < colliders_.size(); ++i)
        if (dynamic_cast<DistanceFieldCollider *>(colliders_[i]))
        {
            auto *dfc = dynamic_cast<DistanceFieldCollider *>(colliders_[i]);
            dfc->update_aabb();
            dfc->update_bvh(); // NOTE: Rigid Body don't need to update BVH at runtime
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
    contacts_mt.resize(1);
#ifdef USE_OPENMP
#ifdef _DEBUG
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

        // NOTE: only rigid-rigid contact currently
        // TODO: support more contact types in the future
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
            // NOTE: only rigid-rigid contact currently
            // TODO: support more contact types in the future
            add_rigid_body_contact(
                    dynamic_cast<RigidBody *>(contact.object1),
                    dynamic_cast<RigidBody *>(contact.object2),
                    contact.contact_point1,
                    contact.contact_point2,
                    contact.normal,
                    contact.distance,
                    contact.restitution,
                    contact.friction
            );
        }
    }
}

void HINASIM::DistanceFieldCollisionDetection::collision_detection_RB_RB(HINASIM::RigidBody *rb1, HINASIM::DistanceFieldCollider *co1, HINASIM::RigidBody *rb2, HINASIM::DistanceFieldCollider *co2,
                                                                         std::vector<std::vector<ContactData>> &contacts_mt)
{
    if (rb1->inv_mass_ == 0.0 && rb2->inv_mass_ == 0.0)
        return;

    const double restitution = rb1->restitution_ * rb2->restitution_;
    const double friction = rb1->friction_ + rb2->friction_;

    const Eigen::Vector3d &com2 = rb2->position_;

    const Eigen::Matrix3d &R = rb2->transformation_R_;
    const Eigen::Vector3d &v1 = rb2->transformation_v1_;
    const Eigen::Vector3d &v2 = rb2->transformation_v2_;

    const PointCloudBSH *bvh = co1->bvh_;

    std::function<bool(unsigned int, unsigned int)> predicate = [&](unsigned int node_index, unsigned int depth)
    {
        const BoundingSphere &bs = bvh->hull(node_index);
        const Eigen::Vector3d &sphere_x = bs.x_;
        const Eigen::Vector3d sphere_x_w = rb1->q_.toRotationMatrix() * sphere_x + rb1->x_; // TODO: potential bugs

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
            if (dist2 == std::numeric_limits<double>::max()) // TODO: ??? maybe to delete this in the future
                return true;
            if (dist2 < bs.r_)
                return true;
        }
        return false;
    };

    std::function<void(unsigned int, unsigned int)> cb = [&](unsigned int node_index, unsigned int depth)
    {
        auto const &node = bvh->node(node_index);
        if (!node.is_leaf())
            return;

        for (auto i = node.begin; i < node.begin + node.n; ++i)
        {
            unsigned int index = bvh->entity(i);
            const Eigen::Vector3d &x_w = rb1->q_.toRotationMatrix() * rb1->V_rest_.row(index).transpose() + rb1->x_;
            const Eigen::Vector3d x = R * (x_w - com2) + v1; // local position according to [RigidBody 2]
            Eigen::Vector3d cp{}, n{};
            double dist = 0;
            if (co2->collision_test(x, tolerance_, cp, n, dist, 0.0))
            {
                const Eigen::Vector3d cp_w = R.transpose() * cp + v2;
                const Eigen::Vector3d n_w = R.transpose() * n;

                int tid = 0;
#ifdef USE_OPENMP
#ifdef _DEBUG
                tid = 0;
#else
                int tid = omp_get_thread_num();
#endif
#endif
                contacts_mt[tid].push_back({ContactType::RigidBody, rb1, rb2, x_w, cp_w, n_w, dist, restitution, friction});
            }
        }
    };

    bvh->traverse_depth_first(predicate, cb);
}
