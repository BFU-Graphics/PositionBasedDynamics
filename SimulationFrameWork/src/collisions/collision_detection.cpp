#include "collision_detection.h"

#include "src/objects/rigidbody.h"

HINASIM::CollisionDetection::CollisionDetection()
{
    colliders_.reserve(1000);
    tolerance_ = 0.01;
    contacts_.emplace_back(new RigidBodyContactConstraint());
}

HINASIM::CollisionDetection::~CollisionDetection()
{
    for (auto &collider: colliders_)
        delete collider;
    colliders_.clear();

    for (auto &contact: contacts_)
        delete contact;
    contacts_.clear();
}

void HINASIM::CollisionDetection::add_rigid_body_contact(HINASIM::RigidBody *rb1, HINASIM::RigidBody *rb2, const Eigen::Vector3d &contact_point1, const Eigen::Vector3d &contact_point2, const Eigen::Vector3d &contact_normal,
                                                         double contact_distance, double restitution, double friction)
{
    dynamic_cast<RigidBodyContactConstraint *>(contacts_[0])->add_contact(rb1, rb2, contact_point1, contact_point2, contact_normal, contact_distance, restitution, friction);
}

void HINASIM::CollisionDetection::contacts_solve()
{
    for (auto &contact: contacts_)
        contact->solve();
}
