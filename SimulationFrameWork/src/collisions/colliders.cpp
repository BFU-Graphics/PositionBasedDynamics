#include "colliders.h"

#include "src/objects/objects.h"

HINASIM::Collider::Collider(SimObject *o)
{
    object_ = o;
    aabb_ = new AABB(o->V_);
}

HINASIM::Collider::~Collider()
{
    delete aabb_;
}

HINASIM::Collider &HINASIM::Collider::update_aabb()
{
    aabb_->calc_aabb(object_->V_);
    return *this;
}

HINASIM::DistanceFieldCollider::DistanceFieldCollider(HINASIM::SimObject *o, bool inside_collision) : Collider(o), inside_collision_(inside_collision)
{
    bvh_ = new PointCloudBSH();
    bvh_->init(object_->V_);
    bvh_->construct();
}

HINASIM::DistanceFieldCollider::~DistanceFieldCollider()
{
    delete bvh_;
}

HINASIM::DistanceFieldCollider &HINASIM::DistanceFieldCollider::update_bvh()
{
    if (object_->TYPE_ == SimObjectType::RigidBody) // NOTE: NO NEED TO UPDATE
        return *this;

    bvh_->init(object_->V_);
    bvh_->update();

    return *this;
}

HINASIM::SphereColliderDF::SphereColliderDF(HINASIM::SimObject *o, bool inside_collision) : DistanceFieldCollider(o, inside_collision)
{
    const BoundingSphere bs(o->V_);
    radius_ = bs.r_;
}

double HINASIM::SphereColliderDF::distance(const Eigen::Vector3d &x, double tolerance)
{
    const double dist = (x.norm() - radius_);// TODO: recheck this
    return inside_collision_ ? (-dist - tolerance) : (dist - tolerance);
}
#include <iostream>
bool HINASIM::SphereColliderDF::collision_test(const Eigen::Vector3d &x, double tolerance, Eigen::Vector3d &contact_point, Eigen::Vector3d &normal, double &distance, double max_distance)
{
    const Eigen::Vector3d d = x;
    const double dl = d.norm();
    distance = inside_collision_ ? (radius_ - dl - tolerance) : (dl - radius_ - tolerance);

    if (distance < max_distance)
    {
        if (dl < 1.e-6) // NOTE: maybe no use here
            normal.setZero();
        else
            normal = inside_collision_ ? (d / -dl) : (d / dl);
        contact_point = (radius_ + tolerance) * normal;
        return true;
    }

    return false;
}