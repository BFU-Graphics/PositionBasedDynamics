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
    double length_x = std::abs(aabb_->aabb_[1].x() - aabb_->aabb_[0].x());
    double length_y = std::abs(aabb_->aabb_[1].y() - aabb_->aabb_[0].y());
    double length_z = std::abs(aabb_->aabb_[1].z() - aabb_->aabb_[0].z());
    radius_ = std::max({length_x, length_y, length_z});
}

double HINASIM::SphereColliderDF::distance(const Eigen::Vector3d &x, double tolerance)
{
    const double dist = (x.norm() - radius_);// TODO: recheck this
    return inside_collision_ ? (-dist - tolerance) : (dist - tolerance);
}
