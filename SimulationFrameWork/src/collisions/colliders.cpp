#include "colliders.h"

#include "src/objects/objects.h"

HINASIM::Colliders::Colliders(SimObject *o)
{
    object_ = o;
    aabb_ = new AABB(o->V_);
}

HINASIM::Colliders::~Colliders()
{
    delete aabb_;
}

HINASIM::Colliders &HINASIM::Colliders::update_aabb()
{
    aabb_->calc_aabb(object_->V_);
    return *this;
}

HINASIM::DistanceFieldCollider::DistanceFieldCollider(HINASIM::SimObject *o, bool inside_collision) : Colliders(o), inside_collision_(inside_collision)
{
    bvh_ = new PointCloudBSH();
    bvh_->init(o->V_);
    bvh_->construct();
}

HINASIM::DistanceFieldCollider::~DistanceFieldCollider()
{
    delete bvh_;
}

HINASIM::SphereColliderDF::SphereColliderDF(HINASIM::SimObject *o, bool inside_collision) : DistanceFieldCollider(o, inside_collision)
{

}

double HINASIM::SphereColliderDF::distance(const Eigen::Vector3d &x, double tolerance)
{
    const double dist = (x.norm() - radius_);// TODO: recheck this
    return inside_collision_ ? (-dist - tolerance) : (dist - tolerance);
}
