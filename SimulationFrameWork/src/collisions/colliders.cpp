#include "colliders.h"

#include "src/objects/rigidbody.h"

HINASIM::Collider::Collider(SimObject *o)
{
    object_ = o;
    if (object_->TYPE_ == SimObjectType::RigidBody)
        aabb_ = new AABB(dynamic_cast<RigidBody *>(object_)->V_buffer_);
}

HINASIM::Collider::~Collider()
{
    delete aabb_;
}

HINASIM::Collider &HINASIM::Collider::update_aabb()
{
    if (object_->TYPE_ == SimObjectType::RigidBody)
        aabb_->calc_aabb(dynamic_cast<RigidBody *>(object_)->V_buffer_);


    std::cout << "======" << std::endl;
    std::cout << aabb_->aabb_[0] << std::endl;
    std::cout << aabb_->aabb_[1] << std::endl;
    std::cout << "======" << std::endl;
    return *this;
}

HINASIM::DistanceFieldCollider::DistanceFieldCollider(HINASIM::SimObject *o, bool inside_collision) : Collider(o), inside_collision_(inside_collision)
{
    bvh_ = new PointCloudBSH();
    if (object_->TYPE_ == SimObjectType::RigidBody)
        bvh_->init(dynamic_cast<RigidBody *>(object_)->V_rest_);
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

bool HINASIM::DistanceFieldCollider::collision_test(const Eigen::Vector3d &x, double tolerance, Eigen::Vector3d &contact_point, Eigen::Vector3d &normal, double &dist, double max_distance)
{
    dist = distance(x, tolerance);
    if (dist < max_distance)
    {
        // approximate gradient
        approximate_normal(x, tolerance, normal);

        contact_point = (x - dist * normal);
        return true;
    }
    return false;
}

void HINASIM::DistanceFieldCollider::approximate_normal(const Eigen::Vector3d &x, double tolerance, Eigen::Vector3d &normal)
{
    // approximate gradient
    double eps = 1.e-6;
    normal.setZero();
    Eigen::Vector3d xTmp(x.x(), x.y(), x.z());

    for (int i = 0; i < 3; ++i)
    {
        xTmp[i] += eps;

        double e_p, e_m;
        e_p = distance(xTmp, tolerance);
        xTmp[i] = x[i] - eps;
        e_m = distance(xTmp, tolerance);
        xTmp[i] = x[i];

        normal[i] = (e_p - e_m) * (1.0 / (2.0 * eps));
    }
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

HINASIM::BoxColliderDF::BoxColliderDF(HINASIM::SimObject *o, bool inside_collision) : DistanceFieldCollider(o, inside_collision)
{
    half_extent_.x() = (aabb_->aabb_[1].x() - aabb_->aabb_[0].x()) / 2.0;
    half_extent_.y() = (aabb_->aabb_[1].y() - aabb_->aabb_[0].y()) / 2.0;
    half_extent_.z() = (aabb_->aabb_[1].z() - aabb_->aabb_[0].z()) / 2.0;
}

double HINASIM::BoxColliderDF::distance(const Eigen::Vector3d &x, double tolerance)
{
    const Eigen::Vector3d d(fabs(x.x()) - half_extent_.x(), fabs(x.y()) - half_extent_.y(), fabs(x.z()) - half_extent_.z());
    const Eigen::Vector3d max_d(std::max(d.x(), 0.0), std::max(d.y(), 0.0), std::max(d.z(), 0.0));
    return inside_collision_ ? (-(std::min(std::max(d.x(), std::max(d.y(), d.z())), 0.0) + max_d.norm()) - tolerance) : ((std::min(std::max(d.x(), std::max(d.y(), d.z())), 0.0) + max_d.norm()) - tolerance);
}
