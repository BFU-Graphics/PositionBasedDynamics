#include "rigidbody.h"

HINASIM::RigidBody::RigidBody(Eigen::Vector3d position, const Eigen::Quaterniond &rotation) : SimObject(std::move(position), rotation)
{
    TYPE_ = HINASIM::SimObjectType::RigidBody;
}

HINASIM::RigidBody::RigidBody(const std::string &path, Eigen::Vector3d position, const Eigen::Quaterniond &rotation) : SimObject(path, std::move(position), rotation)
{
    TYPE_ = HINASIM::SimObjectType::RigidBody;
}

HINASIM::RigidBody::RigidBody(const std::function<void(Eigen::MatrixXd &, Eigen::MatrixXi &)> &custom_init_geometry, Eigen::Vector3d position, const Eigen::Quaterniond &rotation) : SimObject(custom_init_geometry,
                                                                                                                                                                                               std::move(position), rotation)
{
    TYPE_ = HINASIM::SimObjectType::RigidBody;
}
#include <iostream>
void HINASIM::RigidBody::init_physics_states()
{
    x_ = position_;
    v_.setZero();
    a_.setZero();
    t_.setZero();
    q_ = rotation_;
    omega_.setZero();

    inv_mass_ = 1;
    inertia_tensor_local_.setOnes();
    inertia_tensor_world_.setZero();
    inv_inertia_tensor_world_.setZero();
    inertia_tensor_world_ = (q_.matrix()) * inertia_tensor_local_.asDiagonal() * (q_.matrix()).transpose();
    inv_inertia_tensor_local_.setOnes();
    inv_inertia_tensor_world_ = (q_.matrix()) * inv_inertia_tensor_local_.asDiagonal() * (q_.matrix()).transpose();

    V_rest_ = V_;

    mouse_drag_force_.resize(V_.rows(), 3);
    mouse_drag_force_.setZero();
}

void HINASIM::RigidBody::update_rendering_info()
{

}

void HINASIM::RigidBody::update_physics_info()
{
    position_ = x_;

    inertia_tensor_world_ = (q_.matrix()) * inertia_tensor_local_.asDiagonal() * (q_.matrix()).transpose();
    inv_inertia_tensor_world_ = (q_.matrix()) * inv_inertia_tensor_local_.asDiagonal() * (q_.matrix()).transpose();
}
