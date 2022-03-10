#include "rigidbody.h"

HINASIM::RigidBody::RigidBody(Eigen::Vector3d position, Eigen::Vector3d rotation, Eigen::Vector3d scale, double density) : SimObject(std::move(position), std::move(rotation), std::move(scale))
{
    TYPE_ = HINASIM::SimObjectType::RigidBody;
}

HINASIM::RigidBody::RigidBody(const std::string &path, Eigen::Vector3d position, Eigen::Vector3d rotation, Eigen::Vector3d scale, double density) : SimObject(path, std::move(position), std::move(rotation), std::move(scale))
{
    TYPE_ = HINASIM::SimObjectType::RigidBody;
}

HINASIM::RigidBody::RigidBody(const std::function<void(Eigen::MatrixXd &, Eigen::MatrixXi &)> &custom_init_geometry, Eigen::Vector3d position, Eigen::Vector3d rotation, Eigen::Vector3d scale, double density) : SimObject(
        custom_init_geometry,
        std::move(position), std::move(rotation), std::move(scale))
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
    Eigen::AngleAxisd roll_angle((rotation_.z() * M_PI) / 180, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yaw_angle((rotation_.y() * M_PI) / 180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitch_angle((rotation_.x() * M_PI) / 180, Eigen::Vector3d::UnitX());
    q_ = roll_angle * yaw_angle * pitch_angle;
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
    rotation_ = q_.toRotationMatrix().eulerAngles(0, 1, 2); // FIXME: may be not correct with x,y,z

    if (inv_mass_ > 0.0)
    {
        inertia_tensor_world_ = q_.toRotationMatrix() * inertia_tensor_local_.asDiagonal() * (q_.toRotationMatrix()).transpose();
        inv_inertia_tensor_world_ = q_.toRotationMatrix() * inv_inertia_tensor_local_.asDiagonal() * (q_.toRotationMatrix()).transpose();
    }
}
