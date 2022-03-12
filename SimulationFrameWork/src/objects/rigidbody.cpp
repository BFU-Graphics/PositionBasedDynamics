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

HINASIM::RigidBody &HINASIM::RigidBody::set_inv_mass(double inv_mass)
{
    inv_mass_ = inv_mass;
    return *this;
}

void HINASIM::RigidBody::init_physics_states()
{
    x_ = position_;
    v_.setZero();
    a_.setZero();
    t_.setZero();
    Eigen::AngleAxisd roll_angle((rotation_.z() * 3.141592653) / 180, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yaw_angle((rotation_.y() * 3.141592653) / 180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitch_angle((rotation_.x() * 3.141592653) / 180, Eigen::Vector3d::UnitX());
    q_ = roll_angle * yaw_angle * pitch_angle;
    omega_.setZero();

    p_x_.setZero();

    inv_mass_ = 1;
    inertia_tensor_local_.setOnes();
    inertia_tensor_world_.setZero();
    inv_inertia_tensor_world_.setZero();
    inertia_tensor_world_ = (q_.matrix()) * inertia_tensor_local_.asDiagonal() * (q_.matrix()).transpose();
    inv_inertia_tensor_local_.setOnes();
    inv_inertia_tensor_world_ = (q_.matrix()) * inv_inertia_tensor_local_.asDiagonal() * (q_.matrix()).transpose();

    q_mat_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); // TODO:
    q_initial_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); // TODO:
    x0_mat_.setZero(); // TODO:
    transformation_R_ = (q_initial_.inverse() * q_mat_ * q_.inverse()).matrix(); // TODO:
    transformation_v1_ = -q_initial_.inverse().matrix() * x0_mat_; // TODO:
    transformation_v2_ = (q_ * q_mat_.inverse()).matrix() * x0_mat_ + x_; // TODO:

    V_rest_ = Eigen::Map<Eigen::MatrixXd>(V_.data(), V_.rows(), V_.cols());
    for (int i = 0; i < V_.rows(); ++i)
        V_.row(i) = (q_.toRotationMatrix() * V_rest_.row(i).transpose() + x_).transpose();
    V_buffer_.resize(V_rest_.rows(), V_rest_.cols());
    for (int i = 0; i < V_.rows(); ++i)
        V_buffer_.row(i) = (q_.toRotationMatrix() * V_rest_.row(i).transpose() + x_).transpose();

    mouse_drag_force_.resize(V_.rows(), 3);
    mouse_drag_force_.setZero();
}

void HINASIM::RigidBody::update_rendering_info()
{
    V_ = V_buffer_;
}

void HINASIM::RigidBody::update_physics_info()
{
    position_ = x_;
    rotation_ = q_.toRotationMatrix().eulerAngles(0, 1, 2); // FIXME: may be not correct with x,y,z

    if (inv_mass_ > 0.0)
    {
        inertia_tensor_world_ = q_.toRotationMatrix() * inertia_tensor_local_.asDiagonal() * (q_.toRotationMatrix()).transpose();
        inv_inertia_tensor_world_ = q_.toRotationMatrix() * inv_inertia_tensor_local_.asDiagonal() * (q_.toRotationMatrix()).transpose();

        transformation_R_ = (q_initial_.inverse() * q_mat_ * q_.inverse()).matrix(); // TODO:
        transformation_v1_ = -q_initial_.inverse().matrix() * x0_mat_; // TODO:
        transformation_v2_ = (q_ * q_mat_.inverse()).matrix() * x0_mat_ + x_; // TODO:
    }

    for (int i = 0; i < V_.rows(); ++i)
        V_buffer_.row(i) = (p_q_.toRotationMatrix() * V_rest_.row(i).transpose() + p_x_).transpose();
}