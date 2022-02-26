#include "deformable.h"

HINASIM::DeformableObject::DeformableObject(Eigen::Vector3d position, const Eigen::Quaterniond &rotation) : SimObject(std::move(position), rotation)
{
    TYPE_ = HINASIM::SimObjectType::Deformable;
}

HINASIM::DeformableObject::DeformableObject(const std::string &path, Eigen::Vector3d position, const Eigen::Quaterniond &rotation) : SimObject(path, std::move(position), rotation)
{
    TYPE_ = HINASIM::SimObjectType::Deformable;
}

HINASIM::DeformableObject::DeformableObject(const std::function<void(Eigen::MatrixXd &, Eigen::MatrixXi &)> &custom_init_geometry, Eigen::Vector3d position, const Eigen::Quaterniond &rotation) : SimObject(custom_init_geometry,
                                                                                                                                                                                                             std::move(position), rotation)
{
    TYPE_ = HINASIM::SimObjectType::Deformable;
}

HINASIM::DeformableObject &HINASIM::DeformableObject::add_constraint(HINASIM::Constraint *constraint)
{
    inner_constraints_.emplace_back(constraint);
    return *this;
}

HINASIM::DeformableObject &HINASIM::DeformableObject::set_inv_mass(double unified_inv_mass)
{
    inv_mass_.setConstant(unified_inv_mass);
    return *this;
}

HINASIM::DeformableObject &HINASIM::DeformableObject::set_inv_mass(int index, double inv_mass)
{
    inv_mass_(index) = inv_mass;
    return *this;
}

#include <iostream>

void HINASIM::DeformableObject::init_physics_states()
{
    x_.resize(V_.rows(), V_.cols());
    v_.resize(V_.rows(), V_.cols());
    a_.resize(V_.rows(), V_.cols());
    p_.resize(x_.rows(), x_.cols());

    x_ = Eigen::Map<Eigen::MatrixXd>(V_.data(), V_.rows(), V_.cols());
    v_.setZero();
    a_.setZero();
    p_.setZero();

    inv_mass_.resize(V_.rows());
    inv_mass_.setOnes();

    mouse_drag_force_.resize(V_.rows(), 3);
    mouse_drag_force_.setZero();
}

void HINASIM::DeformableObject::update_rendering_info()
{
    V_ = x_;
}

void HINASIM::DeformableObject::update_mouse_drag_force()
{
    for (auto &v_index: HINAVIEWER::MOUSE_CALLBACK::picked_vertices())
    {
        double stiffness = HINAVIEWER::MOUSE_CALLBACK::is_mouse_dragging() ? 1e4 : 0;
        Eigen::Vector3d mouse_drag_delta = HINAVIEWER::MOUSE_CALLBACK::mouse_drag_world() + Eigen::Vector3d::Constant(1e-6);
        Eigen::Vector3d mouse_drag_force = stiffness * mouse_drag_delta;

        mouse_drag_force_.row(v_index) += mouse_drag_force.transpose();
    }

    if (HINAVIEWER::MOUSE_CALLBACK::picked_vertices().empty())
        mouse_drag_force_.setZero();
}

HINASIM::Cloth::Cloth(int rows, int cols, int width, int height, Eigen::Vector3d position, const Eigen::Quaterniond &rotation) : rows_(rows), cols_(cols), width_(width), height_(height), DeformableObject(std::move(position), rotation)
{
    const double dx = width_ / static_cast<double>(cols_ - 1);
    const double dy = height_ / static_cast<double>(rows_ - 1);

    V_.resize(rows_ * cols_, 3);
    F_.resize(2 * (rows_ - 1) * (cols_ - 1), 3);

    for (int i = 0; i < rows_; ++i)
    {
        for (int j = 0; j < cols_; ++j)
        {
            const auto x = static_cast<double>(dx * j);
            const auto y = static_cast<double>(dy * i);

            V_.row(i * cols_ + j) = Eigen::RowVector3d(x, 0.0, y);
        }
    }

    for (int i = 0; i < rows_ - 1; ++i)
    {
        for (int j = 0; j < cols_ - 1; ++j)
        {
            if (i % 2 == j % 2)
            {
                F_.row(2 * (i * (cols_ - 1) + j)) = Eigen::RowVector3i(i * cols_ + j, (i + 1) * cols_ + j + 1, i * cols_ + j + 1);
                F_.row(2 * (i * (cols_ - 1) + j) + 1) = Eigen::RowVector3i(i * cols_ + j, (i + 1) * cols_ + j, (i + 1) * cols_ + j + 1);
            } else
            {
                F_.row(2 * (i * (cols_ - 1) + j)) = Eigen::RowVector3i(i * cols_ + j, (i + 1) * cols_ + j, i * cols_ + j + 1);
                F_.row(2 * (i * (cols_ - 1) + j) + 1) = Eigen::RowVector3i(i * cols_ + j + 1, (i + 1) * cols_ + j, (i + 1) * cols_ + j + 1);
            }
        }
    }

    init_rendering_info();

    std::clog << "Cloth Info: " << "Vertex: " << V_.rows() << " Faces: " << F_.rows() << std::endl;
}