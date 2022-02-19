/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "objects.h"

#include "RenderingFrameWork/src/mouse_callback.h"

#include <igl/readOBJ.h>
#include <igl/readPLY.h>

void HINASIM::SimObject::init_geometry(const std::string &path)
{
    if (custom_init_geometry != nullptr)
    {
        custom_init_geometry(V_, F_);
        return;
    }
    std::string model_name = path.substr(path.find_last_of('/') + 1);

    std::string model_type = path.substr(path.find_last_of('.') + 1);
    if (model_type.empty())
        return;
    model_type = model_type.substr(0, 3);
    if (model_type == "obj")
        igl::readOBJ(path, V_, F_);
    else if (model_type == "ply")
        igl::readPLY(path, V_, F_);

    build_neighbors(); // Edges info would be auto build ourselves

    if (V_.rows() > 0)
        init_physical_state();

    std::clog << "Model Name: " << model_name << "     Vertex Sum: " << V_.rows() << "     Face Sum: " << F_.rows() << std::endl;
}

void HINASIM::DeformableObject::init_physical_state()
{
    q_.resize(V_.rows(), V_.cols());
    qdot_.resize(V_.rows(), V_.cols());
    Eigen::MatrixXd Vt = V_;
    q_ = Eigen::Map<Eigen::MatrixXd>(V_.data(), V_.rows(), V_.cols());
    qdot_.setZero();

    p_.resize(q_.rows(), q_.cols());

    mouse_drag_force_.resize(V_.rows(), 3);
    mouse_drag_force_.setZero();

    inv_mass_.resize(V_.rows());
    inv_mass_.setOnes();
}

void HINASIM::DeformableObject::update_geometry_info()
{
    V_ = q_;
}

void HINASIM::DeformableObject::update_mouse_drag()
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

HINASIM::SimObject &HINASIM::SimObject::add_constraint(HINASIM::Constraint *constraint)
{
    inner_constraints_.emplace_back(constraint);
    return *this;
}

HINASIM::SimObject &HINASIM::SimObject::set_inv_mass(double unified_inv_mass)
{
    inv_mass_.setConstant(unified_inv_mass);

    return *this;
}

HINASIM::SimObject &HINASIM::SimObject::set_inv_mass(int index, double inv_mass)
{
    inv_mass_(index) = inv_mass;

    return *this;
}

HINASIM::Cloth::Cloth(int rows, int cols, int width, int height) : rows_(rows), cols_(cols), width_(width), height_(height)
{
    custom_init_geometry = [&](Eigen::MatrixXd &V_, Eigen::MatrixXi &F_)
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
    };
}

void HINASIM::Cloth::init_geometry()
{
    assert(custom_init_geometry != nullptr);

    custom_init_geometry(V_, F_);

    build_neighbors(); // Edges info would be auto build ourselves

    if (V_.rows() > 0)
        init_physical_state();

    std::clog << "Cloth Info: " << "Vertex: " << V_.rows() << " Faces: " << F_.rows() << std::endl;
}

void HINASIM::RigidBody::init_physical_state()
{

}

void HINASIM::RigidBody::update_geometry_info()
{

}

void HINASIM::RigidBody::update_mouse_drag()
{

}
