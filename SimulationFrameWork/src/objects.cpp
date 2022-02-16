/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "objects.h"

#include <igl/edges.h>
#include <igl/readOBJ.h>
#include <igl/readPLY.h>

void HINASIM::SimObject::init_geometry(const std::string &path)
{
    if (custom_init_geometry != nullptr)
    {
        custom_init_geometry(V_, F_);
        return;
    }

    igl::readOBJ(path, V_, F_);
    igl::edges(F_, E_);

    if (V_.rows() > 0)
        init_physical_state();
}

void HINASIM::DeformableObject::init_physical_state()
{
    q_.resize(V_.rows() * V_.cols());
    qdot_.resize(V_.rows() * V_.cols());
    Eigen::MatrixXd Vt = V_.transpose();
    q_ = Eigen::Map<Eigen::VectorXd>(Vt.data(), Vt.rows() * Vt.cols());
    qdot_.setZero();

    mass_.resize(V_.rows());
    mass_.setOnes();
}

void HINASIM::DeformableObject::update_geometry_info()
{
    for (int i = 0; i < V_.rows(); ++i)
    {
        V_.row(i) = q_.segment<3>(3 * i).transpose();
    }
}

HINASIM::SimObject &HINASIM::SimObject::add_constraint(HINASIM::Constraint *constraint)
{
    inner_constraints_.emplace_back(constraint);
    return *this;
}

HINASIM::SimObject &HINASIM::SimObject::set_inv_mass(double unified_mass)
{
    mass_.setConstant(unified_mass);
    return *this;
}

HINASIM::SimObject &HINASIM::SimObject::set_inv_mass(int index, double mass)
{
    mass_(index) = mass;
    return *this;
}

HINASIM::Cloth::Cloth(int rows, int cols, int width, int height)
{
    custom_init_geometry = [](Eigen::MatrixXd &V_, Eigen::MatrixXi &F_)
    {

    };
}
