/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "objects.h"

#include "cmake-build-release/_deps/libigl-src/include/igl/edges.h"
#include "cmake-build-release/_deps/libigl-src/include/igl/readOBJ.h"
#include "cmake-build-release/_deps/libigl-src/include/igl/readPLY.h"

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

    igl::edges(F_, E_);

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

    inv_mass_.resize(V_.rows());
    inv_mass_.setOnes();
}

void HINASIM::DeformableObject::update_geometry_info()
{
    V_ = q_;
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

HINASIM::Cloth::Cloth(int rows, int cols, int width, int height)
{
    custom_init_geometry = [&](Eigen::MatrixXd &V_, Eigen::MatrixXi &F_)
    {
        const double dx = width / static_cast<double>(cols - 1);
        const double dy = height / static_cast<double>(rows - 1);

        V_.resize(rows * cols, 3);
        F_.resize(2 * (rows - 1) * (cols - 1), 3);

        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                const auto x = static_cast<double>(dx * j);
                const auto y = static_cast<double>(dy * i);

                V_.row(i * cols + j) = Eigen::RowVector3d(x, 0.0, y);
            }
        }

        for (int i = 0; i < rows - 1; ++i)
        {
            for (int j = 0; j < cols - 1; ++j)
            {
                if (i % 2 == j % 2)
                {
                    F_.row(2 * (i * (cols - 1) + j)) = Eigen::RowVector3i(i * cols + j, (i + 1) * cols + j + 1, i * cols + j + 1);
                    F_.row(2 * (i * (cols - 1) + j) + 1) = Eigen::RowVector3i(i * cols + j, (i + 1) * cols + j, (i + 1) * cols + j + 1);
                } else
                {
                    F_.row(2 * (i * (cols - 1) + j)) = Eigen::RowVector3i(i * cols + j, (i + 1) * cols + j, i * cols + j + 1);
                    F_.row(2 * (i * (cols - 1) + j) + 1) = Eigen::RowVector3i(i * cols + j + 1, (i + 1) * cols + j, (i + 1) * cols + j + 1);
                }
            }
        }
    };
}

void HINASIM::Cloth::init_geometry()
{
    assert(custom_init_geometry != nullptr);

    custom_init_geometry(V_, F_);

    igl::edges(F_, E_);

    if (V_.rows() > 0)
        init_physical_state();

    std::clog << "Cloth Info: " << V_.rows() << " / " << F_.rows() << std::endl;
}
