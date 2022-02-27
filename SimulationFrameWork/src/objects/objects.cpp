#include "objects.h"

#include <utility>

HINASIM::SimObject::SimObject(Eigen::Vector3d position, const Eigen::Quaterniond &rotation) : position_(std::move(position)), rotation_(rotation)
{
    TYPE_ = HINASIM::SimObjectType::NoType;
}

HINASIM::SimObject::SimObject(const std::string &path, Eigen::Vector3d position, const Eigen::Quaterniond &rotation) : position_(std::move(position)), rotation_(rotation)
{
    TYPE_ = HINASIM::SimObjectType::NoType;

    load_model_from_path(path);

    init_rendering_info();
}

HINASIM::SimObject::SimObject(const std::function<void(Eigen::MatrixXd &V, Eigen::MatrixXi &F)> &custom_init_geometry, Eigen::Vector3d position, const Eigen::Quaterniond &rotation) : position_(std::move(position)), rotation_(rotation)
{
    TYPE_ = HINASIM::SimObjectType::NoType;

    custom_init_geometry(V_, F_);

    init_rendering_info();
}

void HINASIM::SimObject::init_rendering_info()
{
    assert(V_.rows() > 0);
    assert(F_.rows() > 0);

    build_neighbors();
}
