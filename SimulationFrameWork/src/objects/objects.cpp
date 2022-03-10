#include "objects.h"

#include <utility>

HINASIM::SimObject::SimObject(Eigen::Vector3d position, Eigen::Vector3d rotation, Eigen::Vector3d scale) : position_(std::move(position)), rotation_(std::move(rotation)), scale_(std::move(scale))
{
    TYPE_ = HINASIM::SimObjectType::NoType;
}

HINASIM::SimObject::SimObject(const std::string &path, Eigen::Vector3d position, Eigen::Vector3d rotation, Eigen::Vector3d scale) : position_(std::move(position)), rotation_(std::move(rotation)), scale_(std::move(scale))
{
    TYPE_ = HINASIM::SimObjectType::NoType;

    load_model_from_path(path);

    init_rendering_info();
}

HINASIM::SimObject::SimObject(const std::function<void(Eigen::MatrixXd &V, Eigen::MatrixXi &F)> &custom_init_geometry, Eigen::Vector3d position, Eigen::Vector3d rotation, Eigen::Vector3d scale) : position_(std::move(position)),
                                                                                                                                                                                                    rotation_(std::move(rotation)),
                                                                                                                                                                                                    scale_(std::move(scale))
{
    TYPE_ = HINASIM::SimObjectType::NoType;

    custom_init_geometry(V_, F_);

    init_rendering_info();
}

void HINASIM::SimObject::init_rendering_info()
{
    assert(V_.rows() > 0);
    assert(F_.rows() > 0);

    // do mesh scaling
    for (int i = 0; i < V_.size(); ++i)
    {
        V_.row(i)(0) *= scale_(0);
        V_.row(i)(1) *= scale_(1);
        V_.row(i)(2) *= scale_(2);
    }

    build_neighbors();
}
