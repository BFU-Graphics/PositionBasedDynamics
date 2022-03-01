#include "fields3.h"

Eigen::Vector3d HINASIM::UTILS::ScalarField3::gradient(const Eigen::Vector3d &x) const
{
    return {};
}

double HINASIM::UTILS::ScalarField3::laplacian(const Eigen::Vector3d &x) const
{
    return 0.0;
}

std::function<double(const Eigen::Vector3d &)> HINASIM::UTILS::ScalarField3::sampler() const
{
    const ScalarField3 *self = this;
    return [self](const Eigen::Vector3d &x) -> double
    {
        return self->sample(x);
    };
}

double HINASIM::UTILS::VectorField3::divergence(const Eigen::Vector3d &x) const
{
    return 0.0;
}

Eigen::Vector3d HINASIM::UTILS::VectorField3::curl(const Eigen::Vector3d &x) const
{
    return {};
}

std::function<Eigen::Vector3d(const Eigen::Vector3d &)> HINASIM::UTILS::VectorField3::sampler() const
{
    const VectorField3 *self = this;
    return [self](const Eigen::Vector3d &x) -> Eigen::Vector3d
    {
        return self->sample(x);
    };
}
