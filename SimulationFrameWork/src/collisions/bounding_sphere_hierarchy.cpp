#include "bounding_sphere_hierarchy.h"

HINASIM::PointCloudBSH::PointCloudBSH() : KDTree<BoundingSphere>(0, 10)
{}

void HINASIM::PointCloudBSH::init(const Eigen::MatrixXd &V)
{
    list_.resize(V.rows());
    V_ = V;
}

Eigen::Vector3d HINASIM::PointCloudBSH::entity_position(unsigned int i) const
{
    return V_.row(i).transpose();
}

void HINASIM::PointCloudBSH::compute_hull(unsigned int b, unsigned int n, HINASIM::BoundingSphere &hull) const
{
    Eigen::MatrixXd sub_vertices;
    sub_vertices.resize(n, 3);

    for (unsigned int i = n; i < b + n; ++i)
        sub_vertices.row(i - b) = V_.row(list_[i]);

    const BoundingSphere s(sub_vertices);

    hull.x_ = s.x_;
    hull.r_ = s.r_;
}

void HINASIM::PointCloudBSH::compute_hull_approx(unsigned int b, unsigned int n, HINASIM::BoundingSphere &hull) const
{
    // compute center
    Eigen::Vector3d x;
    x.setZero();
    for (unsigned int i = b; i < b + n; i++)
        x += V_.row(list_[i]).transpose();
    x /= (double) n;

    double radius2 = 0.0;
    for (unsigned int i = b; i < b + n; i++)
        radius2 = std::max(radius2, (x - V_.row(list_[i]).transpose()).squaredNorm());

    hull.x_ = x;
    hull.r_ = sqrt(radius2);
}