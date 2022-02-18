/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "constraints.h"

#ifdef USE_OPENMP

#include "omp.h"

#endif

#include <iostream>


HINASIM::DistanceConstraint::DistanceConstraint(Eigen::MatrixXd &init_q, const Eigen::MatrixXi &edges)
{
    std::clog << "Distance Constraint Sum: " << edges.rows() << std::endl;

    distance_constraints_.reserve(edges.rows());

    for (int i = 0; i < edges.rows(); ++i)
    {
        int q1_index = edges.row(i)(0);
        int q2_index = edges.row(i)(1);
        Eigen::RowVector3d q1 = init_q.row(q1_index);
        Eigen::RowVector3d q2 = init_q.row(q2_index);
        distance_constraints_.emplace_back(std::make_tuple(q1_index, q2_index, (q1 - q2).norm()));
    }
}

bool HINASIM::DistanceConstraint::solve(Eigen::MatrixXd &p, const Eigen::VectorXd &inv_mass, double stiffness)
{
#ifdef USE_OPENMP
    omp_set_num_threads(16);
#pragma omp parallel for
#endif
    for (int i = 0; i < distance_constraints_.size(); ++i)
    {
        auto &dc = distance_constraints_[i];
        int p1_index = std::get<0>(dc);
        int p2_index = std::get<1>(dc);
        double inv_mass_p1 = inv_mass(p1_index);
        double inv_mass_p2 = inv_mass(p2_index);
        double rest_length = std::get<2>(dc);
        Eigen::RowVector3d p1 = p.row(p1_index);
        Eigen::RowVector3d p2 = p.row(p2_index);

        double C = (p1 - p2).norm() - rest_length;
        Eigen::RowVector3d dC = (p1 - p2).normalized().transpose();

        Eigen::RowVector3d dp_1 = -stiffness * (inv_mass_p1) / (inv_mass_p1 + inv_mass_p2) * C * dC;
        Eigen::RowVector3d dp_2 = +stiffness * (inv_mass_p2) / (inv_mass_p1 + inv_mass_p2) * C * dC;

        p.row(p1_index) += dp_1;
        p.row(p2_index) += dp_2;
    }

    return true;
}