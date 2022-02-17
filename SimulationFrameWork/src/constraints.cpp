/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "constraints.h"

#ifdef USE_OPENMP
#include "omp.h"
#endif

#include <iostream>
HINASIM::DistanceConstraint::DistanceConstraint(Eigen::VectorXd &init_q, const Eigen::MatrixXi &edges)
{
    std::clog << "Distance Constraint Sum: " << edges.rows() << std::endl;

    /**
     * Replace all content below at your will
     */

    typedef Eigen::Triplet<double> T;
    std::vector<T> tl_E;

    auto n_q = init_q.rows();
    auto n_r = edges.rows();
    Es.resize(n_r);
    rest_length.resize(n_r);
    tl_E.reserve(6);

    for (int i = 0; i < n_r; ++i)
    {
        rest_length(i) = (init_q.segment(3 * edges(i, 0), 3) - init_q.segment(3 * edges(i, 1), 3)).norm();
        Es[i].resize(6, n_q);
        tl_E.emplace_back(0, 3 * edges(i, 0) + 0, 1);
        tl_E.emplace_back(1, 3 * edges(i, 0) + 1, 1);
        tl_E.emplace_back(2, 3 * edges(i, 0) + 2, 1);
        tl_E.emplace_back(3, 3 * edges(i, 1) + 0, 1);
        tl_E.emplace_back(4, 3 * edges(i, 1) + 1, 1);
        tl_E.emplace_back(5, 3 * edges(i, 1) + 2, 1);
        Es[i].setFromTriplets(tl_E.begin(), tl_E.end());
        tl_E.clear();
    }

    // B = [I -I]
    B << 1, 0, 0, -1, 0, 0
            , 0, 1, 0, 0, -1, 0
            , 0, 0, 1, 0, 0, -1;
}

bool HINASIM::DistanceConstraint::solve(Eigen::VectorXd &q, const Eigen::SparseMatrix<double> &M_inv, double stiffness)
{
    double tracked_C = 0;
    double sin_function = 0;

    /**
     * Replace all content below at your will
     */


    /// If you want to enable OpenMP CPU Multi-Thread Acceleration, turn on [OPTION (USE_OpenMP "Use OpenMP" OFF)] in root CmakeLists.txt, and reset cmake cache
#ifdef USE_OPENMP
    omp_set_num_threads(8);
#pragma omp parallel for
#endif
    for (int i = 0; i < Es.size(); ++i)
    {
        Eigen::Vector6d p = Es[i] * q;
        Eigen::Matrix66d M_inv66 = Es[i] * M_inv * Es[i].transpose();
        double C = (B * p).norm() - rest_length(i);
        Eigen::Vector6d dC = (B.transpose() * B * p) / (B * p).norm();
        Eigen::Vector6d dp = -stiffness * M_inv66.transpose() * (C * dC) / (dC.transpose() * M_inv66 * dC);
        q += Es[i].transpose() * dp;
        tracked_C = C;
    }

    sin_function = std::sin(HINAVIEWER::INSPECTOR::Timeable::simulation_time_);

    record({tracked_C, sin_function}, {"Distance Constraint - C", "plot sin(time)"});

    return true;
}
