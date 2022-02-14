/**
 * @author YC XIANG.
 * @date 02/09, 2022
 */

#include "constraints.h"
#include "Utils/pbd_log.h"

pbd_src::DistanceConstraint::DistanceConstraint(Eigen::VectorXd &init_q, const Eigen::MatrixXi &edges)
{
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

bool pbd_src::DistanceConstraint::solve(const Eigen::VectorXd &q, const Eigen::SparseMatrix<double> &M_inv, Eigen::VectorXd &dq, double stiffness)
{
    double tracked_C = 0;
    for (int i = 0; i < Es.size(); ++i)
    {
        Eigen::Vector6d p = Es[i] * q;
        Eigen::Matrix66d M_inv66 = Es[i] * M_inv * Es[i].transpose();

        double C = (B * p).norm() - rest_length(i);
        Eigen::Vector6d dC = (B.transpose() * B * p) / (B * p).norm();
        Eigen::Vector6d dp = -0.6 * stiffness * M_inv66.transpose() * (C * dC) / (dC.transpose() * M_inv66 * dC);
        dq = dq + Es[i].transpose() * dp;

        tracked_C = C;
    }
//    tracked_C = std::sin(simulation_time_);
    record(tracked_C);
    return true;
}
