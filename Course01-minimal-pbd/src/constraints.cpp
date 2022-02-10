/**
 * @author YC XIANG.
 * @date 02/09, 2022
 */

#include "constraints.h"
#include "Utils/pbd_log.h"
#include <iostream>

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
    for (int i = 0; i < Es.size(); ++i)
    {
        Eigen::Vector6d p = Es[i] * q;
        Eigen::Matrix66d M_inv66 = Es[i] * M_inv * Es[i].transpose();
//        Eigen::Matrix3d M1_inv = M_inv66.block<3, 3>(0, 0);
//        Eigen::Matrix3d M2_inv = M_inv66.block<3, 3>(3, 3);

        double C = (B * p).norm() - rest_length(i);
        Eigen::Vector6d dC = (B.transpose() * B * p) / (B * p).norm();
        Eigen::Vector6d dp = -0.2 * stiffness * M_inv66.transpose() * (C * dC) / (dC.transpose() * M_inv66 * dC);
//        if (C > 2)
//        {
//            pbd_util::log(p, "Distance Constraint", "p", true);
//            pbd_util::log(C, "Distance Constraint", "C", true);
//            pbd_util::log(dC, "Distance Constraint", "dC", true);
//            pbd_util::log(dp, "Distance Constraint", "dp", true);
//            pbd_util::log(Es[i], "Distance Constraint", "Es", true);
//            pbd_util::log(M_inv66, "Distance Constraint", "M_inv66", true);
//        }
        dq = dq + Es[i].transpose() * dp;

    }
    return true;
}