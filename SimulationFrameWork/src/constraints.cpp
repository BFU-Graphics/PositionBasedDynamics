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
    distance_constraints_.reserve(edges.rows());

    for (int i = 0; i < edges.rows(); ++i)
    {
        int q1_index = edges.row(i)(0);
        int q2_index = edges.row(i)(1);
        Eigen::RowVector3d q1 = init_q.row(q1_index);
        Eigen::RowVector3d q2 = init_q.row(q2_index);
        distance_constraints_.emplace_back(std::make_tuple(q1_index, q2_index, (q1 - q2).norm()));
    }
    std::clog << "Distance Constraint Sum: " << distance_constraints_.size() << std::endl;

}

bool HINASIM::DistanceConstraint::solve(Eigen::MatrixXd &p, const Eigen::VectorXd &inv_mass)
{
#ifdef USE_OPENMP
    omp_set_num_threads(16);
#pragma omp parallel for
#endif
    for (int i = 0; i < distance_constraints_.size(); ++i) // do not use for range form to avoid OpenMP bugs
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

        Eigen::RowVector3d dp_1 = -stiffness_ * (inv_mass_p1) / (inv_mass_p1 + inv_mass_p2) * C * dC;
        Eigen::RowVector3d dp_2 = +stiffness_ * (inv_mass_p2) / (inv_mass_p1 + inv_mass_p2) * C * dC;

        p.row(p1_index) += dp_1;
        p.row(p2_index) += dp_2;
    }

    return true;
}

HINASIM::DihedralConstraint::DihedralConstraint(Eigen::MatrixXd &init_q, const std::vector<std::vector<unsigned int>> &EVF, const Eigen::MatrixXi &F)
{
    for (auto &evf: EVF)
    {
        const unsigned int vertex_index_1 = evf[0];
        const unsigned int vertex_index_2 = evf[1];
        const unsigned int face_index_1 = evf[2];
        const unsigned int face_index_2 = evf[3];

        if ((face_index_1 != 0xffffffff) && (face_index_2 != 0xffffffff))
        {
            unsigned int side_index1 = -1;
            unsigned int side_index2 = -1;
            for (int i = 0; i < 3; ++i)
            {
                if ((F.row(face_index_1)(i) != vertex_index_1) && (F.row(face_index_1)(i) != vertex_index_2))
                {
                    side_index1 = F.row(face_index_1)(i);
                    break;
                }
            }
            for (int i = 0; i < 3; ++i)
            {
                if ((F.row(face_index_2)(i) != vertex_index_1) && (F.row(face_index_2)(i) != vertex_index_2))
                {
                    side_index2 = F.row(face_index_2)(i);
                    break;
                }
            }
            if ((side_index1 != -1) && (side_index2 != -1))
            {
                Eigen::Vector3d axis_p1 = init_q.row(vertex_index_1).transpose();
                Eigen::Vector3d axis_p2 = init_q.row(vertex_index_2).transpose();
                Eigen::Vector3d side_p3 = init_q.row(side_index1).transpose();;
                Eigen::Vector3d side_p4 = init_q.row(side_index2).transpose();;

                double elen = (axis_p1 - axis_p2).norm();
                if (elen > 1e-6) // manage bad model condition
                {
                    double inv_elen = 1.0 / elen;

                    Eigen::Vector3d n1 = (axis_p1 - side_p3).cross(axis_p2 - side_p3);
                    n1 /= n1.squaredNorm();
                    n1.normalize();
                    Eigen::Vector3d n2 = (axis_p2 - side_p4).cross(axis_p1 - side_p4);
                    n2 /= n2.squaredNorm();
                    n2.normalize();

                    double dot = n1.dot(n2);
                    dot = std::clamp(dot, -1.0, 1.0);// restrict value to [-1.0, 1.0]

                    double rest_angle = acos(dot);

                    dihedral_constraints_.emplace_back(std::make_tuple(vertex_index_1, vertex_index_2, side_index1, side_index2, rest_angle));
                }
            }
        }
    }

    std::clog << "Dihedral Constraint Sum: " << dihedral_constraints_.size() << std::endl;
}

bool HINASIM::DihedralConstraint::solve(Eigen::MatrixXd &p, const Eigen::VectorXd &inv_mass)
{
#ifdef USE_OPENMP
    omp_set_num_threads(16);
#pragma omp parallel for
#endif
    for (int i = 0; i < dihedral_constraints_.size(); ++i) // do not use for range form to avoid OpenMP bugs
    {
        auto &dc = dihedral_constraints_[i];

        int axis_p1_index = std::get<0>(dc);
        int axis_p2_index = std::get<1>(dc);
        int side_p3_index = std::get<2>(dc);
        int side_p4_index = std::get<3>(dc);
        double rest_angle = std::get<4>(dc);

        if (inv_mass(side_p3_index) == 0 && inv_mass(side_p4_index) == 0)
            return false;

        Eigen::Vector3d axis_p1 = p.row(axis_p1_index).transpose();
        Eigen::Vector3d axis_p2 = p.row(axis_p2_index).transpose();
        Eigen::Vector3d side_p3 = p.row(side_p3_index).transpose();;
        Eigen::Vector3d side_p4 = p.row(side_p4_index).transpose();;

        Eigen::Vector3d e = axis_p2 - axis_p1;
        double elen = e.norm();

        if (elen > 1e-6) // manage bad model condition
        {
            double inv_elen = 1.0 / elen;

            Eigen::Vector3d n1 = (axis_p1 - side_p3).cross(axis_p2 - side_p3);
            n1 /= n1.squaredNorm();
            Eigen::Vector3d n2 = (axis_p2 - side_p4).cross(axis_p1 - side_p4);
            n2 /= n2.squaredNorm();


            Eigen::Vector3d d1 = (side_p3 - axis_p2).dot(e) * inv_elen * n1 + (side_p4 - axis_p2).dot(e) * inv_elen * n2;
            Eigen::Vector3d d2 = (axis_p1 - side_p3).dot(e) * inv_elen * n1 + (axis_p1 - side_p4).dot(e) * inv_elen * n2;
            Eigen::Vector3d d3 = elen * n1;
            Eigen::Vector3d d4 = elen * n2;

            n1.normalize();
            n2.normalize();

            double dot = n1.dot(n2);
            dot = std::clamp(dot, -1.0, 1.0);// restrict value to [-1.0, 1.0]

            double phi = acos(dot);
            // double phi = (-0.6981317 * dot * dot - 0.8726646) * dot + 1.570796;	// fast approximation

            double lambda =
                    inv_mass(axis_p1_index) * d1.squaredNorm() +
                    inv_mass(axis_p2_index) * d2.squaredNorm() +
                    inv_mass(side_p3_index) * d3.squaredNorm() +
                    inv_mass(side_p4_index) * d4.squaredNorm();

            if (lambda != 0.0)
            {
                lambda = (phi - rest_angle) / lambda * stiffness_;

                if (n1.cross(n2).dot(e) > 0.0)
                    lambda = -lambda;

                p.row(axis_p1_index) += -inv_mass(axis_p1_index) * lambda * d1.transpose();
                p.row(axis_p2_index) += -inv_mass(axis_p2_index) * lambda * d2.transpose();
                p.row(side_p3_index) += -inv_mass(side_p3_index) * lambda * d3.transpose();
                p.row(side_p4_index) += -inv_mass(side_p4_index) * lambda * d4.transpose();
            }
        }
    }
    return true;
}