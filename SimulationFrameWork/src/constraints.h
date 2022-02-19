/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_CONSTRAINTS_H
#define POSITIONBASEDDYNAMICS_CONSTRAINTS_H

#include "RenderingFrameWork/src/inspector.h"
#include "RenderingFrameWork/eigen_types.h"

#include <tuple>

namespace HINASIM
{
    class Constraint
    {
    public:
        virtual bool solve(Eigen::MatrixXd &p, const Eigen::VectorXd &inv_mass, double stiffness) = 0;
    };

    class DistanceConstraint : public Constraint, public HINAVIEWER::INSPECTOR::Trackable
    {
    public:
        explicit DistanceConstraint(Eigen::MatrixXd &init_q, const Eigen::MatrixXi &edges);

        bool solve(Eigen::MatrixXd &p, const Eigen::VectorXd &inv_mass, double stiffness) override;

    public:
        /// p1_id, p2_id, rest_length
        std::vector<std::tuple<int, int, double>> distance_constraints_;
    };

    class DihedralConstraint : public Constraint
    {
    public:
        DihedralConstraint(Eigen::MatrixXd &init_q, const std::vector<std::vector<unsigned int>> &EVF, const Eigen::MatrixXi &F);

        bool solve(Eigen::MatrixXd &p, const Eigen::VectorXd &inv_mass, double stiffness) override;

    public:
        /// axis_p1_id, axis_p2_id, side_p3_id, axis_p4_id, rest_angle
        std::vector<std::tuple<int, int, int, int, double>> dihedral_constraints_;
    };
}


#endif //POSITIONBASEDDYNAMICS_CONSTRAINTS_H
