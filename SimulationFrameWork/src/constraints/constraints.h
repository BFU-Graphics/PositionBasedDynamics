#ifndef POSITIONBASEDDYNAMICS_CONSTRAINTS_H
#define POSITIONBASEDDYNAMICS_CONSTRAINTS_H

#include <Eigen/Dense>

#include <vector>
#include <tuple>

namespace HINASIM
{
    class InnerConstraint
    {
    public:
        virtual bool solve(Eigen::MatrixXd &p, const Eigen::VectorXd &inv_mass) = 0;

    public:
        double stiffness_ = 1.0;
    };

    class DistanceConstraint : public InnerConstraint
    {
    public:
        explicit DistanceConstraint(Eigen::MatrixXd &init_x, const Eigen::MatrixXi &edges);

        bool solve(Eigen::MatrixXd &p, const Eigen::VectorXd &inv_mass) override;

    public:
        /// p1_id, p2_id, rest_length
        std::vector<std::tuple<int, int, double>> distance_constraints_;
    };

    class DihedralConstraint : public InnerConstraint
    {
    public:
        DihedralConstraint(Eigen::MatrixXd &init_q, const std::vector<std::vector<unsigned int>> &EVF, const Eigen::MatrixXi &F);

        bool solve(Eigen::MatrixXd &p, const Eigen::VectorXd &inv_mass) override;

    public:
        /// axis_p1_id, axis_p2_id, side_p3_id, axis_p4_id, rest_angle
        std::vector<std::tuple<int, int, int, int, double>> dihedral_constraints_;
    };
}


#endif //POSITIONBASEDDYNAMICS_CONSTRAINTS_H
