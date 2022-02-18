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
        virtual bool solve(Eigen::MatrixXd &q, const Eigen::VectorXd &inv_mass, double stiffness) = 0;
    };

    class DistanceConstraint : public Constraint, public HINAVIEWER::INSPECTOR::Trackable
    {
    public:
        explicit DistanceConstraint(Eigen::MatrixXd &init_q, const Eigen::MatrixXi &edges);

        bool solve(Eigen::MatrixXd &q, const Eigen::VectorXd &inv_mass, double stiffness) override;

    public:
        std::vector<std::tuple<int, int, double>> distance_constraints_;
    };
}


#endif //POSITIONBASEDDYNAMICS_CONSTRAINTS_H
