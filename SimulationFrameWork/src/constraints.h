/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_CONSTRAINTS_H
#define POSITIONBASEDDYNAMICS_CONSTRAINTS_H

#include "RenderingFrameWork/src/inspector.h"
#include "RenderingFrameWork/eigen_types.h"

namespace HINASIM
{
    class Constraint
    {
    public:
        virtual bool solve(Eigen::VectorXd &q, const Eigen::SparseMatrix<double> &M_inv, double stiffness) = 0;
    };

    class DistanceConstraint : public Constraint, public HINAVIEWER::INSPECTOR::Trackable
    {
    public:
        explicit DistanceConstraint(Eigen::VectorXd &init_q, const Eigen::MatrixXi &edges);

        bool solve(Eigen::VectorXd &q, const Eigen::SparseMatrix<double>& M_inv, double stiffness) override;

        /**
         * Replace all content below at your will
         */

    public: // const fields (won't change during the simulation)
        std::vector<Eigen::SparseMatrixd> Es; // election matrix group
        Eigen::Matrix36d B; // composition matrix group
        Eigen::VectorXd rest_length; // rest length group
    };
}


#endif //POSITIONBASEDDYNAMICS_CONSTRAINTS_H
