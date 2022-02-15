/**
 * @author YC XIANG.
 * @date 02/09, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_CONSTRAINTS_H
#define POSITIONBASEDDYNAMICS_CONSTRAINTS_H

#include "Utils/visualization_src/eigen_types.h"
#include "Utils/visualization_src/inspector.h"

#include <vector>

namespace pbd_src
{
    class Constraint
    {
    public:
        virtual bool solve(Eigen::VectorXd &q, const Eigen::SparseMatrix<double> &M_inv, double stiffness) = 0;
    };

    class DistanceConstraint : public Constraint, public pbd_inspector::Trackable
    {
    public:
        explicit DistanceConstraint(Eigen::VectorXd &init_q, const Eigen::MatrixXi &edges);

        bool solve(Eigen::VectorXd &q, const Eigen::SparseMatrix<double> &M_inv, double stiffness) override;

    public: // const fields (won't change during the simulation)
        std::vector<Eigen::SparseMatrixd> Es; // election matrix group
        Eigen::Matrix36d B; // composition matrix group
        Eigen::VectorXd rest_length; // rest length group
    };
}


#endif //POSITIONBASEDDYNAMICS_CONSTRAINTS_H
