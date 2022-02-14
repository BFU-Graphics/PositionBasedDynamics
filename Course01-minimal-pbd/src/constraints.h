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
    class DistanceConstraint : public pbd_inspector::Trackable
    {
    public:
        explicit DistanceConstraint(Eigen::VectorXd &init_q, const Eigen::MatrixXi &edges);

        bool solve(const Eigen::VectorXd &q, const Eigen::SparseMatrix<double>& M_inv, Eigen::VectorXd &dq, double stiffness = 10000.0);

    public: // const fields (won't change during the simulation)
        std::vector<Eigen::SparseMatrixd> Es; // election matrix group
        Eigen::Matrix36d B; // composition matrix group
        Eigen::VectorXd rest_length; // rest length group
    };
}


#endif //POSITIONBASEDDYNAMICS_CONSTRAINTS_H
