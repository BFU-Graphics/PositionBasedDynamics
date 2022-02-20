/**
 * @author YC XIANG.
 * @date 02/21, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_CONTACT_CONSTRAINTS_H
#define POSITIONBASEDDYNAMICS_CONTACT_CONSTRAINTS_H

#include "RenderingFrameWork/eigen_types.h"

#include <tuple>

namespace HINASIM
{
    class SimObject;

    class ContactConstraint
    {
    public:
        virtual bool solve(Eigen::MatrixXd &qdot, const Eigen::VectorXd &inv_mass) = 0;

    public:
        double stiffness_ = 1.0;
    };

    class RigidBodyContactConstraint : public ContactConstraint
    {
    public:
        RigidBodyContactConstraint(const std::vector<SimObject *> &objects, int rigidbody_index1, int rigidbody_index2, const Eigen::Vector3d &contact_point1, const Eigen::Vector3d &contact_point2, const Eigen::Vector3d &contact_normal2,
                                   double restitution, double friction);

        bool solve(Eigen::MatrixXd &p, const Eigen::VectorXd &inv_mass) override;

    public:
        /// rigidbody1_id, rigidbody2_id, friction_coefficient, sum_impulses
        std::vector<std::tuple<int, int, double, double>> distance_constraints_;
    };
}


#endif //POSITIONBASEDDYNAMICS_CONTACT_CONSTRAINTS_H
