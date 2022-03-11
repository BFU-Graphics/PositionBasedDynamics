#ifndef POSITIONBASEDDYNAMICS_TIME_INTEGRATION_H
#define POSITIONBASEDDYNAMICS_TIME_INTEGRATION_H

#include "Eigen/Dense"

namespace HINASIM
{
    class TimeIntegrationDeformable
    {
    public:
        static void semi_implicit_integration_with_damping(double h,
                                                           const Eigen::VectorXd &inv_mass,
                                                           Eigen::MatrixXd &predicted_positions,
                                                           const Eigen::MatrixXd &positions,
                                                           Eigen::MatrixXd &velocities,
                                                           const Eigen::MatrixXd &accelerations,
                                                           double damping);

        static void velocity_update_first_order(double h,
                                                const Eigen::VectorXd &inv_mass,
                                                const Eigen::MatrixXd &predicted_positions,
                                                const Eigen::MatrixXd &old_positions,
                                                Eigen::MatrixXd &velocities);
    };

    class TimeIntegrationRigidBody
    {
    public:
        static void semi_implicit_integration_with_damping(double h,
                                                           double inv_mass,
                                                           Eigen::Vector3d &predicted_position,
                                                           const Eigen::Vector3d &position,
                                                           Eigen::Vector3d &velocity,
                                                           const Eigen::Vector3d &acceleration,
                                                           const Eigen::Matrix3d &inertia_world,
                                                           const Eigen::Matrix3d &inv_inertia_world,
                                                           Eigen::Quaterniond &predicted_rotation,
                                                           const Eigen::Quaterniond &rotation,
                                                           Eigen::Vector3d &angular_velocity,
                                                           const Eigen::Vector3d &torque,
                                                           double damping);

        static void velocity_update_first_order(double h,
                                                double inv_mass,
                                                const Eigen::Vector3d &predicted_position,
                                                const Eigen::Vector3d &position,
                                                Eigen::Vector3d &velocity,
                                                const Eigen::Quaterniond &predicted_rotation,
                                                const Eigen::Quaterniond &rotation,
                                                Eigen::Vector3d &angular_velocity
        );
    };
}


#endif //POSITIONBASEDDYNAMICS_TIME_INTEGRATION_H
