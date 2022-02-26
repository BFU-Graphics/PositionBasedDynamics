#ifndef POSITIONBASEDDYNAMICS_TIME_INTEGRATION_H
#define POSITIONBASEDDYNAMICS_TIME_INTEGRATION_H

#include "RenderingFrameWork/eigen_types.h"

namespace HINASIM
{
    class TimeIntegration
    {
    public:
        static void semi_implicit_integration_with_damping(
                double h,
                const Eigen::VectorXd &inv_mass,
                Eigen::MatrixXd &predict_positions,
                const Eigen::MatrixXd &positions,
                Eigen::MatrixXd &velocities,
                const Eigen::MatrixXd &accelerations, double damping);

        static void velocity_update_first_order(double h,
                                                const Eigen::VectorXd &inv_mass,
                                                const Eigen::MatrixXd &predict_positions,
                                                const Eigen::MatrixXd &old_positions,
                                                Eigen::MatrixXd &velocities);
    };
}


#endif //POSITIONBASEDDYNAMICS_TIME_INTEGRATION_H
