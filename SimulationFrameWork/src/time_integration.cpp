#include "time_integration.h"

void HINASIM::TimeIntegration::semi_implicit_integration_with_damping(const double h, const Eigen::VectorXd &inv_mass, Eigen::MatrixXd &predict_positions, const Eigen::MatrixXd &positions, Eigen::MatrixXd &velocities,
                                                                      const Eigen::MatrixXd &accelerations, double damping)
{
    assert(predict_positions.rows() == velocities.rows());

    for (int i = 0; i < predict_positions.rows(); ++i)
    {
        if (inv_mass(i) != 0)
        {
            velocities.row(i) += h * accelerations.row(i);
            velocities.row(i) *= damping;
        }
        predict_positions.row(i) = positions.row(i) + h * velocities.row(i);
    }
}

void HINASIM::TimeIntegration::velocity_update_first_order(double h, const Eigen::VectorXd &inv_mass, const Eigen::MatrixXd &predict_positions, const Eigen::MatrixXd &old_positions, Eigen::MatrixXd &velocities)
{
    assert(predict_positions.rows() == velocities.rows());

    for (int i = 0; i < predict_positions.rows(); ++i)
    {
        if (inv_mass(i) != 0)
        {
            velocities.row(i) = (1.0 / h) * (predict_positions.row(i) - old_positions.row(i));
        }
    }
}
