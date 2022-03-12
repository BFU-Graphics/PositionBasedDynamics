#include "time_integration.h"

void HINASIM::TimeIntegrationDeformable::semi_implicit_integration_with_damping(const double h, const Eigen::VectorXd &inv_mass, Eigen::MatrixXd &predicted_positions, const Eigen::MatrixXd &positions, Eigen::MatrixXd &velocities,
                                                                                const Eigen::MatrixXd &accelerations, double damping)
{
    assert(predicted_positions.rows() == velocities.rows());

    for (int i = 0; i < predicted_positions.rows(); ++i)
    {
        if (inv_mass(i) != 0)
        {
            velocities.row(i) += h * accelerations.row(i);
            velocities.row(i) *= damping;
        }
        predicted_positions.row(i) = positions.row(i) + h * velocities.row(i);
    }
}

void HINASIM::TimeIntegrationDeformable::velocity_update_first_order(double h, const Eigen::VectorXd &inv_mass, const Eigen::MatrixXd &predicted_positions, const Eigen::MatrixXd &old_positions, Eigen::MatrixXd &velocities)
{
    assert(predicted_positions.rows() == velocities.rows());

    for (int i = 0; i < predicted_positions.rows(); ++i)
        if (inv_mass(i) != 0)
            velocities.row(i) = (1.0 / h) * (predicted_positions.row(i) - old_positions.row(i));
}

void HINASIM::TimeIntegrationRigidBody::semi_implicit_integration_with_damping(double h, double inv_mass, Eigen::Vector3d &predicted_position, const Eigen::Vector3d &position, Eigen::Vector3d &velocity, const Eigen::Vector3d &acceleration,
                                                                               const Eigen::Matrix3d &inertia_world, const Eigen::Matrix3d &inv_inertia_world, Eigen::Quaterniond &predicted_rotation, const Eigen::Quaterniond &rotation,
                                                                               Eigen::Vector3d &angular_velocity, const Eigen::Vector3d &torque, double damping)
{
    if (inv_mass != 0.0)
    {
        // linear integration
        velocity += acceleration * h;
        velocity *= damping;

        // angular integration
        angular_velocity += h * inv_inertia_world * (torque - (angular_velocity.cross(inertia_world * angular_velocity)));
    }
    predicted_position = position + h * velocity;

    Eigen::Quaterniond angVelQ(0.0, angular_velocity[0], angular_velocity[1], angular_velocity[2]);
    predicted_rotation = rotation.coeffs() + h * 0.5 * (angVelQ * rotation).coeffs();
    predicted_rotation.normalize();
}

void HINASIM::TimeIntegrationRigidBody::velocity_update_first_order(double h, double inv_mass, const Eigen::Vector3d &predicted_position, const Eigen::Vector3d &position, Eigen::Vector3d &velocity,
                                                                    const Eigen::Quaterniond &predicted_rotation, const Eigen::Quaterniond &rotation, Eigen::Vector3d &angular_velocity)
{
    if (inv_mass != 0.0)
    {
        // linear update
        velocity = (1.0 / h) * (predicted_position - position);

        // angular update
        const Eigen::Quaterniond relRot = (predicted_rotation * rotation.conjugate());
        angular_velocity = relRot.vec() * (2.0 / h);
    }
}

