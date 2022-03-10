#ifndef POSITIONBASEDDYNAMICS_RIGIDBODY_H
#define POSITIONBASEDDYNAMICS_RIGIDBODY_H

#include "objects.h"

namespace HINASIM
{
    class RigidBody : public SimObject
    {
    public: // constructors
        explicit RigidBody(const std::string &path, Eigen::Vector3d position = {0, 0, 0}, Eigen::Vector3d rotation = {0, 0, 0}, Eigen::Vector3d scale = {1, 1, 1}, double density = 100.0);
        explicit RigidBody(const std::function<void(Eigen::MatrixXd &V, Eigen::MatrixXi &F)> &custom_init_geometry, Eigen::Vector3d position = {0, 0, 0}, Eigen::Vector3d rotation = {0, 0, 0}, Eigen::Vector3d scale = {1, 1, 1}, double density = 100.0);

    public: // chained useful methods

    public: // simulation state
        Eigen::Vector3d x_; // linear position
        Eigen::Vector3d v_; // linear velocity
        Eigen::Vector3d a_; // linear acceleration
        Eigen::Quaterniond q_; // quaternion
        Eigen::Vector3d omega_; // angular velocity
        Eigen::Vector3d t_; // torque

        double inv_mass_;
        Eigen::Vector3d inertia_tensor_local_;
        Eigen::Matrix3d inertia_tensor_world_;
        Eigen::Vector3d inv_inertia_tensor_local_;
        Eigen::Matrix3d inv_inertia_tensor_world_;

        Eigen::MatrixXd V_rest_; // rest pose of a rigid body
        double restitution_; // TODO: to init here
        double friction_; // TODO: to init here

    protected: // disabled constructors
        explicit RigidBody(Eigen::Vector3d position = {0, 0, 0}, Eigen::Vector3d rotation = {0, 0, 0}, Eigen::Vector3d scale = {1, 1, 1}, double density = 100.0);

    protected:
        void init_physics_states() override;
        void update_rendering_info() override;
        void update_physics_info() override;
    };
}


#endif //POSITIONBASEDDYNAMICS_RIGIDBODY_H
