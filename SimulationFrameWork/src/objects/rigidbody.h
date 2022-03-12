#ifndef POSITIONBASEDDYNAMICS_RIGIDBODY_H
#define POSITIONBASEDDYNAMICS_RIGIDBODY_H

#include "objects.h"

namespace HINASIM
{
    class RigidBody : public SimObject
    {
    public: // constructors
        explicit RigidBody(const std::string &path, Eigen::Vector3d position = {0, 0, 0}, Eigen::Vector3d rotation = {0, 0, 0}, Eigen::Vector3d scale = {1, 1, 1}, double density = 100.0);
        explicit RigidBody(const std::function<void(Eigen::MatrixXd &V, Eigen::MatrixXi &F)> &custom_init_geometry, Eigen::Vector3d position = {0, 0, 0}, Eigen::Vector3d rotation = {0, 0, 0}, Eigen::Vector3d scale = {1, 1, 1},
                           double density = 100.0);
        ~RigidBody() override = default;

    public: // chained useful methods
        RigidBody &set_inv_mass(double inv_mass);

    public: // simulation state
        Eigen::Vector3d x_; // linear position
        Eigen::Vector3d v_; // linear velocity
        Eigen::Vector3d a_; // linear acceleration
        Eigen::Quaterniond q_; // quaternion
        Eigen::Vector3d omega_; // angular velocity
        Eigen::Vector3d t_; // torque

        Eigen::Vector3d p_x_; // predicted linear position
        Eigen::Quaterniond p_q_; // predicted angular quaternion

        double inv_mass_;
        Eigen::Vector3d inertia_tensor_local_;
        Eigen::Matrix3d inertia_tensor_world_;
        Eigen::Vector3d inv_inertia_tensor_local_;
        Eigen::Matrix3d inv_inertia_tensor_world_;

        Eigen::MatrixXd V_rest_; // rest pose of a rigid body
        Eigen::MatrixXd V_buffer_; // rendering pose buffer (because physics update was faster than rendering update, we won;t update rendering V_ every physics frame)
        Eigen::Vector3d C_rest_; // rest center of mass

        double restitution_{0.5}; // TODO: to init here
        double friction_{0.2}; // TODO: to init here

        // transformation required to transform a point to local space or vice vera
        Eigen::Matrix3d transformation_R_;
        Eigen::Vector3d transformation_v1_;
        Eigen::Vector3d transformation_v2_;
        Eigen::Vector3d transformation_R_X_v1_;

        Eigen::Quaterniond q_mat_; // mat for material~
        Eigen::Quaterniond q_initial_;
        Eigen::Vector3d x0_mat_;


    protected: // disabled constructors
        explicit RigidBody(Eigen::Vector3d position = {0, 0, 0}, Eigen::Vector3d rotation = {0, 0, 0}, Eigen::Vector3d scale = {1, 1, 1}, double density = 100.0);

    protected:
        void init_physics_states() override;
        void update_rendering_info() override;
        void update_physics_info() override;
    };
}


#endif //POSITIONBASEDDYNAMICS_RIGIDBODY_H
