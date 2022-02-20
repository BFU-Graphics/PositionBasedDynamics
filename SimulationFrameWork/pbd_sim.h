/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_PBD_SIM_H
#define POSITIONBASEDDYNAMICS_PBD_SIM_H

#include "src/objects.h"
#include "src/constraints.h"
#include "src/contact_constraints.h"

#include <vector>

namespace HINASIM
{
    class PBDSim
    {
    public:
        void simulate(double dt);

        void simulate_real_dt();

    public:
        void add_object(SimObject *object);

        void update_all_rendering_state();

    public:
        PBDSim &fix_dt(double dt);

    protected: // PBD Kernel Region
        void pbd_kernel_loop(double dt);

        void integrate_velocity_by_external_force(Eigen::Ref<Eigen::MatrixXd> qdot, Eigen::Ref<Eigen::MatrixXd> external_force, Eigen::Ref<Eigen::VectorXd> inv_mass, double dt);

        static void damping_velocity(Eigen::Ref<Eigen::MatrixXd> qdot);

        static void predict_position(Eigen::Ref<Eigen::MatrixXd> p, Eigen::Ref<Eigen::MatrixXd> q, Eigen::Ref<Eigen::MatrixXd> qdot, double dt);

        static void generate_collision_constraints();

        static void constraints_projection(SimObject *o);

        static void update_q_and_qdot(Eigen::Ref<Eigen::MatrixXd> q, Eigen::Ref<Eigen::MatrixXd> qdot, Eigen::Ref<Eigen::MatrixXd> p, double dt);

    public:
        using Joint = Constraint;
        std::vector<SimObject *> objects_;
        std::vector<Joint *> joints_;
        std::vector<RigidBodyContactConstraint *> rigid_body_contacts_constraints_;

    protected: // Env
        Eigen::RowVector3d gravity_{0, -9.8, 0};
        double fixed_dt = 0.02;
    };
}

#endif //POSITIONBASEDDYNAMICS_PBD_SIM_H
