/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_PBD_SIM_H
#define POSITIONBASEDDYNAMICS_PBD_SIM_H

#include "src/objects.h"
#include "src/constraints.h"

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
        PBDSim& fix_dt(double dt);

    protected: // PBD Kernel Region
        void pbd_kernel_loop(double dt);

        void integrate_velocity_by_gravity(Eigen::Ref<Eigen::VectorXd> qdot, Eigen::Ref<Eigen::VectorXd> inv_mass, double dt);

        static void damping_velocity(Eigen::Ref<Eigen::VectorXd> qdot);

        static void predict_position(Eigen::Ref<Eigen::VectorXd> p, Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> qdot, double dt);

        static void generate_collision_constraints();

        static void constraints_projection(SimObject *o);

        static void update_q_and_qdot(Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> qdot, Eigen::Ref<Eigen::VectorXd> p, double dt);

    protected:
        typedef Constraint Joint;
        std::vector<SimObject *> objects_;
        std::vector<Joint *> joints_;

    protected: // Env
        Eigen::Vector3d gravity{0, -9.8, 0};
        double fixed_dt = 0.02;
    };
}

#endif //POSITIONBASEDDYNAMICS_PBD_SIM_H
