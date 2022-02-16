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

    public:
        void add_object(SimObject *object);

    protected:
        void integrate_velocity_by_gravity(Eigen::Ref<Eigen::VectorXd> qdot, double dt);

        void damping_velocity(Eigen::Ref<Eigen::VectorXd> qdot);

        void predict_position(Eigen::Ref<Eigen::VectorXd> p, Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> qdot, double dt);

        void generate_collision_constraints();

        void constraints_projection(SimObject *o);

        void update_q_and_qdot(Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> qdot, Eigen::Ref<Eigen::VectorXd> p, double dt);

    protected:
        typedef Constraint Joint;
        std::vector<SimObject *> objects_;
        std::vector<Joint *> joints_;

    protected: // Env
        Eigen::Vector3d gravity{0, -9.8, 0};
    };
}

#endif //POSITIONBASEDDYNAMICS_PBD_SIM_H
