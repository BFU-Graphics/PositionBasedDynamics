/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_PBD_SIM_H
#define POSITIONBASEDDYNAMICS_PBD_SIM_H

#include "src/objects/deformable.h"
#include "src/constraints/constraints.h"
#include "src/time_integration.h"

#include <vector>

namespace HINASIM
{
    class PBDSim
    {
    public:
        void simulate_real_dt();

        void simulate(double dt);

    public:
        void add_object(SimObject *object);

        void update_all_rendering_state();

    protected: // pbd kernel process
        void pbd_kernel_loop(double dt);

        void external_force(HINASIM::SimObject *o);

        static void integrate_prediction_with_damping(HINASIM::SimObject *o, double dt, double damping = 0.999);

        void generate_collision_constraints();

        void project_position_constraints();

        static void update_positions_and_velocities(HINASIM::SimObject *o, double dt);

    public:
        std::vector<SimObject *> objects_;
        std::vector<class Joint *> joints_;

    protected: // env
        Eigen::RowVector3d gravity_{0, -9.8, 0};
        double fixed_dt = 0.02;
    };
}

#endif //POSITIONBASEDDYNAMICS_PBD_SIM_H
