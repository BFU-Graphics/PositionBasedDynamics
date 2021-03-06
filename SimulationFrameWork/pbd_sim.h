#ifndef POSITIONBASEDDYNAMICS_PBD_SIM_H
#define POSITIONBASEDDYNAMICS_PBD_SIM_H

#include "src/objects/rigidbody.h"
#include "src/objects/deformable.h"
#include "src/simple_collision/collision_objects.h"
#include "src/constraints/constraints.h"
#include "src/constraints/joints.h"
#include "src/collisions/collision_detection.h"
#include "src/time_integration.h"

#include <vector>

namespace HINASIM
{
    class PBDSim
    {
    public:
        PBDSim() = default;
        ~PBDSim();

    public:
        void simulate_real_dt();
        void simulate(double dt);
        bool pause_ = true;

    public:
        PBDSim &set_collision_engine(CollisionDetection *collision_engine);
        PBDSim &add_object(SimObject *object);
        void update_all_rendering_state();

    protected: // pbd kernel process
        void pbd_kernel_loop(double dt);
        void external_force(HINASIM::SimObject *o);
        static void integrate_prediction_with_damping(HINASIM::SimObject *o, double dt, double damping = 0.999);
        void generate_collision_constraints();
        void project_position_constraints();
        void project_velocity_constraints();
        static void update_positions_and_velocities(HINASIM::SimObject *o, double dt);

    public:
        std::vector<SimObject *> objects_;
        std::vector<Joint *> joints_; // not used yet, for rigid-rigid situation

    protected: // Env vars
        CollisionDetection *collision_engine_ = nullptr;
        Eigen::RowVector3d gravity_{0, -9.8, 0};
        double fixed_dt_ = 0.02;

    public: // ! TO DELETE IN THE FUTURE
        PBDSim &add_collider(CollisionObject *collider);
        std::vector<CollisionObject *> colliders_;
        void collision_response();
        static void collision_response_to_a_sphere(HINASIM::DeformableObject *deformable, const Eigen::Vector3d &center, double radius);
    };
}

#endif //POSITIONBASEDDYNAMICS_PBD_SIM_H
