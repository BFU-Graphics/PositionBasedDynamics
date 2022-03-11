#include "RenderingFrameWork/pbd_viewer.h"
#include "SimulationFrameWork/pbd_sim.h"
#include "SimulationFrameWork/src/collisions/distance_field_collision_detection.h"

int main()
{
    // ======================================== Phase 0: Init Simulation & Rendering & Collision Engine  ========================================
    HINAVIEWER::PBDViewer pbd_viewer;
    HINASIM::PBDSim pbd_sim;
    HINASIM::DistanceFieldCollisionDetection cd;

    // ======================================== Phase 1: Init Simulation World Info  ========================================

    HINASIM::RigidBody sphere1(PBD_MODEL_DIR + std::string("sphere.obj"), {0, -5, 0});
    HINASIM::RigidBody sphere2(PBD_MODEL_DIR + std::string("sphere.obj"), {0, 5, 0});

    pbd_sim
            .set_collision_engine(&cd)
            .add_object(&sphere1)
            .add_object(&sphere2);

    sphere1.set_inv_mass(0);

    cd.add_collider_sphere(&sphere1);
    cd.add_collider_sphere(&sphere2);

    pbd_viewer.record(&sphere1);
    pbd_viewer.record(&sphere2);

    // ======================================== Phase 2: Set Up Simulation Thread ========================================


    bool pause = false;
    auto simulate = [&pbd_sim, &pause]()
    {
        while (!pause)
        {
            pbd_sim.simulate_real_dt(); // simulate with real time elapsed;
        }
    };

    std::thread simulation_thread(simulate);
    simulation_thread.detach();

    // ======================================== Phase 3: Set Up Rendering Thread ========================================

    pbd_viewer.viewer().callback_post_draw = [&pbd_viewer, &pbd_sim](igl::opengl::glfw::Viewer &viewer) -> bool
    {
        pbd_sim.update_all_rendering_state();
        for (auto &o: pbd_sim.objects_)
            pbd_viewer.update_vertex_positions(o->ID_, o->V_);
        return false;
    };

    pbd_viewer
            .set_max_fps(120)
            .launch_rendering("Test Cloth");

    return 0;
}
