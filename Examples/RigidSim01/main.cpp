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

    HINASIM::RigidBody sphere1(PBD_MODEL_DIR + std::string("sphere.obj"), {0, -3, 0}, {70, 40, 20}, {5, 5, 5});
    HINASIM::RigidBody sphere2(PBD_MODEL_DIR + std::string("sphere.obj"), {1, 4, 0}, {-70, -40, -20}, {1, 1, 1});
    HINASIM::RigidBody sphere3(PBD_MODEL_DIR + std::string("sphere.obj"), {-1, 6, 0}, {0, 0, 0}, {1, 1, 1});
    HINASIM::RigidBody sphere4(PBD_MODEL_DIR + std::string("sphere.obj"), {0, 8, -1}, {0, 0, 0}, {1, 1, 1});
    HINASIM::RigidBody sphere5(PBD_MODEL_DIR + std::string("sphere.obj"), {0, 10, 1}, {0, 0, 0}, {1, 1, 1});
    HINASIM::RigidBody sphere6(PBD_MODEL_DIR + std::string("sphere.obj"), {0, 12, 0}, {0, 0, 0}, {1, 1, 1});

    pbd_sim
            .set_collision_engine(&cd)
            .add_object(&sphere1)
            .add_object(&sphere2)
            .add_object(&sphere3)
            .add_object(&sphere4)
            .add_object(&sphere5)
            .add_object(&sphere6);

    sphere1.set_inv_mass(0);

    cd.add_collider_sphere(&sphere1);
    cd.add_collider_sphere(&sphere2);
    cd.add_collider_sphere(&sphere3);
    cd.add_collider_sphere(&sphere4);
    cd.add_collider_sphere(&sphere5);
    cd.add_collider_sphere(&sphere6);

    pbd_viewer.record(&sphere2);
    pbd_viewer.record(&sphere3);
    pbd_viewer.record(&sphere4);
    pbd_viewer.record(&sphere5);
    pbd_viewer.record(&sphere6);
    pbd_viewer.record(&sphere1);

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

    pbd_viewer.viewer().core().camera_zoom = 0.5;

    pbd_viewer
            .set_max_fps(120)
            .launch_rendering("Test Rigid Body");

    return 0;
}
