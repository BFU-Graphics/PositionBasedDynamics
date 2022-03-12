#include "RenderingFrameWork/pbd_viewer.h"
#include "SimulationFrameWork/pbd_sim.h"
#include "SimulationFrameWork/src/collisions/distance_field_collision_detection.h"

void add_many_spheres(HINASIM::PBDSim &pbd_sim, HINAVIEWER::PBDViewer &pbd_viewer, HINASIM::DistanceFieldCollisionDetection &cd)
{
    int size = 5;
    for (int i = -size; i < size; ++i)
        for (int j = -size; j < size; ++j)
            for (int k = -size; k < size; ++k)
            {
                auto *sphere = new HINASIM::RigidBody(PBD_MODEL_DIR + std::string("sphere.obj"), {2.2 * i, 2.2 * j + size + 10, 2.2 * k});
                pbd_sim.add_object(sphere);
                cd.add_collider_sphere(sphere);
                pbd_viewer.record(sphere);
            }
};

int main()
{
    // ======================================== Phase 0: Init Simulation & Rendering & Collision Engine  ========================================
    HINAVIEWER::PBDViewer pbd_viewer;
    HINASIM::PBDSim pbd_sim;
    HINASIM::DistanceFieldCollisionDetection cd;

    // ======================================== Phase 1: Init Simulation World Info  ========================================

    HINASIM::RigidBody sphere_fixed(PBD_MODEL_DIR + std::string("sphere.obj"), {0, -300, 0}, {0, 0, 0}, {300, 300, 300});

    pbd_sim
            .set_collision_engine(&cd)
            .add_object(&sphere_fixed);

    sphere_fixed.set_inv_mass(0);

    cd.add_collider_sphere(&sphere_fixed);

    pbd_viewer.record(&sphere_fixed);

    add_many_spheres(pbd_sim, pbd_viewer, cd);

    // ======================================== Phase 2: Set Up Simulation Thread ========================================

    bool is_simulating = true;
    auto simulate = [&pbd_sim, &is_simulating]()
    {
        while (is_simulating)
        {
            std::cout << (pbd_sim.pause_ ? "Paused" : "") << std::endl;
            if (!pbd_sim.pause_)
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

    pbd_viewer.viewer().callback_key_down = [&pbd_sim, &is_simulating](igl::opengl::glfw::Viewer &viewer, unsigned int key, int modifiers) -> bool
    {
        if (key == ' ')
            pbd_sim.pause_ = !pbd_sim.pause_;
        if (key == 'x')
            is_simulating = false;
        return false;
    };

    pbd_viewer.viewer().core().camera_zoom = 0.3;

    pbd_viewer
            .set_max_fps(120)
            .launch_rendering("Test Rigid Body");

    return 0;
}
