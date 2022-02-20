#include "RenderingFrameWork/pbd_viewer.h"
#include "../../pbd_sim.h"

int main()
{
    // ======================================== Phase 0: Init Simulation & Rendering Engine  ========================================

    HINAVIEWER::PBDViewer pbd_viewer;
    HINASIM::PBDSim pbd_sim;

    // ======================================== Phase 1: Init Simulation World Info  ========================================

    HINASIM::DeformableObject starry_fish;
    starry_fish.init_geometry("/Users/xayah/Desktop/GithubRepo/tree-gen/out/tree1.obj");
    HINASIM::DistanceConstraint dc(starry_fish.q_, starry_fish.E_);
    starry_fish.set_inv_mass(0, 0).add_constraint(&dc);
//    pbd_sim.add_object(&starry_fish);
    pbd_viewer.record(&starry_fish);

    // ======================================== Phase 2: Set Up Simulation Thread ========================================

    bool pause = false;
    auto simulate = [&pbd_sim, &pause]()
    {
        while (!pause)
        {
            pbd_sim.simulate_real_dt();
        }
    };

    std::thread simulation_thread(simulate);
    simulation_thread.detach();

    // ======================================== Phase 3: Set Up Rendering Thread ========================================

    pbd_viewer.viewer().callback_post_draw = [&pbd_viewer, &pbd_sim](igl::opengl::glfw::Viewer &viewer) -> bool
    {
        pbd_sim.update_all_rendering_state();
        for (auto &o: pbd_sim.objects_)
        {
            o->update_mouse_drag();
            pbd_viewer.update_vertex_positions(o->ID_, o->V_);
        }
        return false;
    };

    pbd_viewer
            .set_max_fps(60)
            .launch_rendering("Test Cloth");

    return 0;
}