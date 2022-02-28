#include "RenderingFrameWork/pbd_viewer.h"
#include "SimulationFrameWork/pbd_sim.h"

int main()
{
    // ======================================== Phase 0: Init Simulation & Rendering Engine  ========================================

    HINAVIEWER::PBDViewer pbd_viewer;
    HINASIM::PBDSim pbd_sim;

    // ======================================== Phase 1: Init Simulation World Info  ========================================

    HINASIM::Cloth cloth(30, 30, 10, 10, {0, 6, 3});

    HINASIM::DistanceConstraint dc_cloth(cloth.V_, cloth.E_);
    HINASIM::DihedralConstraint dic_cloth(cloth.V_, cloth.EVF_, cloth.F_);
    dic_cloth.stiffness_ = 0.3;

    HINASIM::SphereCollider sphere_collider({0, 0, 0}, 3);

    pbd_sim // !important: make sure to add all objects before set/alter any physics state
            .add_object(&cloth)
            .add_collider(&sphere_collider);

    cloth // !important: make sure to set/alter all physics states after SimObjects were added into PBDSim
            .set_inv_mass(0, 0)
            .set_inv_mass(29, 0)
            .add_constraint(&dc_cloth)
            .add_constraint(&dic_cloth);

    pbd_viewer.record(&cloth);
    pbd_viewer.record(&sphere_collider);

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
            .set_max_fps(60)
            .launch_rendering("Test Cloth");

    return 0;
}
