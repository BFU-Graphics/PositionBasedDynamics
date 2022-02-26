#include "RenderingFrameWork/pbd_viewer.h"
#include "SimulationFrameWork/pbd_sim.h"

int main()
{
    HINAVIEWER::PBDViewer pbd_viewer;
    HINASIM::PBDSim pbd_sim;

    HINASIM::RigidBody cube(PBD_MODEL_DIR + std::string("cube.obj"));

//    HINASIM::DistanceConstraint dc(cube.V_, cube.E_);

    pbd_sim.add_object(&cube);
//    pbd_viewer.record(&cube);
//
//    cube.set_inv_mass(4, 0).add_constraint(&dc);
//
//    pbd_viewer.viewer().callback_post_draw = [&pbd_viewer, &pbd_sim](igl::opengl::glfw::Viewer &viewer) -> bool
//    {
//        pbd_sim.update_all_rendering_state();
//        for (auto &o: pbd_sim.objects_)
//            pbd_viewer.update_vertex_positions(o->ID_, o->V_);
//
//        return false;
//    };
//
//    bool pause = false;
//    auto simulate = [&pbd_sim, &pause]()
//    {
//        while (!pause)
//        {
//            pbd_sim.simulate_real_dt();
//        }
//    };
//
//    std::thread simulation_thread(simulate);
//    simulation_thread.detach();
//
//    pbd_viewer
//            .set_max_fps(60)
//            .launch_rendering("Test Cube");

    return 0;
}