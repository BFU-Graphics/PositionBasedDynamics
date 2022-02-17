/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "RenderingFrameWork/pbd_viewer.h"
#include "../pbd_sim.h"

int main()
{
    HINAVIEWER::PBDViewer pbd_viewer;
    HINASIM::PBDSim pbd_sim;

    HINASIM::DeformableObject obj;
    obj.init_geometry(PBD_MODEL_DIR + std::string("cube.obj"));
    HINASIM::DistanceConstraint dc(obj.q_, obj.E_);
    obj.set_inv_mass(0, 0).add_constraint(&dc);

    pbd_sim.add_object(&obj);
    pbd_viewer.record(&obj);
    pbd_viewer.track(&dc, 1);
    pbd_viewer.track(&dc, 2);

    pbd_viewer.viewer().callback_post_draw = [&pbd_viewer, &obj](igl::opengl::glfw::Viewer &viewer) -> bool
    {
        pbd_viewer.update_vertex_positions(obj.ID_, obj.V_);
        return false;
    };

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

//    pbd_viewer.set_max_fps(60).launch_rendering();
    pbd_viewer.set_max_fps(60).show_inspector().launch_rendering("Simulation Framework Test 01");
//    pbd_viewer.set_max_fps(60).focus_object(0).show_inspector().launch_rendering();

    return 0;
}
