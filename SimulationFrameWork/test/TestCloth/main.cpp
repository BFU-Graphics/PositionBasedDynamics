#include "RenderingFrameWork/pbd_viewer.h"
#include "../../pbd_sim.h"

int main()
{
    HINAVIEWER::PBDViewer pbd_viewer;
    HINASIM::PBDSim pbd_sim;

    HINASIM::Cloth cloth(30, 30, 10, 10);
    cloth.init_geometry();
    HINASIM::DistanceConstraint dc_cloth(cloth.q_, cloth.E_);
    HINASIM::DihedralConstraint dic_cloth(cloth.q_, cloth.EVF_, cloth.F_);
    cloth
            .set_inv_mass(0, 0)
            .set_inv_mass(29, 0)
            .add_constraint(&dc_cloth)
            .add_constraint(&dic_cloth);

    pbd_sim.add_object(&cloth);
    pbd_viewer.record(&cloth);

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

    pbd_viewer
            .set_max_fps(60)
            .launch_rendering("Simulation Framework Test 01");

    return 0;
}