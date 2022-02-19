/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "RenderingFrameWork/pbd_viewer.h"
#include "../pbd_sim.h"

/// SET FFMPEG PATH TO ENABLE OUTPUT MP4 VIDEO
//#define FFMPEG_PATH "D:/ffmpeg-n4.4-latest-win64-gpl-4.4/bin/ffmpeg.exe"

int main()
{
    HINAVIEWER::PBDViewer pbd_viewer;
    HINASIM::PBDSim pbd_sim;

//    HINASIM::DeformableObject cube;
//    cube.init_geometry(PBD_MODEL_DIR + std::string("cube.obj"));
//    HINASIM::DistanceConstraint dc(cube.q_, cube.E_);
//    func(pbd_viewer, pbd_sim, cube, dc);

    HINASIM::Cloth cloth(30, 30, 20, 20);
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

#ifdef FFMPEG_PATH
        pbd_viewer.write_current_frame();
#endif
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
//            .show_inspector()
//            .focus_object(0)
#ifdef FFMPEG_PATH
                    .save_to_mp4(FFMPEG_PATH, "my_output.mp4")
#endif
            .launch_rendering("Simulation Framework Test 01");

    return 0;
}