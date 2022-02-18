/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "RenderingFrameWork/pbd_viewer.h"
#include "../pbd_sim.h"

/// SET FFMPEG PATH TO ENABLE OUTPUT MP4 VIDEO
//#define FFMPEG_PATH "D:/ffmpeg-n4.4-latest-win64-gpl-4.4/bin/ffmpeg.exe"

void func(HINAVIEWER::PBDViewer &pbd_viewer, HINASIM::PBDSim &pbd_sim, HINASIM::DeformableObject &obj, HINASIM::DistanceConstraint &dc)
{
    obj.set_inv_mass(0, 0).add_constraint(&dc);
    pbd_sim.add_object(&obj);
    pbd_viewer.record(&obj);
    pbd_viewer.track(&dc, 1);
}

int main()
{
    HINAVIEWER::PBDViewer pbd_viewer;
    HINASIM::PBDSim pbd_sim;

    HINASIM::DeformableObject cube;
    cube.init_geometry(PBD_MODEL_DIR + std::string("cube.obj"));
    HINASIM::DistanceConstraint dc(cube.q_, cube.E_);

    HINASIM::Cloth cloth(20, 20, 3, 3);
    cloth.init_geometry();
    HINASIM::DistanceConstraint dc_cloth(cloth.q_, cloth.E_);

//    func(pbd_viewer, pbd_sim, cube, dc);
    func(pbd_viewer, pbd_sim, cloth, dc_cloth);


    pbd_viewer.viewer().callback_post_draw = [&pbd_viewer, &pbd_sim](igl::opengl::glfw::Viewer &viewer) -> bool
    {
        for (auto &o: pbd_sim.objects_)
        {
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
#ifdef FFMPEG_PATH
                    .save_to_mp4(FFMPEG_PATH, "my_output.mp4")
#endif
            .launch_rendering("Simulation Framework Test 01");

    return 0;
}