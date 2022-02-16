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
    obj.set_inv_mass(1).add_constraint(new HINASIM::DistanceConstraint(obj.q_, obj.E_));

    pbd_sim.add_object(&obj);
    pbd_viewer.record(&obj);

    pbd_viewer.viewer().callback_post_draw = [&pbd_viewer, &obj](igl::opengl::glfw::Viewer &viewer) -> bool
    {
        pbd_viewer.update_vertex_positions(obj.ID_, obj.V_);
        return false;
    };

    pbd_viewer.launch_rendering();
    return 0;
}
