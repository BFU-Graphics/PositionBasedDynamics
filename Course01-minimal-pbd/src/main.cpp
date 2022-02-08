#include <igl/opengl/glfw/Viewer.h>

#include <igl/readOBJ.h>
#include <igl/readPLY.h>
#include <igl/edges.h>

#include "Eigen/Eigen"

#include <iostream>
#include <string>

#include "Utils/visualization/viewer.h"
#include "Utils/pbd_log.h"

bool pause = true;
Eigen::VectorXd q;
Eigen::VectorXd qdot;
Eigen::SparseMatrix<double> M;
double t = 0; //simulation time
double dt = 0.005; //time step
double k = 1e5;
double k_selected = 1e5; //stiff spring for pulling on object
double m = 1.;


void simulate()
{
    while (!pause)
    {

    }
}

bool draw(igl::opengl::glfw::Viewer &viewer)
{
    return false; // stay false if success
}

int main(int argc, char *argv[])
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXi E;

    std::string cube_path = std::string(PBD_MODEL_DIR) + "cube.obj";
    std::string bunny_path = std::string(PBD_MODEL_DIR) + "bun_zipper_res3.ply";
    std::string mitsuba_path = std::string(PBD_TEXTURE_DIR) + "mitsuba.png";

//    igl::readPLY(cube_path, V, F);
    igl::readOBJ(cube_path, V, F);
    igl::edges(F, E);

    std::thread simulation_thread(simulate);
    simulation_thread.detach();

    q.resize(V.rows() * V.cols());
    qdot.resize(V.rows() * V.cols());

    Eigen::MatrixXd Vt = V.transpose();
    q = Eigen::Map<Eigen::VectorXd>(Vt.data(), Vt.rows() * Vt.cols());
    qdot.setZero();

    pbd_viewer::setup(q, qdot);
    pbd_viewer::add_object_to_scene(V, F, Eigen::RowVector3d(244, 165, 130) / 255.);
    pbd_viewer::viewer().callback_post_draw = &draw;

//    pbd_util::log(V, "vertices");
//    pbd_util::log(F, "faces");
//    pbd_util::log(E, "edges");

    pbd_viewer::viewer().launch_init(true, false, "Mass-Spring Systems", 0, 0);
    pbd_viewer::viewer().launch_rendering(true);
}
