#include <igl/opengl/glfw/Viewer.h>

#include <igl/readOBJ.h>
#include <igl/readPLY.h>
#include <igl/edges.h>

#include "Eigen/Eigen"

#include "Utils/visualization/viewer.h"
#include "Utils/pbd_log.h"
#include "Utils/performace_check.h"

#include <iostream>
#include <string>

bool pause = false;
Eigen::VectorXd q;
Eigen::VectorXd qdot;
Eigen::SparseMatrix<double> M;
double t = 0; //simulation time
double dt = 0.005; //time step
double k = 1e5;
double k_selected = 1e5; //stiff spring for pulling on object
double m = 1.;

Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::MatrixXi E;

void simulate()
{
    Eigen::VectorXd g;
    g.resize(q.rows());
    g.setZero();
    for (int i = 0; i < qdot.rows(); i += 3)
    {
        g[i + 1] = -0.98 * dt * 0.01;
    }

    while (!pause)
    {
        auto start = std::chrono::steady_clock::now();
        qdot = g;
        q = (q + (dt * qdot)).eval();
        t += dt;
        std::cout << "Physics Rate(ms)=" << pbd_util::since(start).count() << std::endl;
    }
}

bool draw(igl::opengl::glfw::Viewer &viewer)
{
    pbd_viewer::update_vertex_positions(0, q);
    return false; // stay false if success
}

int main(int argc, char *argv[])
{

    std::string cube_path = std::string(PBD_MODEL_DIR) + "cube.obj";
    std::string bunny_path = std::string(PBD_MODEL_DIR) + "bun_zipper_res3.ply";
    std::string mitsuba_path = std::string(PBD_TEXTURE_DIR) + "mitsuba.png";

//    igl::readPLY(bunny_path, V, F);
    igl::readOBJ(cube_path, V, F);
    igl::edges(F, E);

    q.resize(V.rows() * V.cols());
    qdot.resize(V.rows() * V.cols());

    Eigen::MatrixXd Vt = V.transpose();
    q = Eigen::Map<Eigen::VectorXd>(Vt.data(), Vt.rows() * Vt.cols());
    qdot.setZero();

    std::thread simulation_thread(simulate);
    simulation_thread.detach();

    pbd_viewer::setup(q, qdot, true);
    pbd_viewer::add_object_to_scene(V, F, Eigen::RowVector3d(244, 165, 130) / 255.);
    pbd_viewer::viewer().callback_post_draw = &draw;

    pbd_viewer::viewer().launch_init(true, false, "Mass-Spring Systems", 0, 0);
    pbd_viewer::viewer().launch_rendering(true);
    pause = true;
    pbd_viewer::viewer().launch_shut();



















//    pbd_util::log(V, "vertices");
//    pbd_util::log(F, "faces");
//    pbd_util::log(E, "edges");
}
