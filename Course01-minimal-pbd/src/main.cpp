#include <igl/opengl/glfw/Viewer.h>

#include <igl/readOBJ.h>
#include <igl/readPLY.h>
#include <igl/edges.h>

#include "Utils/visualization/eigen_types.h"
#include "Utils/visualization/viewer.h"
#include "Utils/pbd_log.h"
#include "Utils/performace_check.h"

#include "constraints.h"

#include <iostream>
#include <string>

bool pause = false;
Eigen::VectorXd q;
Eigen::VectorXd qdot;
Eigen::SparseMatrix<double> M;
Eigen::SparseMatrix<double> M_inv;
double t = 0; //simulation time
double dt = 0.005; //time step
double k = 1;
double k_selected = 1e5; //stiff spring for pulling on object
double m = 1.;

pbd_src::DistanceConstraint *distance_constraint;

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
        g[i + 1] = -9.8 * dt;
    }

    Eigen::VectorXd p;
    Eigen::VectorXd dp;
    p.resize(q.rows());
    dp.resize(q.rows());
    while (!pause)
    {
        auto start = std::chrono::steady_clock::now();

        dp.setZero();

        // (5) forall vertices i do v_i <- v_i + \Delta t * w_i * f_external
        qdot = qdot + dt * M_inv * g;

        // (6) damping velocities v_i
        qdot *= 0.999;

        // (7) forall vertices i do p_i <- x_i + \Delta t * v_i
        p = q + dt * qdot;


        // (8) forall vertices i do generateCollisionConstraints(x_i → p_i)


        // (9) ~ (11)
        // loop solverIterations times
        // projectConstraints(C_1,...,C_M+Mcoll ,p_1,...,p_N)
        // end loop
        for (int i = 0; i < 5; ++i)
        {
            distance_constraint->solve(q, M_inv, dp, k);
        }


        // (12) ~ (15) forall vertices i
        // v_i <- (p_i - x_i) / \Delta t
        // x_i <- p_i
//        qdot = dp / dt;
        q = p + dp;
//        std::cout << dq << std::endl;


        // (16) velocityUpdate(v_1,...,v_N)





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
    // Phase I: Load Resources ================================================================================
    std::string cube_path = std::string(PBD_MODEL_DIR) + "cube.obj";
    std::string bunny_path = std::string(PBD_MODEL_DIR) + "bun_zipper_res3.ply";
    std::string mitsuba_path = std::string(PBD_TEXTURE_DIR) + "mitsuba.png";

    igl::readOBJ(cube_path, V, F);
    igl::edges(F, E);

    pbd_viewer::add_object_to_scene(V, F, Eigen::RowVector3d(244, 165, 130) / 255.);

    // Phase II: Init Physics State ================================================================================
    q.resize(V.rows() * V.cols());
    qdot.resize(V.rows() * V.cols());

    Eigen::MatrixXd Vt = V.transpose();
    q = Eigen::Map<Eigen::VectorXd>(Vt.data(), Vt.rows() * Vt.cols());
    qdot.setZero();
    M.resize(V.rows() * V.cols(), V.rows() * V.cols());
    typedef Eigen::Triplet<double> T;
    std::vector<T> tl_M;
//    M.setZero();
//    for (int i = 0; i < q.rows(); ++i)
//    {
//        if ((i % 3) == 0)
//            continue;
//        tl_M.emplace_back(i, i, 1);
//    }
//    M.setFromTriplets(tl_M.begin(), tl_M.end());
    M.setIdentity();
    M_inv = M;
    distance_constraint = new pbd_src::DistanceConstraint(q, E);

    pbd_viewer::setup(q, qdot, false);

    // Phase III: Init Physics State ================================================================================
    std::thread simulation_thread(simulate);
    simulation_thread.detach();

    pbd_viewer::viewer().callback_post_draw = &draw;
    pbd_viewer::viewer().launch_init(true, false, "Hello Minimal PBD", 0, 0);
    pbd_viewer::viewer().launch_rendering(true);
    pause = true;
    pbd_viewer::viewer().launch_shut();



















//    pbd_util::log(V, "vertices");
//    pbd_util::log(F, "faces");
//    pbd_util::log(E, "edges");
}
