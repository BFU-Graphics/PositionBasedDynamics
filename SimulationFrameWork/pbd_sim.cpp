/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "pbd_sim.h"

#include "utils/performance_check.h"

#include <thread>
#include <iostream>

void HINASIM::PBDSim::simulate(double dt)
{
    auto start = std::chrono::steady_clock::now();

    pbd_kernel_loop(dt);

    update_all_rendering_state();

    HINAVIEWER::INSPECTOR::Timeable::simulation_time_ += dt;
    HINAVIEWER::INSPECTOR::Timeable::physics_rate = dt;
    HINAVIEWER::INSPECTOR::Timeable::physics_runtime = HINASIM::UTILS::since(start).count();

//    std::cout << "Physics Rate(ms)=" << HINAVIEWER::INSPECTOR::Timeable::physics_runtime << "ms" << std::endl;
}

void HINASIM::PBDSim::simulate_real_dt()
{
    auto start = std::chrono::steady_clock::now();
    simulate(fixed_dt);
    long long rest_time = static_cast<long long>(fixed_dt * 1000) - HINASIM::UTILS::since(start).count();

    if (rest_time > 0)
    {
        std::this_thread::sleep_for(std::chrono::microseconds((int) (rest_time)));
    }
}

void HINASIM::PBDSim::add_object(HINASIM::SimObject *object)
{
    objects_.emplace_back(object);
}

void HINASIM::PBDSim::update_all_rendering_state()
{
    for (auto &o: objects_)
    {
        o->update_geometry_info();
    }
}

void HINASIM::PBDSim::pbd_kernel_loop(double dt)
{
    for (auto &o: objects_)
    {
        o->p_.setZero();

        // (5) forall vertices i do v_i <- v_i + \Delta t * w_i * f_external
        integrate_velocity_by_gravity(o->qdot_, o->inv_mass_, dt);

        // (6) damping velocities v_i
        damping_velocity(o->qdot_);

        // (7) forall vertices i do p_i <- x_i + \Delta t * v_i
        predict_position(o->p_, o->q_, o->qdot_, dt);

        // (8) forall vertices i do generateCollisionConstraints(x_i → p_i)
        generate_collision_constraints(); // pass

        // (9) ~ (11)
        // loop solverIterations times
        // projectConstraints(C_1,...,C_M+Mcoll ,p_1,...,p_N)
        // end loop
        for (int i = 0; i < 50; ++i)
        {
            constraints_projection(o);
        }

        // (12) ~ (15) forall vertices i
        // v_i <- (p_i - x_i) / \Delta t
        // x_i <- p_i
        update_q_and_qdot(o->q_, o->qdot_, o->p_, dt);
    }
}

void HINASIM::PBDSim::integrate_velocity_by_gravity(Eigen::Ref<Eigen::VectorXd> qdot, Eigen::Ref<Eigen::VectorXd> inv_mass, double dt)
{
    for (int i = 0; i < qdot.size(); i += 3)
    {
        if (inv_mass(i / 3) != 0)
            qdot.segment<3>(i) += dt * gravity;
    }
}

void HINASIM::PBDSim::damping_velocity(Eigen::Ref<Eigen::VectorXd> qdot)
{
    qdot *= 0.9999;
}

void HINASIM::PBDSim::predict_position(Eigen::Ref<Eigen::VectorXd> p, Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> qdot, double dt)
{
    p = q + dt * qdot;
}

void HINASIM::PBDSim::generate_collision_constraints()
{

}

void HINASIM::PBDSim::constraints_projection(SimObject *o)
{
    for (auto &c: o->inner_constraints_)
    {
        c->solve(o->p_, o->M_inv, 1 /** temp stiffness **/);
    }
}

void HINASIM::PBDSim::update_q_and_qdot(Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> qdot, Eigen::Ref<Eigen::VectorXd> p, double dt)
{
    qdot = (p - q) / dt;
    q = p;
}

HINASIM::PBDSim &HINASIM::PBDSim::fix_dt(double dt)
{
    fixed_dt = dt;
    return *this;
}
