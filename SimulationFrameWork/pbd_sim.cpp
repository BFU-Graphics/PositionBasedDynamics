/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "pbd_sim.h"

#include "utils/performance_check.h"

#include <iostream>

void HINASIM::PBDSim::simulate(double dt)
{
    auto start = std::chrono::steady_clock::now();

    for (auto &o: objects_)
    {
        o->p_.setZero();
        integrate_velocity_by_gravity(o->qdot_, dt);
        damping_velocity(o->qdot_);
        predict_position(o->p_, o->q_, o->qdot_, dt);
        generate_collision_constraints(); // pass
        constraints_projection(o);
        update_q_and_qdot(o->q_, o->qdot_, o->p_, dt);
    }


    HINAVIEWER::INSPECTOR::Timeable::simulation_time_ += dt;

    std::cout << "Physics Rate(ms)=" << HINASIM::UTILS::since(start).count() << "ms" << std::endl;
}

void HINASIM::PBDSim::add_object(HINASIM::SimObject *object)
{
    objects_.emplace_back(object);
}

void HINASIM::PBDSim::integrate_velocity_by_gravity(Eigen::Ref<Eigen::VectorXd> qdot, double dt)
{
    for (int i = 0; i < qdot.size(); i += 3)
    {
        qdot.segment<3>(i) += dt * gravity;
    }
}

void HINASIM::PBDSim::damping_velocity(Eigen::Ref<Eigen::VectorXd> qdot)
{
    qdot *= 0.999;
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
