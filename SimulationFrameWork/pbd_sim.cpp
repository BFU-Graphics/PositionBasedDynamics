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

    HINAVIEWER::INSPECTOR::Timeable::simulation_time_ += dt;
    HINAVIEWER::INSPECTOR::Timeable::physics_rate = dt;
    HINAVIEWER::INSPECTOR::Timeable::physics_runtime = HINASIM::UTILS::since(start).count();
}

void HINASIM::PBDSim::simulate_real_dt()
{
    auto start = std::chrono::steady_clock::now();
    simulate(fixed_dt);
    long long rest_time = static_cast<long long>(fixed_dt * 1000) - HINASIM::UTILS::since(start).count();

    if (rest_time > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(rest_time));
    }
}

void HINASIM::PBDSim::add_object(HINASIM::SimObject *object)
{
    object->init_physics_states();
    objects_.emplace_back(object);
}

void HINASIM::PBDSim::update_all_rendering_state()
{
    for (auto &o: objects_)
    {
        o->update_rendering_info();
        o->update_mouse_drag_force();
    }
}

void HINASIM::PBDSim::pbd_kernel_loop(double dt)
{
    for (auto &o: objects_)
    {
        // calc external force
        external_force(o);

        // (5) ~ (7)
        // forall vertices i do v_i <- v_i + \Delta t * w_i * f_external
        // damping velocities v_i
        // forall vertices i do p_i <- x_i + \Delta t * v_i
        integrate_prediction_with_damping(o, dt);
    }

    // (8) forall vertices i do generateCollisionConstraints(x_i → p_i)
    generate_collision_constraints(); // omit

    // (9) ~ (11)
    // loop solverIterations times
    // projectConstraints(C_1,...,C_M+Mcoll ,p_1,...,p_N)
    // end loop
    project_position_constraints();

    // (12) ~ (15) forall vertices i
    // v_i <- (p_i - x_i) / \Delta t
    // x_i <- p_i
    for (auto &o: objects_)
    {
        update_positions_and_velocities(o, dt);
    }
}

void HINASIM::PBDSim::external_force(HINASIM::SimObject *o)
{
    if (o->TYPE_ == SimObjectType::Deformable)
    {
        auto *deformable = dynamic_cast<HINASIM::DeformableObject *>(o);
        deformable->a_.setZero();
        for (int i = 0; i < deformable->a_.rows(); ++i)
            if (deformable->inv_mass_(i) != 0)
                deformable->a_.row(i) = gravity_ + deformable->inv_mass_(i) * deformable->mouse_drag_force_.row(i);
    }
}

void HINASIM::PBDSim::integrate_prediction_with_damping(HINASIM::SimObject *o, double dt, double damping)
{
    if (o->TYPE_ == SimObjectType::Deformable)
    {
        auto *deformable = dynamic_cast<HINASIM::DeformableObject *>(o);
        deformable->p_.setZero();
        HINASIM::TimeIntegration::semi_implicit_integration_with_damping(dt, deformable->inv_mass_, deformable->p_, deformable->x_, deformable->v_, deformable->a_, damping);
    }
}

void HINASIM::PBDSim::generate_collision_constraints()
{

}

void HINASIM::PBDSim::project_position_constraints()
{
    for (auto &o: objects_)
    {
        if (o->TYPE_ == SimObjectType::Deformable)
        {
            auto *deformable = dynamic_cast<HINASIM::DeformableObject *>(o);
            for (int i = 0; i < 5; ++i)
            {
                for (auto &c: deformable->inner_constraints_)
                {
                    c->solve(deformable->p_, deformable->inv_mass_);
                }
            }
        }
    }
}

void HINASIM::PBDSim::update_positions_and_velocities(HINASIM::SimObject *o, double dt)
{
    if (o->TYPE_ == SimObjectType::Deformable)
    {
        auto *deformable = dynamic_cast<HINASIM::DeformableObject *>(o);
        HINASIM::TimeIntegration::velocity_update_first_order(dt, deformable->inv_mass_, deformable->p_, deformable->x_, deformable->v_);
        deformable->x_ = deformable->p_;
    }
}