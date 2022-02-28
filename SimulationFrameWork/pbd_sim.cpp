#include "pbd_sim.h"

#include "RenderingFrameWork/src/inspector.h"
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

HINASIM::PBDSim &HINASIM::PBDSim::add_object(HINASIM::SimObject *object)
{
    object->init_physics_states();
    objects_.emplace_back(object);
    return *this;
}

HINASIM::PBDSim &HINASIM::PBDSim::add_collider(HINASIM::CollisionObject *collider)
{
    colliders_.emplace_back(collider);
    return *this;
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
    generate_collision_constraints(); // omit it by now

    // (9) ~ (11)
    // loop solverIterations times
    // projectConstraints(C_1,...,C_M+M_coll ,p_1,...,p_N)
    // end loop
    project_position_constraints();

    //////////////////// TEMPORARY COLLISION RESPONSE ////////////////////
    collision_response();
    //////////////////// TEMPORARY COLLISION RESPONSE ////////////////////

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
    switch (o->TYPE_)
    {
        case SimObjectType::Deformable:
        {
            auto *deformable = dynamic_cast<HINASIM::DeformableObject *>(o);
            deformable->a_.setZero();
            for (int i = 0; i < deformable->a_.rows(); ++i)
                if (deformable->inv_mass_(i) != 0)
                    deformable->a_.row(i) = gravity_ + deformable->inv_mass_(i) * deformable->mouse_drag_force_.row(i);
        }
            break;
        case SimObjectType::RigidBody:
        {
            // TODO:
            std::cerr << "Rigid Body Not Implemented yet" << std::endl;
        }
            break;
        case SimObjectType::Fluid:
        {
            // TODO:
            std::cerr << "Fluid Not Implemented yet" << std::endl;
        }
            break;
        default:
            break;
    }
}

void HINASIM::PBDSim::integrate_prediction_with_damping(HINASIM::SimObject *o, double dt, double damping)
{
    switch (o->TYPE_)
    {
        case SimObjectType::Deformable:
        {
            auto *deformable = dynamic_cast<HINASIM::DeformableObject *>(o);
            deformable->p_.setZero();
            HINASIM::TimeIntegration::semi_implicit_integration_with_damping(dt, deformable->inv_mass_, deformable->p_, deformable->x_, deformable->v_, deformable->a_, damping);
        }
            break;
        case SimObjectType::RigidBody:
        {
            // TODO:
            std::cerr << "Rigid Body Not Implemented yet" << std::endl;
        }
            break;
        case SimObjectType::Fluid:
        {
            // TODO:
            std::cerr << "Fluid Not Implemented yet" << std::endl;
        }
        default:
            break;
    }
}

void HINASIM::PBDSim::generate_collision_constraints()
{
    // TODO:
}

void HINASIM::PBDSim::project_position_constraints()
{
    // solve internal constraints(aka. joints)
    for (auto &j: joints_)
    {
        // TODO:
    }

    // solve inner constraints
    for (auto &o: objects_)
    {
        switch (o->TYPE_)
        {
            case SimObjectType::Deformable:
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
                break;
            case SimObjectType::RigidBody:
            {
                // TODO:
                std::cerr << "Rigid Body Not Implemented yet" << std::endl;
            }
                break;
            case SimObjectType::Fluid:
            {
                // TODO:
                std::cerr << "Fluid Not Implemented yet" << std::endl;
            }
            default:
                break;
        }
    }
}

void HINASIM::PBDSim::update_positions_and_velocities(HINASIM::SimObject *o, double dt)
{
    switch (o->TYPE_)
    {
        case SimObjectType::Deformable:
        {
            auto *deformable = dynamic_cast<HINASIM::DeformableObject *>(o);

            HINASIM::TimeIntegration::velocity_update_first_order(dt, deformable->inv_mass_, deformable->p_, deformable->x_, deformable->v_);
            deformable->x_ = deformable->p_;
        }
            break;
        case SimObjectType::RigidBody:
        {
            // TODO:
            std::cerr << "Rigid Body Not Implemented yet" << std::endl;
        }
            break;
        case SimObjectType::Fluid:
        {
            // TODO:
            std::cerr << "Fluid Not Implemented yet" << std::endl;
        }
        default:
            break;
    }

    o->update_physics_info();
}

void HINASIM::PBDSim::collision_response()
{
    for (auto &o: objects_)
    {
        switch (o->TYPE_)
        {
            case SimObjectType::Deformable:
            {
                auto *deformable = dynamic_cast<HINASIM::DeformableObject *>(o);

                for (auto &collider: colliders_)
                {
                    switch (collider->TYPE_)
                    {
                        case CollisionObjectType::Sphere:
                        {
                            auto sc = dynamic_cast<SphereCollider *>(collider);
                            collision_response_to_a_sphere(deformable, sc->position_, sc->radius_);
                        }
                            break;
                        case CollisionObjectType::Box:
                        {
                        }
                            break;
                        default:
                            break;
                    }
                }
            }
                break;
            case SimObjectType::RigidBody:
            {
                // TODO:
                std::cerr << "Rigid Body Not Implemented yet" << std::endl;
            }
                break;
            case SimObjectType::Fluid:
            {
                // TODO:
                std::cerr << "Fluid Not Implemented yet" << std::endl;
            }
            default:
                break;
        }
    }
}

void HINASIM::PBDSim::collision_response_to_a_sphere(HINASIM::DeformableObject *deformable, const Eigen::Vector3d &center, double radius)
{
    const double eps = 1e-2;
    radius = radius + eps; // a little larger sphere
    for (int i = 0; i < deformable->p_.rows(); ++i)
    {
        if (deformable->inv_mass_(i) != 0)
        {
            Eigen::Vector3d arrow = deformable->p_.row(i).transpose() - center;
            Eigen::Vector3d unit_direction = arrow.normalized();
            double distance = arrow.norm();

            if (distance < radius)
                deformable->p_.row(i) = (unit_direction * radius).transpose();
        }
    }
}
