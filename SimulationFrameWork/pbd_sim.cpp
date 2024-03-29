#include "pbd_sim.h"

#include "RenderingFrameWork/src/inspector.h"
#include "utils/performance_check.h"

#include <thread>

HINASIM::PBDSim::~PBDSim()
{
    if (!objects_.empty())
        for (auto &o: objects_)
            delete o;
}

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
    simulate(fixed_dt_);
    long long rest_time = static_cast<long long>(fixed_dt_ * 1000) - HINASIM::UTILS::since(start).count();

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
        integrate_prediction_with_damping(o, dt, 0.99);
    }

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

    // (8) forall vertices i do generateCollisionConstraints(x_i → p_i)
    // collision detection
    generate_collision_constraints();

    project_velocity_constraints();
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
                if (deformable->inv_mass_(i) != 0.0)
                    deformable->a_.row(i) = gravity_ + deformable->inv_mass_(i) * deformable->mouse_drag_force_.row(i);
        }
            break;
        case SimObjectType::RigidBody:
        {
            auto *rigid_body = dynamic_cast<HINASIM::RigidBody *>(o);
            rigid_body->a_.setZero();
            rigid_body->t_.setZero();
            if (rigid_body->inv_mass_ != 0.0)
                rigid_body->a_ = gravity_; // TODO: allow drag force
        }
            break;
        case SimObjectType::Fluid:
        {
            // TODO: Fluid Not Implemented yet
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
            HINASIM::TimeIntegrationDeformable::semi_implicit_integration_with_damping(
                    dt,
                    deformable->inv_mass_,
                    deformable->p_,
                    deformable->x_,
                    deformable->v_,
                    deformable->a_,
                    damping
            );
        }
            break;
        case SimObjectType::RigidBody:
        {
            auto *rigid_body = dynamic_cast<HINASIM::RigidBody *>(o);
            rigid_body->p_x_.setZero();
            rigid_body->p_q_.setIdentity();
            HINASIM::TimeIntegrationRigidBody::semi_implicit_integration_with_damping(
                    dt,
                    rigid_body->inv_mass_,
                    rigid_body->p_x_,
                    rigid_body->x_,
                    rigid_body->v_,
                    rigid_body->a_,
                    rigid_body->inertia_tensor_world_,
                    rigid_body->inv_inertia_tensor_world_,
                    rigid_body->p_q_,
                    rigid_body->q_,
                    rigid_body->omega_,
                    rigid_body->t_,
                    damping
            );
            // TODO: maybe need to update inertia tensors
        }
            break;
        case SimObjectType::Fluid:
        {
            // TODO: Fluid Not Implemented yet
        }
        default:
            break;
    }
}

void HINASIM::PBDSim::generate_collision_constraints()
{
    if (collision_engine_)
        collision_engine_->collision_detection();
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
                for (int i = 0; i < 15; ++i) // solve multiple times to ensure convergence
                    for (auto &c: deformable->inner_constraints_)
                        c->solve(deformable->p_, deformable->inv_mass_);
            }
                break;
            case SimObjectType::RigidBody: // NOTE: Rigid Body normally DON'T have any inner constraint
                break;
            case SimObjectType::Fluid:
            {
                // TODO: Fluid Not Implemented yet
            }
            default:
                break;
        }
    }
}

void HINASIM::PBDSim::project_velocity_constraints()
{
    if (collision_engine_)
        for (int i = 0; i < 10; ++i)
            collision_engine_->contacts_solve();
}

void HINASIM::PBDSim::update_positions_and_velocities(HINASIM::SimObject *o, double dt)
{
    switch (o->TYPE_)
    {
        case SimObjectType::Deformable:
        {
            auto *deformable = dynamic_cast<HINASIM::DeformableObject *>(o);
            HINASIM::TimeIntegrationDeformable::velocity_update_first_order(
                    dt,
                    deformable->inv_mass_,
                    deformable->p_,
                    deformable->x_,
                    deformable->v_
            );
            deformable->x_ = deformable->p_;
        }
            break;
        case SimObjectType::RigidBody:
        {
            auto *rigid_body = dynamic_cast<HINASIM::RigidBody *>(o);
            HINASIM::TimeIntegrationRigidBody::velocity_update_first_order(
                    dt,
                    rigid_body->inv_mass_,
                    rigid_body->p_x_,
                    rigid_body->x_,
                    rigid_body->v_,
                    rigid_body->p_q_,
                    rigid_body->q_,
                    rigid_body->omega_
            );
            rigid_body->x_ = rigid_body->p_x_;
            rigid_body->q_ = rigid_body->p_q_;
        }
            break;
        case SimObjectType::Fluid:
        {
            // TODO: Fluid Not Implemented yet
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
                // TODO: Rigid Body Not Implemented yet
            }
                break;
            case SimObjectType::Fluid:
            {
                // TODO: Fluid Not Implemented yet
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

HINASIM::PBDSim &HINASIM::PBDSim::set_collision_engine(HINASIM::CollisionDetection *collision_engine)
{
    collision_engine_ = collision_engine;
    return *this;
}
