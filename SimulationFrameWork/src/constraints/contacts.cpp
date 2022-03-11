#include "contacts.h"

#include "src/objects/rigidbody.h"

#ifdef USE_OPENMP
#include "omp.h"
#endif

void compute_matrix_K(const Eigen::Vector3d &connector,
                      const double invMass,
                      const Eigen::Vector3d &x,
                      const Eigen::Matrix3d &inertiaInverseW,
                      Eigen::Matrix3d &K)
{
    if (invMass != 0.0)
    {
        const Eigen::Vector3d v = connector - x;
        const double a = v[0];
        const double b = v[1];
        const double c = v[2];

        // J is symmetric
        const double j11 = inertiaInverseW(0, 0);
        const double j12 = inertiaInverseW(0, 1);
        const double j13 = inertiaInverseW(0, 2);
        const double j22 = inertiaInverseW(1, 1);
        const double j23 = inertiaInverseW(1, 2);
        const double j33 = inertiaInverseW(2, 2);

        K(0, 0) = c * c * j22 - b * c * (j23 + j23) + b * b * j33 + invMass;
        K(0, 1) = -(c * c * j12) + a * c * j23 + b * c * j13 - a * b * j33;
        K(0, 2) = b * c * j12 - a * c * j22 - b * b * j13 + a * b * j23;
        K(1, 0) = K(0, 1);
        K(1, 1) = c * c * j11 - a * c * (j13 + j13) + a * a * j33 + invMass;
        K(1, 2) = -(b * c * j11) + a * c * j12 + a * b * j13 - a * a * j23;
        K(2, 0) = K(0, 2);
        K(2, 1) = K(1, 2);
        K(2, 2) = b * b * j11 - a * b * (j12 + j12) + a * a * j22 + invMass;
    } else
        K.setZero();
}

void
HINASIM::RigidBodyContactConstraint::add_contact(HINASIM::RigidBody *rb1, HINASIM::RigidBody *rb2, const Eigen::Vector3d &contact_point1, const Eigen::Vector3d &contact_point2, const Eigen::Vector3d &contact_normal, double contact_distance,
                                                 double restitution, double friction)
{
    // compute goal velocity in normal direction after collision
    const Eigen::Vector3d r0 = contact_point1 - rb1->x_;
    const Eigen::Vector3d r1 = contact_point2 - rb2->x_;
    const Eigen::Vector3d u0 = rb1->v_ + rb1->omega_.cross(r0);
    const Eigen::Vector3d u1 = rb2->v_ + rb2->omega_.cross(r1);
    const Eigen::Vector3d u_rel = u0 - u1;
    const double u_rel_n = contact_normal.dot(u_rel);
    double goal_velocity_normal = (u_rel_n < 0.0) ? (-restitution * u_rel_n) : 0.0;

    // tangent direction
    Eigen::Vector3d t = u_rel - u_rel_n * contact_normal;
    double tl2 = t.squaredNorm();
    if (tl2 > 1.0e-6)
        t *= 1.0 / sqrt(tl2);
    Eigen::Vector3d contact_tangent = t;


    Eigen::Matrix3d K1, K2;
    compute_matrix_K(contact_point1, rb1->inv_mass_, rb1->x_, rb1->inv_inertia_tensor_world_, K1);
    compute_matrix_K(contact_point2, rb2->inv_mass_, rb2->x_, rb2->inv_inertia_tensor_world_, K2);
    Eigen::Matrix3d K = K1 + K2;

    double max_impulse_tangent = 1.0 / (t.dot(K * t)) * u_rel.dot(t);

    rb_rb_contacts_.emplace_back(std::make_tuple(
            rb1,
            rb2,
            contact_point1,
            contact_point2,
            contact_normal,
            contact_tangent,
            1.0 / (contact_normal.dot(K * contact_normal)),
            max_impulse_tangent,
            goal_velocity_normal,
            friction,
            0.0 // sum impulse for output
    ));
}

bool HINASIM::RigidBodyContactConstraint::solve()
{
#ifdef USE_OPENMP
    omp_set_num_threads(omp_get_max_threads());
#pragma omp parallel for
#endif
    for (int i = 0; i < rb_rb_contacts_.size(); ++i)
    {
        RigidBody *rb1 = std::get<0>(rb_rb_contacts_[i]);
        RigidBody *rb2 = std::get<1>(rb_rb_contacts_[i]);

        if (rb1->inv_mass_ == 0.0 && rb2->inv_mass_ == 0.0)
            return false;

        Eigen::Vector3d contact_point1 = std::get<2>(rb_rb_contacts_[i]);
        Eigen::Vector3d contact_point2 = std::get<3>(rb_rb_contacts_[i]);
        Eigen::Vector3d contact_normal = std::get<4>(rb_rb_contacts_[i]);
        Eigen::Vector3d contact_tangent = std::get<5>(rb_rb_contacts_[i]);
        const double nKn_inv = std::get<6>(rb_rb_contacts_[i]);
        const double p_max = std::get<7>(rb_rb_contacts_[i]);
        const double goal_u_rel_n = std::get<8>(rb_rb_contacts_[i]);
        const double friction = std::get<9>(rb_rb_contacts_[i]);
        double sum_impulse = std::get<10>(rb_rb_contacts_[i]); // for output, inited by 0.0


        const Eigen::Vector3d r1 = contact_point1 - rb1->x_;
        const Eigen::Vector3d r2 = contact_point2 - rb2->x_;
        const Eigen::Vector3d u1 = rb1->v_ + rb1->omega_.cross(r1);
        const Eigen::Vector3d u2 = rb2->v_ + rb2->omega_.cross(r2);
        const Eigen::Vector3d u_rel = u1 - u2;
        const double u_rel_n = u_rel.dot(contact_normal);
        const double delta_u_rel_n = goal_u_rel_n - u_rel_n;
        double correctionMagnitude = nKn_inv * delta_u_rel_n;

        if (correctionMagnitude < -sum_impulse)
            correctionMagnitude = -sum_impulse; // collision contact may be solved multiple times, so sum_impulse would be non-zero

        // penetration depth
        const double d = contact_normal.dot(contact_point1 - contact_point2);
        if (d < 0.0)
            correctionMagnitude -= stiffness_ * nKn_inv * d;

        Eigen::Vector3d p(correctionMagnitude * contact_normal);
        sum_impulse += correctionMagnitude;

        // dynamic friction
        const double pn = p.dot(contact_normal);
        if (friction * pn > p_max)
            p -= p_max * contact_tangent;
        else if (friction * pn < -p_max)
            p += p_max * contact_tangent;
        else
            p -= friction * pn * contact_tangent;

        if (rb1->inv_mass_ > 0.0)
        {
            rb1->v_ += rb1->inv_mass_ * p;
            rb1->omega_ += rb1->inv_inertia_tensor_world_ * (r1.cross(p));
        }

        if (rb2->inv_mass_ > 0.0)
        {
            rb2->v_ += rb2->inv_mass_ * -p;
            rb2->omega_ += rb2->inv_inertia_tensor_world_ * (r2.cross(-p));
        }

        std::get<10>(rb_rb_contacts_[i]) = sum_impulse;
    }
    return true;
}
