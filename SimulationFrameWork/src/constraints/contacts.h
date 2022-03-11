#ifndef POSITIONBASEDDYNAMICS_CONTACTS_H
#define POSITIONBASEDDYNAMICS_CONTACTS_H

#include <Eigen/Dense>

#include <vector>
#include <tuple>

namespace HINASIM
{
    class ContactConstraint
    {
    public:
        virtual bool solve() = 0;

    public:
        double stiffness_ = 1.0;
    };

    class RigidBodyContactConstraint : public ContactConstraint
    {
    public:
        void add_contact(class RigidBody *rb1, class RigidBody *rb2, const Eigen::Vector3d& contact_point1, const Eigen::Vector3d& contact_point2, const Eigen::Vector3d& contact_normal, double contact_distance, double restitution,
                         double friction);
        bool solve() override;

    private:
        std::vector<std::tuple<
                class RigidBody *, // RigidBody 1
                class RigidBody *, // RigidBody 2
                Eigen::Vector3d, // contact point 1
                Eigen::Vector3d, // contact point 2
                Eigen::Vector3d, // contact normal
                Eigen::Vector3d, // contact tangent
                double, // 1.0 / normal^T * K * normal
                double, // max impulse in tangent direction
                double, // goal velocity in normal direction after collision
                double, // restitution
                double // friction
        >> rb_rb_contacts_;
    };
}

#endif //POSITIONBASEDDYNAMICS_CONTACTS_H
