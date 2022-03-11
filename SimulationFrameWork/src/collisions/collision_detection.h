#ifndef POSITIONBASEDDYNAMICS_COLLISION_DETECTION_H
#define POSITIONBASEDDYNAMICS_COLLISION_DETECTION_H

#include "colliders.h"

#include "src/constraints/contacts.h"

#include <Eigen/Dense>
#include <vector>

namespace HINASIM
{
    enum class ContactType
    {
        RigidBody, // Rigid-Rigid Contact

        // TODO: support more contact types
    };

    struct ContactData
    {
        ContactType TYPE;
        class SimObject *object1;
        class SimObject *object2;
        Eigen::Vector3d contact_point1;
        Eigen::Vector3d contact_point2;
        Eigen::Vector3d normal;
        double distance;
        double restitution;
        double friction;
    };

    class CollisionDetection
    {
    public:
        CollisionDetection();
        virtual ~CollisionDetection();

    public:
        void add_rigid_body_contact(class RigidBody *rb1, class RigidBody *rb2, const Eigen::Vector3d &contact_point1, const Eigen::Vector3d &contact_point2, const Eigen::Vector3d &contact_normal, double contact_distance, double restitution,
                                    double friction);

    public:
        virtual void collision_detection() = 0;

    protected:
        std::vector<Collider *> colliders_;
        double tolerance_;

        std::vector<ContactConstraint *> contacts_; // for collision response, update every sim loop
    };
}

#endif //POSITIONBASEDDYNAMICS_COLLISION_DETECTION_H
