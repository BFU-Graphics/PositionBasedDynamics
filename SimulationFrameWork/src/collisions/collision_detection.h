#ifndef POSITIONBASEDDYNAMICS_COLLISION_DETECTION_H
#define POSITIONBASEDDYNAMICS_COLLISION_DETECTION_H

#include "colliders.h"

#include <Eigen/Dense>
#include <vector>

namespace HINASIM
{
    enum class ContactType
    {
        RigidBody,
        Particle,
        ParticleRigidBody,
        ParticleSolid,
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
        virtual void collision_detection() = 0;

    protected:
        std::vector<Collider *> colliders_;
        double tolerance_;
    };
}


#endif //POSITIONBASEDDYNAMICS_COLLISION_DETECTION_H
