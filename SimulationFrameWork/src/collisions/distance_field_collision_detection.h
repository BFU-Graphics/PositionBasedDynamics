#ifndef POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H
#define POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H

#include "collision_detection.h"

namespace HINASIM
{
    class DistanceFieldCollisionDetection : public CollisionDetection
    {
    public:
        DistanceFieldCollisionDetection() = default;
        ~DistanceFieldCollisionDetection() override = default;

    public:
        CollisionDetection &add_collider_sphere(class SimObject *o);

    public:
        void collision_detection() override;

    private:
        void collision_detection_RB_RB(class RigidBody *rb1, DistanceFieldCollider *co1, class RigidBody *rb2, DistanceFieldCollider *co2, std::vector<std::vector<ContactData>> &contacts_mt);
    };
}

#endif //POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H
