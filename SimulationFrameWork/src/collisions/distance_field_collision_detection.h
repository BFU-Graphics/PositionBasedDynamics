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
    };
}

#endif //POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H
