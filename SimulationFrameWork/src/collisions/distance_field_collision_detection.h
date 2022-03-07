#ifndef POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H
#define POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H

#include "collision_detection.h"
#include "bounding_sphere_hierarchy.h"

namespace HINASIM
{
    class DistanceFieldCollisionObject : public CollisionObject
    {
    public:
        explicit DistanceFieldCollisionObject(class SimObject *object);

        virtual ~DistanceFieldCollisionObject() = default;

    public:
        class SimObject *object_; // the object that

        PointCloudBSH bvh_;

        bool invertSDF_ = false;

    public:

        virtual double distance(const Eigen::Vector3d &x, double tolerance) = 0;
    };

    class DistanceFieldCollisionSphere : public DistanceFieldCollisionObject
    {
    public:
//        explicit DistanceFieldCollisionSphere();

    public:
        double radius_;

    public:
        double distance(const Eigen::Vector3d &x, double tolerance) override;
    };

    class DistanceFieldCollisionDetection : public CollisionDetection
    {
    public:
        DistanceFieldCollisionDetection();

        ~DistanceFieldCollisionDetection() override = default;

    public:
        void collision_detection() override;

    public:
        struct ContactData
        {

        };
    };
}
#endif //POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H
