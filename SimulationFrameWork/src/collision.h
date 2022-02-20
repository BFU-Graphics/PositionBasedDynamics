/**
 * @author YC XIANG.
 * @date 02/19, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_COLLISION_H
#define POSITIONBASEDDYNAMICS_COLLISION_H

#include "RenderingFrameWork/eigen_types.h"

#include <limits>

namespace HINASIM
{
    enum class CollisionObjectType
    {
        RigidBodyCollision,
        TriangleModelCollision,
        TetrahedralModelCollision,
    };

    class AABB
    {
    public:
        explicit AABB(const Eigen::MatrixXd &V)
        {
            aabb_ << std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()
                    , std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min();

            for (int i = 0; i < V.rows(); ++i)
            {
                if (aabb_.row(0)(0) > V.row(i)(0)) aabb_.row(0)(0) = V.row(i)(0);
                if (aabb_.row(0)(1) > V.row(i)(1)) aabb_.row(0)(1) = V.row(i)(1);
                if (aabb_.row(0)(2) > V.row(i)(2)) aabb_.row(0)(2) = V.row(i)(2);
                if (aabb_.row(1)(0) < V.row(i)(0)) aabb_.row(1)(0) = V.row(i)(0);
                if (aabb_.row(1)(1) < V.row(i)(1)) aabb_.row(1)(1) = V.row(i)(1);
                if (aabb_.row(1)(2) < V.row(i)(2)) aabb_.row(1)(2) = V.row(i)(2);
            }
        };

        Eigen::Matrix23d aabb_;

        static FORCE_INLINE bool intersection(const AABB &a1, const AABB &a2)
        {
            for (int i = 0; i < 3; ++i)
            {
                const double min0 = a1.aabb_.row(0)(i);
                const double max0 = a1.aabb_.row(1)(i);
                const double min1 = a2.aabb_.row(0)(i);
                const double max1 = a2.aabb_.row(1)(i);
                if (((max0 < min1) || (min0 > max1)))
                    return true;
            }
            return false;
        }
    };

    class CollisionDetection
    {

    public:

        

    public:
        struct CollisionObject
        {
            CollisionObject(const Eigen::MatrixXd &V, CollisionObjectType type) : aabb_(AABB(V)), type_(type)
            {}

            CollisionObjectType type_;
            using ContactCallbackFunction = void (*)();
            using SolidContactCallbackFunction = void (*)();
            AABB aabb_;
        };

        std::vector<CollisionObject> collision_objects_;
    };

    class DistanceFieldCollisionDetection
    {
    public:

    };
}


#endif //POSITIONBASEDDYNAMICS_COLLISION_H
