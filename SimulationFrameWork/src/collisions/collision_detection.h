#ifndef POSITIONBASEDDYNAMICS_COLLISION_DETECTION_H
#define POSITIONBASEDDYNAMICS_COLLISION_DETECTION_H

#include <Eigen/Dense>

#include "aabb.h"

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

    class CollisionObject
    {
    public:
        explicit CollisionObject() = default;

        virtual ~CollisionObject() = default;

    public:
        AABB aabb;
        class SimObject *object_; // the object that
    };

    class CollisionDetection
    {

    protected:
        typedef void (*ContactCallbackFunction)(const ContactType contactType, const unsigned int bodyIndex1, const unsigned int bodyIndex2,
                                                const Eigen::Vector3d &cp1, const Eigen::Vector3d &cp2,
                                                const Eigen::Vector3d &normal, const double dist,
                                                const double restitutionCoeff, const double frictionCoeff, void *userData);

        typedef void (*SolidContactCallbackFunction)(const ContactType contactType, const unsigned int bodyIndex1, const unsigned int bodyIndex2,
                                                     const unsigned int tetIndex, const Eigen::Vector3d &bary,
                                                     const Eigen::Vector3d &cp1, const Eigen::Vector3d &cp2,
                                                     const Eigen::Vector3d &normal, const double dist,
                                                     const double restitutionCoeff, const double frictionCoeff, void *userData);

    public:
        double tolerance_;
        ContactCallbackFunction contactCB_;
        SolidContactCallbackFunction solidContactCB_;
        void *contactCBUserData_;
        void *solidContactCBUserData_;

        static void updateAABB(const Eigen::Vector3d &p, AABB &aabb);

    public:
        CollisionDetection();

        virtual void collision_detection() = 0; // TODO: param set

        virtual ~CollisionDetection();

        void cleanup();

        void addRigidBodyContact(unsigned int rbIndex1, unsigned int rbIndex2,
                                 const Eigen::Vector3d &cp1, const Eigen::Vector3d &cp2,
                                 const Eigen::Vector3d &normal, double dist,
                                 double restitutionCoeff, double frictionCoeff) const;

        void addParticleRigidBodyContact(unsigned int particleIndex, unsigned int rbIndex,
                                         const Eigen::Vector3d &cp1, const Eigen::Vector3d &cp2,
                                         const Eigen::Vector3d &normal, double dist,
                                         double restitutionCoeff, double frictionCoeff) const;

        void addParticleSolidContact(unsigned int particleIndex, unsigned int solidIndex,
                                     unsigned int tetIndex, const Eigen::Vector3d &bary,
                                     const Eigen::Vector3d &cp1, const Eigen::Vector3d &cp2,
                                     const Eigen::Vector3d &normal, double dist,
                                     double restitutionCoeff, double frictionCoeff) const;

        virtual void addCollisionObject(unsigned int bodyIndex, unsigned int bodyType);

        void setContactCallback(CollisionDetection::ContactCallbackFunction val, void *userData);

        void setSolidContactCallback(CollisionDetection::SolidContactCallbackFunction val, void *userData);

        // TODO: UPDATE AABBS










    public:
        std::vector<CollisionObject *> collision_objects_;
    };
}


#endif //POSITIONBASEDDYNAMICS_COLLISION_DETECTION_H
