#include "collision_detection.h"

void HINASIM::CollisionDetection::updateAABB(const Eigen::Vector3d &p, HINASIM::AABB &aabb)
{
    if (aabb.aabb_[0](0) > p(0))
        aabb.aabb_[0](0) = p(0);
    if (aabb.aabb_[0](1) > p(1))
        aabb.aabb_[0](1) = p(1);
    if (aabb.aabb_[0](2) > p(2))
        aabb.aabb_[0](2) = p(2);
    if (aabb.aabb_[1](0) < p(0))
        aabb.aabb_[1](0) = p(0);
    if (aabb.aabb_[1](1) < p(1))
        aabb.aabb_[1](1) = p(1);
    if (aabb.aabb_[1](2) < p(2))
        aabb.aabb_[1](2) = p(2);
}

HINASIM::CollisionDetection::CollisionDetection()
{
    collision_objects_.reserve(1000);
    contactCB_ = nullptr;
    solidContactCB_ = nullptr;
    tolerance_ = 0.01;

    contactCBUserData_ = nullptr;
    solidContactCBUserData_ = nullptr;
}

HINASIM::CollisionDetection::~CollisionDetection()
{
    cleanup();
}

void HINASIM::CollisionDetection::cleanup()
{
    for (unsigned int i = 0; i < collision_objects_.size(); i++)
        delete collision_objects_[i];
    collision_objects_.clear();
}

void HINASIM::CollisionDetection::addRigidBodyContact(const unsigned int rbIndex1, const unsigned int rbIndex2,
                                                      const Eigen::Vector3d &cp1, const Eigen::Vector3d &cp2,
                                                      const Eigen::Vector3d &normal, const double dist,
                                                      const double restitutionCoeff, const double frictionCoeff) const
{
    if (contactCB_)
        contactCB_(ContactType::RigidBody, rbIndex1, rbIndex2, cp1, cp2, normal, dist, restitutionCoeff, frictionCoeff, contactCBUserData_);
}

void HINASIM::CollisionDetection::addParticleRigidBodyContact(const unsigned int particleIndex, const unsigned int rbIndex,
                                                              const Eigen::Vector3d &cp1, const Eigen::Vector3d &cp2,
                                                              const Eigen::Vector3d &normal, const double dist,
                                                              const double restitutionCoeff, const double frictionCoeff) const
{
    if (contactCB_)
        contactCB_(ContactType::ParticleRigidBody, particleIndex, rbIndex, cp1, cp2, normal, dist, restitutionCoeff, frictionCoeff, contactCBUserData_);
}

void HINASIM::CollisionDetection::addParticleSolidContact(const unsigned int particleIndex, const unsigned int solidIndex,
                                                          const unsigned int tetIndex, const Eigen::Vector3d &bary, const Eigen::Vector3d &cp1, const Eigen::Vector3d &cp2,
                                                          const Eigen::Vector3d &normal, const double dist, const double restitutionCoeff, const double frictionCoeff) const
{
    if (solidContactCB_)
        solidContactCB_(ContactType::ParticleSolid, particleIndex, solidIndex, tetIndex, bary, cp1, cp2, normal, dist, restitutionCoeff, frictionCoeff, contactCBUserData_);
}

void HINASIM::CollisionDetection::addCollisionObject(const unsigned int bodyIndex, const unsigned int bodyType)
{
//    CollisionObjectWithoutGeometry *co = new CollisionObjectWithoutGeometry();
//    co->m_bodyIndex = bodyIndex;
//    co->m_bodyType = bodyType;
//    m_collisionObjects.push_back(co);
}

void HINASIM::CollisionDetection::setContactCallback(CollisionDetection::ContactCallbackFunction val, void *userData)
{
    contactCB_ = val;
    contactCBUserData_ = userData;
}

void HINASIM::CollisionDetection::setSolidContactCallback(CollisionDetection::SolidContactCallbackFunction val, void *userData)
{
    solidContactCB_ = val;
    solidContactCBUserData_ = userData;
}