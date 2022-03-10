#include "distance_field_collision_detection.h"

void HINASIM::DistanceFieldCollisionDetection::collision_detection()
{

}

//
//void HINASIM::DistanceFieldCollisionDetection::collision_detection()
//{
//    std::vector<std::pair<unsigned int, unsigned int>> collider_pairs;
//    for (int i = 0; i < collision_objects_.size(); ++i)
//        for (int j = 0; j < collision_objects_.size(); ++j)
//            if (i != j)
//                // TODO: self collisions for deformables in the future
//                collider_pairs.emplace_back(i, j);
//#ifdef USE_OPENMP
//#pragma omp parallel for
//#endif
//    for (int i = 0; i < collision_objects_.size(); ++i)
//    {
//        auto *co = collision_objects_[i];
//
//        if (dynamic_cast<DistanceFieldCollisionObject *>(co))
//        {
//            auto *dfco = dynamic_cast<DistanceFieldCollisionObject *>(co);
//            // TODO: update AABB
//
//            // TODO: update BVH
//        }
//    }
//
//
//    std::vector<std::vector<ContactData> > contacts_mt; // multi-thread contacts
//#ifdef USE_OPENMP
//#ifdef _DEBUG
//    const unsigned int maxThreads = 1;
//#else
//    const unsigned int maxThreads = omp_get_max_threads();
//#endif
//    contacts_mt.resize(maxThreads);
//#pragma omp parallel for
//#endif
//    for (int i = 0; i < collider_pairs.size(); ++i)
//    {
//        std::pair<unsigned int, unsigned int> &collider_pair = collider_pairs[i];
//
//        // collision detection rb silid
//    }
//}