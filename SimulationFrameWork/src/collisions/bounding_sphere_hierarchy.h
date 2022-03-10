#ifndef POSITIONBASEDDYNAMICS_BOUNDING_SPHERE_HIERARCHY_H
#define POSITIONBASEDDYNAMICS_BOUNDING_SPHERE_HIERARCHY_H

#include "kdtree.h"
#include "bounding_sphere.h"

namespace HINASIM
{
    class PointCloudBSH : public KDTree<BoundingSphere>
    {
    public:
        PointCloudBSH();

    public:
        void init(const Eigen::MatrixXd &V);

    protected:
        [[nodiscard]] Eigen::Vector3d entity_position(unsigned int i) const override;
        void compute_hull(unsigned int b, unsigned int n, BoundingSphere &hull) const override;
        void compute_hull_approx(unsigned int b, unsigned int n, BoundingSphere &hull) const override;

    private:
        Eigen::MatrixXd V_;
    };
}


#endif //POSITIONBASEDDYNAMICS_BOUNDING_SPHERE_HIERARCHY_H
