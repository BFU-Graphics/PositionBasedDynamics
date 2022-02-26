#ifndef POSITIONBASEDDYNAMICS_AABB_H
#define POSITIONBASEDDYNAMICS_AABB_H

#include <Eigen/Dense>

#if defined(WIN32) || defined(_WIN32) || defined(WIN64)
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE __attribute__((always_inline))
#endif

namespace Eigen
{
    using Matrix23d = Matrix<double, 2, 3, DontAlign>;
}

namespace HINASIM
{
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
}

#endif //POSITIONBASEDDYNAMICS_AABB_H
