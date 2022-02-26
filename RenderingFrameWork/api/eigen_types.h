/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_EIGEN_TYPES_H
#define POSITIONBASEDDYNAMICS_EIGEN_TYPES_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace Eigen
{

    //dense types
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix23d = Eigen::Matrix<double, 2, 3>;
    using Matrix36d = Eigen::Matrix<double, 3, 6>;
    using Matrix66d = Eigen::Matrix<double, 6, 6>;
    using Matrix44f = Eigen::Matrix<float, 4, 4>;
    //sparse types
    using SparseMatrixd = Eigen::SparseMatrix<double>;

}

#if defined(WIN32) || defined(_WIN32) || defined(WIN64)
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE __attribute__((always_inline))
#endif

#endif //POSITIONBASEDDYNAMICS_EIGEN_TYPES_H
