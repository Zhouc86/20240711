/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes
 * Eigen types, template types, aliases, ...
 */

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <vector>
#include <algorithm>
#include <memory>
#include "cTypes.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <limits>
#include <eigen3/Eigen/StdVector>

using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;


enum class LegName {LeftLeg = 1, RightLeg, Unknown};
enum class CoordinateAxis {X = 1, Y, Z, Unknown};
enum class SingleLegJointId {HipZ = 1, HipX, HipY, Knee, AnkleY, Unknown};
enum class WalkPhase {HANGING = 1, 
                    HANGING_TO_STANCE, 
                    DOUBLE_LEG_STANCE,
                    LEFT_LEG_SUPPORT_WALK, 
                    RIGHT_LEG_SUPPORT_WALK, 
                    UNKNOWN};

template <typename Type, int Size>
using Vector = Matrix<Type, Size, 1>;

using Matrix4d = Matrix<double, 4, 4>;

// Rotation Matrix
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 2x1 Vector
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

template <typename T>
using Vec5 = typename Eigen::Matrix<T, 5, 1>;

// 3x3 Matrix
template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// // 4x1 Vector
template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// // Spatial Vector (6x1, all subspaces)
template <typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// // Spatial Transform (6x6)
// template <typename T>
// using SXform = typename Eigen::Matrix<T, 6, 6>;

// // 6x6 Matrix
template <typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// // Dynamically sized vector
template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Dynamically sized matrix
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// Dynamically sized matrix with cartesian vector columns
template <typename T>
using D3Mat = typename Eigen::Matrix<T, 3, Eigen::Dynamic>;

union Byte8
{
    uint64_t udata;
    uint8_t buffer[8];
};

union Byte2
{
    int16_t data;
    uint16_t udata;
    uint8_t buffer[2];
};

union Byte4 {
    uint32_t udata;
    int32_t sdata;
    float fdata;
    uint8_t data[4];
};


#endif  // PROJECT_CPPTYPES_H
