#pragma once

#include <Tools/Include.h>
#include <Tools/EigenInc.h>

using Eigen::Array;

typedef float real; // ~20% faster than double 
using Vec3 = Eigen::Matrix<real, 3, 1>;
using VecX = Eigen::Matrix<real, Eigen::Dynamic, 1>;
using MatX = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;

// forward declarations:
