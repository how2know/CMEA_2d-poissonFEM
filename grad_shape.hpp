#pragma once
#include <Eigen/Core>


//! The gradient of the shape function (on the reference element)
//! 
//! We have three shape functions
//!
//! @param i : integer between 0 and 2 (inclusive). Decides which shape function to return.
//! @param x : x coordinate in the reference element.
//! @param y : y coordinate in the reference element.

inline Eigen::Vector2d gradientLambda(const int i, double x, double y)
{
// (write your solution here)

    /// Start of my solution ///

    // lambda = alpha + beta * x + gamma * y    =>    grad(lambda) = (beta, gamma)

    // lambda_0   =>   alpha = 1, beta = -1, gamma = -1
    if (i == 0)
    {
        return Eigen::Vector2d(-1,-1);
    }

    // lambda_1   =>   alpha = 0, beta = 1, gamma = 0
    if (i == 1)
    {
        return Eigen::Vector2d(1,0);
    }

    // lambda_2   =>   alpha = 0, beta = 0, gamma = 1
    if (i == 2)
    {
        return Eigen::Vector2d(0,1);
    }

    /// End of my solution ///
}
