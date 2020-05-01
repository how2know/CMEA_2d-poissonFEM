#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "coordinate_transform.hpp"
#include "integrate.hpp"
#include "shape.hpp"
#include <functional>

//----------------compVectorBegin----------------

//! Evaluate the load vector on the triangle spanned by
//! the points (a, b, c).
//!
//! Here, the load vector is a vector $(v_i)$ of
//! three components, where 
//! 
//! $$v_i = \int_{K} \lambda_i^K(x, y) f(x, y) \; dV$$
//! 
//! where $K$ is the triangle spanned by (a, b, c).
//!
//! @param[out] loadVector should be a vector of length 3. 
//!                        At the end, will contain the integrals above.
//!
//! @param[in] a the first corner of the triangle
//! @param[in] b the second corner of the triangle
//! @param[in] c the third corner of the triangle
//! @param[in] f the function f (LHS).

template<class Vector, class Point>

void computeLoadVector(Vector& loadVector,
                       const Point& a, const Point& b, const Point& c,
                       const std::function<double(double, double)>& f)
{
    // Jacobian J_K of the mapping function
    Eigen::Matrix2d coordinateTransform = makeCoordinateTransform(b - a, c - a);

    // absolute value of the determinant of J_K
    double volumeFactor = std::abs(coordinateTransform.determinant());

// (write your solution here)

    /// Start of my solution ///

    for (int i = 0; i < 3; i++)
    {
        // function to compute the integral
        auto integrand = [&](double x, double y)
        {
            // coordinate in the reference element
            Eigen::Vector2d ref_coordinate;
            ref_coordinate << x,y;

            // point a of the triangle
            Eigen::Vector2d point_a;
            point_a << a(0), a(1);

            // mapping of the reference element to the triangle
            Eigen::Vector2d map_coordinate = coordinateTransform * ref_coordinate + point_a;

            return f(map_coordinate[0], map_coordinate[1]) * lambda(i, x, y) * volumeFactor;
        };

        // element F_i of the load vector
        loadVector(i) = integrate(integrand);
    }

    /// End of my solution ///

}

//----------------compVectorEnd----------------
