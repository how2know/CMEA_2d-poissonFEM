#pragma once
#include <Eigen/Core>
#include "load_vector.hpp"

//----------------AssembleVectorBegin----------------

//! Assemble the load vector into the full right hand side
//! for the linear system
//!
//! @param[out] F will at the end contain the RHS values for each vertex.
//! @param[in] vertices a list of triangle vertices
//! @param[in] triangles a list of triangles
//! @param[in] f the RHS function f.

void assembleLoadVector(Eigen::VectorXd& F,
                        const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles,
                        const std::function<double(double, double)>& f)
{
    // number of triangles
     const int numberOfElements = triangles.rows();

     F.resize(vertices.rows());
     F.setZero();

// (write your solution here)

     /// Start of my solution ///

     // loop over all triangles
     for (int i = 0; i < numberOfElements; i++)
     {
         // store the coordinates of the three points of the triangle
         auto point_a = vertices.row(triangles(i, 0));
         auto point_b = vertices.row(triangles(i, 1));
         auto point_c = vertices.row(triangles(i, 2));

         // load vector of one triangle
         Eigen::Vector3d loadVector;
         computeLoadVector(loadVector, point_a, point_b, point_c, f);

         // store the elements in the big load vector
         for (int alpha = 0; alpha < 3; alpha++)
         {
             F(triangles(i, alpha)) += loadVector(alpha);
         }
     }

     /// End of my solution ///
}

//----------------AssembleVectorEnd----------------
