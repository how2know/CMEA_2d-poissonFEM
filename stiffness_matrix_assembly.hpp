#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>

#include "stiffness_matrix.hpp"

//! Sparse Matrix type. Makes using this type easier.
typedef Eigen::SparseMatrix<double> SparseMatrix;

//! Used for filling the sparse matrix.
typedef Eigen::Triplet<double> Triplet;

//----------------AssembleMatrixBegin----------------

//! Assemble the stiffness matrix
//! for the linear system
//!
//! @param[out] A will at the end contain the Galerkin matrix
//! @param[in] vertices a list of triangle vertices
//! @param[in] triangles a list of triangles

template<class Matrix>

void assembleStiffnessMatrix(Matrix& A, const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles)
{
    // number of triangles
    const int numberOfElements = triangles.rows();

    A.resize(vertices.rows(), vertices.rows());
    
    std::vector<Triplet> triplets;

    triplets.reserve(numberOfElements * 3 * 3);

// (write your solution here)

    /// Start of my solution ///

    // loop over all triangles
    for (int i = 0; i < numberOfElements; i++)
    {
        // store the coordinates of the three points of the triangle
        auto point_a = vertices.row(triangles(i, 0));
        auto point_b = vertices.row(triangles(i, 1));
        auto point_c = vertices.row(triangles(i, 2));

        // 3x3 stiffness matrix of one triangle
        Eigen::Matrix3d stiffnessMatrix;
        computeStiffnessMatrix(stiffnessMatrix, point_a, point_b, point_c);

        // store the elements in the big stiffness matrix (assembly)
        for (int alpha = 0; alpha < 3; alpha++)
        {
            for (int beta = 0; beta < 3; beta++)
            {
                triplets.push_back(Triplet(triangles(i, alpha), triangles(i, beta), stiffnessMatrix(alpha, beta)));
            }
        }
    }



    /// End of my solution ///

    A.setFromTriplets(triplets.begin(), triplets.end());
}
//----------------AssembleMatrixEnd----------------
