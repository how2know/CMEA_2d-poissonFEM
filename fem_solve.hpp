#pragma once
#include <Eigen/Core>
#include <string>
#include <igl/readMESH.h>
#include <igl/readSTL.h>
#include <igl/slice.h>
#include <igl/slice_into.h>
#include "stiffness_matrix_assembly.hpp"
#include "load_vector_assembly.hpp"
#include "dirichlet_boundary.hpp"

typedef Eigen::VectorXd Vector;

//----------------solveBegin----------------

//! Solve the FEM system.
//!
//! @param[out] u will at the end contain the FEM solution.
//! @param[in] vertices list of triangle vertices for the mesh
//! @param[in] triangles list of triangles (described by indices)
//! @param[in] f the RHS f (as in the exercise)
//! return number of degrees of freedom (without the boundary dofs)

int solveFiniteElement(Vector& u,
                       const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles,
                       const std::function<double(double, double)>& f)
{
    // stiffness matrix
    SparseMatrix A;

// (write your solution here)

    /// Start of my solution ///

    // compute the stiffness matrix A
    assembleStiffnessMatrix(A, vertices, triangles);

    /// End of my solution ///

    // load vector
    Vector F;

// (write your solution here)

    /// Start of my solution ///

    // compute the load vector F
    assembleLoadVector(F, vertices, triangles, f);

    /// End of my solution ///

    // vector U cointaining the solution of the Poisson equation
    u.resize(vertices.rows());
    u.setZero();

    // vector cointaining the indices of all internal vertices (that are not at the boundary)
    Eigen::VectorXi interiorVertexIndices;

    // function g at the boundary (u = g(x) = 0)
    auto zerobc = [](double x, double y){ return 0;};

    // set homogeneous Dirichlet Boundary conditions
// (write your solution here)

    /// Start of my solution ///

    // store all u_i = 0 for all u_i at the boundary
    // store all indices of internal vertices
    setDirichletBoundary(u, interiorVertexIndices, vertices, triangles, zerobc);

    // put the solution on the boundary in the load vector (F_i = F_i - A * u   with i on the boundary)
    F -= A * u;

    /// End of my solution ///

    // stiffness matrix containing only the element for the internal vertices
    SparseMatrix AInterior;
    igl::slice(A, interiorVertexIndices, interiorVertexIndices, AInterior);

    // matrix to solve the linear algebra
    Eigen::SimplicialLDLT<SparseMatrix> solver;

    // load vector containing only the element for the internal vertices
    Vector FInterior;
    igl::slice(F, interiorVertexIndices, FInterior);

    //initialize solver for AInterior
// (write your solution here)

    /// Start of my solution ///

    solver.compute(AInterior);

    /// End of my solution ///

    //solve interior system
// (write your solution here)

    /// Start of my solution ///

    // solve AU=F
    Vector u_inner = solver.solve(FInterior);

    // store the solution for the internal U in the general U
    igl::slice_into(u_inner, interiorVertexIndices, u);

    /// End of my solution ///

    return interiorVertexIndices.size();

}

//----------------solveEnd----------------
