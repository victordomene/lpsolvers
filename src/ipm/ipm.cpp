/*
 * Copyright (c) 2017, Victor Domene
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ipm.hpp"
#include <cassert>
#include <cmath>
#include <limits>

#include "utils.hpp"

namespace lp {
namespace solver {
namespace ipm {

IPMSolver::IPMSolver(Matrix& A, Vector& b, Vector& c)
    : _lambda(utils::_precision),
      _x(c.length(), 1),
      _b(b.length(), 1),
      _c(c.length(), 1),
      a(c.length()),
      _A(A) {
    for (int i = c.firstIndex(); i <= c.lastIndex(); ++i) {
        _c(i, 1) = c(i);
        _b(i, 1) = b(i);
    }
    _x = 0.0;
}

IPMSolver::~IPMSolver() {}

Vector& IPMSolver::Solve() {
    FindInitialSolution();

    while (!Done()) {
        NewtonOptimization();
        UpdateLambda();
    }

    return a;
}

void IPMSolver::UpdateLambda() {
    /*
     * Update lambda. This is the theoretical minimum; we can probably
     * simply do more here, and check later if this is a good number.
     */
    _lambda = _lambda * (1.0 + 1.0 / 4.0 * std::sqrt(_A.numCols()));
}

bool IPMSolver::Done() { return _lambda > _A.numCols() / utils::_precision; }

void IPMSolver::FindInitialSolution() { return; }

void IPMSolver::NewtonOptimization() {
    /* Get slack matrix. */
    Matrix S(_b.numRows(), _b.numRows());
    S = 0;

    Matrix slacks(_b.numRows(), 1);
    slacks = _A * _x - _b;

    for (int i = slacks.firstRow(); i <= slacks.lastRow(); ++i) {
        S(i, i) = slacks(i, slacks.firstCol());
    }

    /* Not sure what this needs to be... */
    VectorInt piv(_x.numRows());

    /* Invert slack matrix */
    int info = flens::lapack::trf(S, piv);
    if (info != 0) {
        std::cout << "Singular slack matrix!!!" << std::endl;
        exit(1);
    }
    flens::lapack::tri(S, piv);

    /* Gradient matrix. */
    Matrix e(_c.numRows(), 1);
    e = 1.0;
    Matrix G = _lambda * _c - flens::transpose(_A) * S * e;

    /* Inverse Hessian matrix computation */
    Matrix H = flens::transpose(_A) * S * S * _A;

    info = flens::lapack::trf(H, piv);
    if (info != 0) {
        std::cout << "Hessian matrix singular!!!" << std::endl;
        exit(1);
    }
    flens::lapack::tri(H, piv);

    /* Newton update step! */
    _x = _x - H * G;
}

}  // namespace ipm
}  // namespace solver
}  // namespace lp
