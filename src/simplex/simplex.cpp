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

#include "simplex.hpp"

namespace lp {
namespace solver {
namespace simplex {

SimplexSolver::SimplexSolver(Matrix& A, Vector& b, Vector& c)
    : _A(A.numRows(), A.numCols() * 2), _b(b.length() * 2), _c(c.length() * 2) {
    GetStandardForm(A, b, c);
}

SimplexSolver::~SimplexSolver() {}

Vector& SimplexSolver::Solve() {
    /*
     * FIXME: Replace this with something that like, actually solves it.
     */

    /*
     * Pseudo-code idea:
     *   - Find an initial vertex;
     *   - Find whatever basis corresponds to it (should be != 0 entries);
     *   - Rewrite the LP using A_B, x_B, A_N, x_N;
     *   - If the reduced cost vector is >= 0, we are at OPT;
     *   - Otherwise, for whatever component j  of the reduced cost vector is
     *     < 0, increment x_j while maintaining feasibility;
     *   - Recall that by changing x_j, we change x_N, which changes x_B
     *     implicitly through the relation on the constraint;
     *   - Pivotting rule: kickout everyone who became 0, and add x_j in.
     *   - Need to handle all the crappy conditions
     */
    Vector* a = new Vector(1);
    return *a;
}

void SimplexSolver::GetStandardForm(const Matrix& A, const Vector& b,
                                    const Vector& c) {
    _A = 1;
    _b = 0;
    _c = 0;

    for (Vector::IndexType i = A.firstRow(); i <= A.lastRow(); ++i) {
        for (Vector::IndexType j = A.firstCol(); j <= A.lastCol(); ++j) {
            _A(i, j) = A(i, j);
        }
    }

    for (Vector::IndexType i = b.firstIndex(); i <= b.lastIndex(); ++i) {
        _b(i) = b(i);
    }

    for (Vector::IndexType i = c.firstIndex(); i <= c.lastIndex(); ++i) {
        _c(i) = c(i);
    }
}

}  // namespace simplex
}  // namespace solver
}  // namespace lp
