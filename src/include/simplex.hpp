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

#include "solver.hpp"

namespace lp {
namespace solver {
namespace simplex {

class SimplexSolver : public Solver {
   public:
    Vector _x;
    Matrix _tableau;

    SimplexSolver(Matrix& A, Vector& b, Vector& c);
    ~SimplexSolver() override;

    /*
     * Solves a linear program specified by the tableau, using the simplex
     * method.
     */
    Vector& Solve() override;

   private:
    /*
     * Builds the simplex tableau, which is where we do all operations to solve
     * the linear program.
     */
    void BuildTableau(const Matrix& A, const Vector& b, const Vector& c);

    /*
     * Tests whether a given column is basic.
     */
    int IsBasicCol(int col);

    /*
     * Returns true when we are at an optimum.
     */
    bool Done();

    /*
     * Perform Gaussian elimination about an element (row, col) given in the
     * pair.
     */
    void PivotAbout(const std::pair<int, int>& pivot);

    /*
     * Finds a pivot element. Currently uses Bland's Rule. Must allow new types
     * of pivotting rules.
     */
    std::pair<Vector::IndexType, Vector::IndexType> FindPivot();
};

}  // namespace simplex
}  // namespace solver
}  // namespace lp
