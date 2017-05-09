#ifndef _INCLUDE_LP_HPP_
#define _INCLUDE_LP_HPP_

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

#include "flens/flens.cxx"

namespace lp {

/*
 * FIXME: This should be types.hpp.
 */
typedef flens::GeMatrix<flens::FullStorage<double, flens::ColMajor> > Matrix;
typedef flens::DenseVector<flens::Array<double> > Vector;
typedef flens::DenseVector<flens::Array<int> > VectorInt;
typedef flens::Underscore<Matrix::IndexType> Underscore;

class UnboundedLinearProgram : public std::runtime_error {
   public:
    UnboundedLinearProgram() : std::runtime_error("Unbounded Linear Program") {}
};

namespace solver {
namespace simplex {
enum class PivotingRules;
}
}

/*
 * Encodes a Linear Program in the form:
 *
 *    max: _c^Tx
 *    subject to: _Ax >= _b
 *
 * The constructor that accepts a file assumes it is an MPS format file, and
 * automatically converts the LP into standard form.
 */
class LinearProgram {
   public:
    Matrix _A;
    Vector _b;
    Vector _c;

    LinearProgram(int rowsA, int colsA, double** A, double* b, double* c);
    LinearProgram(std::string mpsfile);
    ~LinearProgram() {};

    /*
     * Creates a SimplexSolver and runs it.
     */
    Vector SimplexSolve(solver::simplex::PivotingRules rule);

    /*
     * Creates an IPMSolver and runs it.
     */
    Vector IPMSolve();

    /*
     * Creates an EllipsoidSolver and runs it.
     */
    Vector EllipsoidSolve();

   private:
};

}  // namespace simplex

#endif  // _INCLUDE_LP_HPP_
