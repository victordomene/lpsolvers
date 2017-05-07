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
#include "mpsparser.hpp"
#include "simplex.hpp"

namespace lp {

LinearProgram::LinearProgram(int rowsA, int colsA, double** A, double* b,
                             double* c)
    : _A(rowsA, colsA), _b(rowsA), _c(rowsA) {}

LinearProgram::LinearProgram(std::string mpsfile) {
    mps::MPSParser parser(mpsfile);
    _A = parser.GetA();
    _b = parser.GetB();
    _c = parser.GetC();

    //std::cout << _A << std::endl << std::endl;
    //std::cout << _b << std::endl << std::endl;
    //std::cout << _c << std::endl << std::endl;
}

Vector LinearProgram::SimplexSolve() {
    solver::simplex::SimplexSolver solver(_A, _b, _c, solver::simplex::PivottingRules::BLAND);
    return solver.Solve();
}

Vector LinearProgram::IPMSolve() {
    Vector a(1);
    return a;
}

Vector LinearProgram::EllipsoidSolve() {
    Vector a(1);
    return a;
}

}  // namespace lp
