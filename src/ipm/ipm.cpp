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
#include <limits>

namespace lp {
namespace solver {
namespace ipm {

IPMSolver::IPMSolver(Matrix& A, Vector& b, Vector& c)
    : _x(c.length()), _A(A), _S(c.length(), c.length()) {}

IPMSolver::~IPMSolver() {}

Vector& IPMSolver::Solve() {
    while (!Done()) {
        /* Notice that this may throw. The caller to Solve() must be aware. */
        std::pair<int, int> pivot = FindPivot();
        PivotAbout(pivot);
    }

    int index = _x.firstIndex();
    for (int i = _tableau.firstCol() + 1;
         i <= _tableau.firstCol() + _x.lastIndex(); ++i) {
        int basic_row = IsBasicCol(i);
        if (basic_row != -1) {
            _x(index) = _tableau(basic_row, _tableau.lastCol());
            ++index;
        }
    }
    std::cout << _tableau(_tableau.firstRow(), _tableau.lastCol()) << std::endl;
    return _x;
}

}  // namespace ipm
}  // namespace solver
}  // namespace lp