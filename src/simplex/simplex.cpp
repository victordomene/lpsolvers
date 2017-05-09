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

#include <cassert>
#include <limits>
#include <random>

#include "simplex.hpp"
#include "utils.hpp"

namespace lp {
namespace solver {
namespace simplex {

SimplexSolver::SimplexSolver(Matrix& A, Vector& b, Vector& c,
                             PivotingRules rule)
    : _x(c.length()),
      _tableau(A.numRows() + 1, A.numCols() + A.numRows() + 2),
      _rule(rule) {
    BuildTableau(A, b, c);
}

SimplexSolver::~SimplexSolver() {}

std::pair<Vector::IndexType, Vector::IndexType> SimplexSolver::FindPivot() {
    int pivot_col = -1;
    int pivot_row = -1;

    double min = std::numeric_limits<double>::max();

    switch (_rule) {
        case PivotingRules::DANTZIG: {
            for (int i = _tableau.firstCol() + 1; i <= _tableau.lastCol() - 1;
                 ++i) {
                double current = _tableau(_tableau.firstRow(), i);
                if (current <= min) {
                    min = current;
                    pivot_col = i;
                }
            }
            break;
        }
        case PivotingRules::RANDOM: {
            std::random_device rng;
            std::uniform_int_distribution<std::random_device::result_type> dist(
                _tableau.firstRow() + 1, _tableau.lastRow());

            int i = dist(rng);
            while (_tableau(_tableau.firstRow(), i) >= 0.0) {
                i = dist(rng);
            }
            pivot_col = i;
            break;
        }
        case PivotingRules::BLAND: {
            for (int i = _tableau.firstCol() + 1; i <= _tableau.lastCol() - 1;
                 ++i) {
                double current = _tableau(_tableau.firstRow(), i);
                if (current < 0.0) {
                    min = current;
                    pivot_col = i;
                    break;
                }
            }
            break;
        }
    }
    assert(_tableau(_tableau.firstRow(), pivot_col) < 0);
    assert(pivot_col >= 0);

    min = std::numeric_limits<double>::max();
    int fallback = -1;
    for (int i = _tableau.firstRow() + 1; i <= _tableau.lastRow(); ++i) {
        double denom = _tableau(i, pivot_col);
        double num = _tableau(i, _tableau.lastCol());
#ifdef DEBUG
        std::cerr << "[DEBUG] (row, num, denom): (" << i << ", " << num << ", "
                  << denom << ")" << std::endl;
#endif

        /*
         * Only compute ratios if the denom is actually positive.
         */
        if (fallback == -1 && num == 0.0 && denom != 0.0) {
            fallback = i;
        }
        if (denom <= 0.0) {
            continue;
        }

        double ratio = num / denom;
        if (ratio > 0.0 && ratio < min) {
            min = ratio;
            pivot_row = i;
        }
    }
    if (pivot_row < 0) {
        pivot_row = fallback;
    }

    if (pivot_row < 0) {
#ifdef DEBUG
        std::cerr << "[DEBUG] Could not find a 'good' pivoting row. "
                     "Program is unbounded."
                  << std::endl;
        std::cerr << _tableau << std::endl;
#endif
        throw UnboundedLinearProgram();
    }

#ifdef DEBUG
    std::cerr << _tableau << std::endl;
    std::cerr << "[DEBUG] Pivot (row, col): (" << pivot_row << ", " << pivot_col
              << ")" << std::endl;
#endif
    return std::pair<int, int>(pivot_row, pivot_col);
}

void SimplexSolver::PivotAbout(const std::pair<int, int>& pivot) {
    Underscore _;
    Matrix::View pivot_row = _tableau(_(pivot.first, pivot.first), _);
    pivot_row = pivot_row * 1 / _tableau(pivot.first, pivot.second);

    for (int i = _tableau.firstRow(); i <= _tableau.lastRow(); ++i) {
        if (i == pivot.first) {
            continue;
        }

        Matrix::View current = _tableau(_(i, i), _);
        current = current - pivot_row * _tableau(i, pivot.second);
    }
}

bool SimplexSolver::Done() {
    for (int i = _tableau.firstCol() + 1; i <= _tableau.lastCol() - 1; ++i) {
        if (_tableau(_tableau.firstRow(), i) < 0) {
            return false;
        }
    }
#ifdef DEBUG
    std::cerr << "[DEBUG] Final tableau:" << std::endl;
    std::cerr << _tableau << std::endl;
#endif
    return true;
}

/*
 * FIXME: This function is particuarly, uh, ugly.
 */
int SimplexSolver::IsBasicCol(int col) {
    int num_ones = 0;
    int one_pos = -1;
    for (int i = _tableau.firstRow(); i <= _tableau.lastRow(); ++i) {
        double current = _tableau(i, col);

        if (utils::within(current, 1.0)) {
            num_ones++;
            one_pos = i;
        } else if (!utils::within(current, 0.0)) {
            return -1;
        }
    }

    if (num_ones == 1) {
        return one_pos;
    }
    return -1;
}

Vector& SimplexSolver::Solve() {
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
        }
        ++index;
    }
    return _x;
}

void SimplexSolver::BuildTableau(const Matrix& A, const Vector& b,
                                 const Vector& c) {
    _x = 0;
    _tableau = 0;

    /* First column corresponds to the objective value, Z */
    _tableau(_tableau.firstRow(), _tableau.firstCol()) = 1;

    /* Add in the entries in c to the tableau */
    for (Vector::IndexType i = c.firstIndex(); i <= c.lastIndex(); ++i) {
        _tableau(_tableau.firstRow(), i + 1) = -1 * c(i);
    }

    /* Add in the entries in b to the tableau */
    for (Vector::IndexType i = b.firstIndex(); i <= b.lastIndex(); ++i) {
        _tableau(i + 1, _tableau.lastCol()) = b(i);
    }

    /* Add in the entries in A to the tableau; they start a column after */
    for (Vector::IndexType i = A.firstRow(); i <= A.lastRow(); ++i) {
        bool must_negate = b(i) < 0.0;
        if (must_negate) {
            _tableau(i + 1, _tableau.lastCol()) = -1.0 * b(i);
        }

        for (Vector::IndexType j = A.firstCol(); j <= A.lastCol(); ++j) {
            if (must_negate) {
                _tableau(i + 1, j + 1) = -1 * A(i, j);
            } else {
                _tableau(i + 1, j + 1) = A(i, j);
            }
        }

        /*
         * Add in slack variables. The initial solution will be basically
         * x_i = 0, and the slack variables equal to the right hand side of
         * that particular constraint. So, since all equations were converted
         * in the MPS to Ax >= b, the slacks have -1 coefficients.
         */
        if (must_negate) {
            _tableau(i + 1, A.lastCol() + i + 1) = 1;
        } else {
            _tableau(i + 1, A.lastCol() + i + 1) = -1;
        }
    }

#ifdef DEBUG
    std::cerr << "[DEBUG] Initialized tableau." << std::endl;
#endif
}

}  // namespace simplex
}  // namespace solver
}  // namespace lp
