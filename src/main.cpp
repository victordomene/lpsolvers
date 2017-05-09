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

#include <iostream>

#include "flens/flens.cxx"
#include "lp.hpp"
#include "simplex.hpp"

int main(int argc, char* argv[]) {
    if (argc != 2 && argc != 3) {
        std::cout
            << "Usage: ./lpsolver [opt: <bland/dantzig/random>] <mps filename>"
            << std::endl;
        std::cout << "IPMSolver is not ready for deployment." << std::endl;
        exit(1);
    }

    lp::solver::simplex::PivotingRules rule =
        lp::solver::simplex::PivotingRules::BLAND;
    std::string filename = argv[1];

    if (argc == 3) {
        filename = argv[2];
        std::string rule_str = argv[1];

        if (rule_str == "bland") {
            rule = lp::solver::simplex::PivotingRules::BLAND;
        } else if (rule_str == "dantzig") {
            rule = lp::solver::simplex::PivotingRules::DANTZIG;
        } else if (rule_str == "random") {
            rule = lp::solver::simplex::PivotingRules::RANDOM;
        } else {
            std::cout << "Invalid pivoting rule option." << std::endl;
            exit(1);
        }
    }

    /*
     * Creates the linear program with the given MPS file.
     */
    lp::LinearProgram lp(filename);

    try {
        lp::Vector x = lp.SimplexSolve(rule);
        std::cout << "Optimum found at vector:" << x << std::endl;
        std::cout << "Maximum value: " << lp._c * x << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Unbounded Linear Program." << std::endl;
    }

    return 0;

    // lp::Matrix m(3,2);
    // m = 1, 2, 1, 1, 3, 2;
    // lp::Vector b(3);
    // b = 16, 9, 24;
    // lp::Vector c(2);
    // c = 40, 30;
    // lp::solver::simplex::SimplexSolver solver(m, b, c,
    // lp::solver::simplex::PivottingRules::FIRST);
    // lp::Vector& sol = solver.Solve();
    // std::cout << sol << "endl";
    // return 0;
}
