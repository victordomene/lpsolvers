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
#include <fstream>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include "mpsparser.hpp"

namespace lp {
namespace mps {

std::unique_ptr<std::vector<std::string> > MPSParser::TokenizeLine(
    std::string& line) {
    std::vector<std::string> v;

    size_t start = 0;
    size_t end = 0;

    while (1) {
        end = line.find(' ', start);

        /* no separators until the end of the line */
        if (end == std::string::npos) {
            break;
        }

        if (end != start) {
            v.push_back(line.substr(start, end - start));
        }
        start = end + 1;
    }

    if (end != start) {
        v.push_back(line.substr(start));
    }

    return std::make_unique<std::vector<std::string> >(v);
}

void MPSParser::ParseLP() {
    std::string line;
    ParsePhase phase;

    while (getline(_f, line)) {
        if (line.length() == 0) {
            continue;
        }
        if (line[0] == '*') {
            continue;
        }

        /* parse the contents of this line */
        auto tokens_ptr = TokenizeLine(line);
        auto& tokens = *tokens_ptr;

        if (tokens.size() == 0) {
            continue;
        }

        std::string& ident = tokens[0];
        if (ident == "NAME") {
            continue;
        } else if (ident == "ROWS") {
            phase = ParsePhase::ROWS;
            continue;
        } else if (ident == "COLUMNS") {
            phase = ParsePhase::COLUMNS;
            continue;
        } else if (ident == "RHS") {
            phase = ParsePhase::RHS;
            continue;
        } else if (ident == "BOUNDS") {
            phase = ParsePhase::BOUNDS;
            continue;
        } else if (ident == "ENDATA") {
            break;
        }

        switch (phase) {
            case ParsePhase::COLUMNS: {
                assert(tokens.size() == 3 || tokens.size() == 5);
                const std::string& constr_name = tokens[1];
                const std::string& var_name = tokens[0];
                const double value = stod(tokens[2]);

                ConstraintType type = _constr_meta[constr_name].first;
                int row = _constr_meta[constr_name].second;
                int col = _var_meta[var_name];

                if (type == ConstraintType::LE) {
                    _A[row][col] = -1 * value;
                } else if (type == ConstraintType::GE ||
                           type == ConstraintType::NONE) {
                    _A[row][col] = value;
                } else {
                    _A[row][col] = value;
                    row = _constr_meta[constr_name + "LE"].second;
                    _A[row][col] = -1 * value;
                }

                if (tokens.size() == 5) {
                    const std::string& constr_name = tokens[3];
                    const double value = stod(tokens[4]);

                    type = _constr_meta[constr_name].first;
                    row = _constr_meta[constr_name].second;

                    if (type == ConstraintType::LE) {
                        _A[row][col] = -1 * value;
                    } else if (type == ConstraintType::GE ||
                               type == ConstraintType::NONE) {
                        _A[row][col] = value;
                    } else {
                        _rhs[row] = 1.0 * value;
                        row = _constr_meta[constr_name + "LE"].second;
                        _rhs[row] = -1.0 * value;
                    }
                }
                break;
            }
            case ParsePhase::RHS: {
                assert(tokens.size() == 3 || tokens.size() == 5);
                const std::string& constr_name = tokens[1];
                const double value = stod(tokens[2]);

                ConstraintType type = _constr_meta[constr_name].first;
                int row = _constr_meta[constr_name].second;

                if (type == ConstraintType::LE) {
                    _rhs[row] = -1 * value;
                } else if (type == ConstraintType::GE ||
                           type == ConstraintType::NONE) {
                    _rhs[row] = value;
                } else {
                    _rhs[row] = value;
                    row = _constr_meta[constr_name + "LE"].second;
                    _rhs[row] = -1.0 * value;
                }

                if (tokens.size() == 5) {
                    const std::string& constr_name = tokens[3];
                    const double value = stod(tokens[4]);
                    row = _constr_meta[constr_name].second;

                    if (type == ConstraintType::LE) {
                        _rhs[row] = -1 * value;
                    } else if (type == ConstraintType::GE ||
                               type == ConstraintType::NONE) {
                        _rhs[row] = value;
                    } else {
                        _rhs[row] = value;
                        row = _constr_meta[constr_name + "LE"].second;
                        _rhs[row] = -1.0 * value;
                    }
                }
                break;
            }
            case ParsePhase::BOUNDS: {
                assert(tokens.size() == 4);
                const std::string constr_name =
                    "BOUNDCONSTRAINT" + tokens[0] + tokens[2];
                const std::string& var_name = tokens[2];
                const double value = stod(tokens[3]);

                ConstraintType type = _constr_meta[constr_name].first;
                int row = _constr_meta[constr_name].second;
                int col = _var_meta[var_name];

                if (type == ConstraintType::LE) {
                    _A[row][col] = -1.0;
                    _rhs[row] = -1 * value;
                } else if (type == ConstraintType::GE) {
                    _A[row][col] = 1.0;
                    _rhs[row] = value;
                } else {
                    _A[row][col] = 1.0;
                    _rhs[row] = value;
                    row = _constr_meta[constr_name + "LE"].second;
                    _A[row][col] = -1.0;
                    _rhs[row] = -1.0 * value;
                }
                break;
            }
            default:
                continue;
        }
    }

    /* resets file to beginning */
    _f.clear();
    _f.seekg(0, std::ios::beg);
}

void MPSParser::ParseMeta() {
    std::string line;
    ParsePhase phase;

    while (getline(_f, line)) {
        if (line.length() == 0) {
            continue;
        }
        if (line[0] == '*') {
            continue;
        }

        /* parse the contents of this line */
        auto tokens_ptr = TokenizeLine(line);
        auto& tokens = *tokens_ptr;

        if (tokens.size() == 0) {
            continue;
        }

        std::string ident = tokens[0];
        if (ident == "NAME") {
            assert(tokens.size() == 2);
            _name = tokens[1];
            continue;
        } else if (ident == "ROWS") {
            phase = ParsePhase::ROWS;
            assert(tokens.size() == 1);
            continue;
        } else if (ident == "COLUMNS") {
            phase = ParsePhase::COLUMNS;
            assert(tokens.size() == 1);
            continue;
        } else if (ident == "RHS") {
            phase = ParsePhase::RHS;
            assert(tokens.size() == 1);
            continue;
        } else if (ident == "BOUNDS") {
            phase = ParsePhase::BOUNDS;
            assert(tokens.size() == 1);
            continue;
        } else if (ident == "ENDATA") {
            break;
        }

        switch (phase) {
            case ParsePhase::ROWS: {
                assert(tokens.size() == 2);
                ConstraintType type;
                if (tokens[0] == "E") {
                    type = ConstraintType::EQ;

                    int id = _constr_meta.size();
                    _constr_meta[tokens[1] + "LE"] =
                        std::pair<ConstraintType, int>(type, id);
                } else if (tokens[0] == "L") {
                    type = ConstraintType::LE;
                } else if (tokens[0] == "G") {
                    type = ConstraintType::GE;
                } else if (tokens[0] == "N") {
                    type = ConstraintType::NONE;
                } else {
                    std::cout << "Unrecognized type of ROW. Can't handle..."
                              << std::endl;
                    exit(1);
                }

                int id = _constr_meta.size();
                _constr_meta[tokens[1]] =
                    std::pair<ConstraintType, int>(type, id);
                break;
            }
            case ParsePhase::BOUNDS: {
                assert(tokens.size() == 4);
                ConstraintType type;
                if (tokens[0] == "UP") {
                    type = ConstraintType::LE;
                } else if (tokens[0] == "LO") {
                    type = ConstraintType::GE;
                } else if (tokens[0] == "FX") {
                    type = ConstraintType::EQ;

                    int id = _constr_meta.size();
                    _constr_meta["BOUNDCONSTRAINT" + tokens[0] + tokens[2] + "LE"] =
                        std::pair<ConstraintType, int>(type, id);
                } else if (tokens[0] == "FR") {
                    std::cout << "Unsupported FR bounds!" << std::endl;
                    exit(1);
                }

                int id = _constr_meta.size();
                _constr_meta["BOUNDCONSTRAINT" + tokens[0] + tokens[2]] =
                    std::pair<ConstraintType, int>(type, id);
                break;
            }
            case ParsePhase::COLUMNS: {
                assert(tokens.size() == 3 || tokens.size() == 5);
                auto it = _var_meta.find(tokens[0]);

                if (it == _var_meta.end()) {
                    /* New variable... */
                    int id = _var_meta.size();
                    _var_meta[tokens[0]] = id;
                }
            }
            default:
                continue;
        }
    }

    /* resets file to beginning */
    _f.clear();
    _f.seekg(0, std::ios::beg);
}

MPSParser::MPSParser(std::string& filename) : _filename(filename) {
    _f.open(_filename);
    ParseMeta();

    std::vector<double> row(_var_meta.size(), 0);
    _A = std::vector<std::vector<double> >(_constr_meta.size(), row);
    _rhs = std::vector<double>(_constr_meta.size(), 0);

    ParseLP();

#ifdef DEBUG
    for (const auto& row : _A) {
        for (const auto& x : row) {
            std::cout << x << ", ";
        }
        std::cout << std::endl;
    }
#endif
}

}  // namespace mps
}  // namespace lp
