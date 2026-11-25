// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief Searches for the first contact or line crossing on free paths

#pragma once

#include "free_path.hpp"
#include "stone_body.hpp"

namespace digitalcurling::simulators::fcv1event {

/// @brief Lines at which a step ends
struct OutLines {
    float x_limit;  ///< |x| of the sides, or infinity
    float y_back_line;
    float y_back_board;

    /// @brief The lines that a place is past, one bit each
    unsigned Get(double x, double y) const;
};

/// @brief Find the first frame after which two stones touch and close in on each other
/// @param a path of one stone
/// @param b path of the other
/// @param limit last frame to look at
/// @return the frame, or a number above limit if there is none
int FindContact(FreePath const& a, FreePath const& b, int limit);

/// @brief Find the first frame after which a stone is past a line that it isn't past at first
/// @param path path of the stone
/// @param out_lines lines that the stone is past at first
/// @param lines all lines
/// @param limit last frame to look at
/// @return the frame, or a number above limit if there is none
int FindCrossing(FreePath const& path, unsigned out_lines, OutLines const& lines, int limit);

} // namespace digitalcurling::simulators::fcv1event
