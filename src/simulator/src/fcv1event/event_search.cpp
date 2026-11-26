// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cmath>
#include "event_search.hpp"

namespace digitalcurling::simulators::fcv1event {

namespace {

using Node = FreePath::Node;

// Distance below which stones may touch; the padding covers rounding
constexpr double kPadding = 2e-5;
constexpr double kReach = kDiameter + kPadding;

// Enough for the depth of a binary search over any number of frames
constexpr int kStackSize = 40;

// How far a path that turns by `turn` can be from the chord between its ends: it
// stays in the triangle between the chord and the tangents at its ends.
inline double GetBulge(double chord, double turn)
{
    double const t = 0.5 * turn;
    return 0.5 * chord * (t + t * t * t);  // above tan(t) for t <= 1
}
constexpr double kMaxTurn = 1.5;

// Squared distance between the origin and the segment from (x0, y0) to (x1, y1)
inline double GetSquaredDistance(double x0, double y0, double x1, double y1)
{
    double const ex = x1 - x0;
    double const ey = y1 - y0;
    double const length_squared = ex * ex + ey * ey;
    double const t = length_squared > 0. ? std::clamp(-(x0 * ex + y0 * ey) / length_squared, 0., 1.) : 0.;
    double const x = x0 + t * ex;
    double const y = y0 + t * ey;
    return x * x + y * y;
}

// Search (low, limit] for the first frame for which is_event is true, skipping the
// intervals for which is_clear proves that there is none.
template <class NodeType, class GetNode, class IsClear, class IsEvent>
int FindFirst(NodeType low, int limit, GetNode get_node, IsClear is_clear, IsEvent is_event)
{
    NodeType stack[kStackSize];
    int size = 0;
    stack[size++] = get_node(limit);
    while (size > 0) {
        NodeType const& high = stack[size - 1];
        if (!is_clear(low, high)) {
            if (high.frame - low.frame > 1) {
                int const middle = low.frame + (high.frame - low.frame) / 2;
                stack[size++] = get_node(middle);
                continue;
            }
            if (is_event(high)) return high.frame;
        }
        low = high;
        --size;
    }
    return limit + 1;
}

// Two stones, the second relative to the first
struct PairNode {
    int frame;
    Node a, b;
    double x, y;
};

inline bool IsTouchingAndClosing(PairNode const& n)
{
    return Touches(n.a.x, n.a.y, n.b.x, n.b.y)
        && n.x * (n.b.vx - n.a.vx) + n.y * (n.b.vy - n.a.vy) < 0.;
}

// A bound on how much the velocity changes between two nodes of a path
inline double GetVelocityChange(Node const& low, Node const& high)
{
    return low.speed - high.speed + low.speed * (high.turn - low.turn);
}

bool IsPairClear(PairNode const& low, PairNode const& high)
{
    double const change = GetVelocityChange(low.a, high.a) + GetVelocityChange(low.b, high.b);

    // 1. The paths stay apart.
    double const ex = high.x - low.x;
    double const ey = high.y - low.y;
    double bulge;
    if (low.b.speed <= 0.f) {
        double const turn = high.a.turn - low.a.turn;
        if (turn > kMaxTurn) return false;
        bulge = GetBulge(std::sqrt(ex * ex + ey * ey), turn);
    } else {
        // The relative position is this close to linear interpolation, as the mean
        // velocities of any two parts of the interval differ by at most `change`.
        bulge = 0.25 * change * kFrameSeconds * (high.frame - low.frame);
    }
    double const needed = kReach + bulge;
    if (GetSquaredDistance(low.x, low.y, high.x, high.y) >= needed * needed) return true;

    // 2. They move apart all the time.
    double const opening = low.x * (low.b.vx - low.a.vx) + low.y * (low.b.vy - low.a.vy);
    return opening > 0. && opening * opening >= change * change * (low.x * low.x + low.y * low.y);
}

} // unnamed namespace


unsigned OutLines::Get(double x, double y) const
{
    float const fx = static_cast<float>(x);
    float const fy = static_cast<float>(y);
    return (fx > x_limit ? 1u : 0u) | (fx < -x_limit ? 2u : 0u)
        | (fy > y_back_line ? 4u : 0u) | (fy > y_back_board ? 8u : 0u) | (fy < 0.f ? 16u : 0u);
}

int FindContact(FreePath const& a, FreePath const& b, int limit)
{
    if (a.GetLength() <= 0.) return b.GetLength() <= 0. ? limit + 1 : FindContact(b, a, limit);

    auto const get_node = [&](int frame) {
        PairNode n{ frame, a.At(frame), b.At(frame), 0., 0. };
        n.x = n.b.x - n.a.x;
        n.y = n.b.y - n.a.y;
        return n;
    };
    Node const& start = a.GetStart();
    Node const& other = b.GetStart();
    PairNode const low{ 0, start, other, other.x - start.x, other.y - start.y };
    double const reach = kReach + a.GetLength() + b.GetLength();
    if (low.x * low.x + low.y * low.y > reach * reach) return limit + 1;
    Node const& stop = a.GetStop();
    if (b.GetLength() <= 0. && stop.turn <= kMaxTurn) {
        // The whole path stays away from a resting stone.
        double const needed = kReach + GetBulge(a.GetLength(), stop.turn);
        if (GetSquaredDistance(low.x, low.y, other.x - stop.x, other.y - stop.y) >= needed * needed) return limit + 1;
    }
    if (IsTouchingAndClosing(low)) return 0;
    if (limit <= 0) return limit + 1;
    return FindFirst(low, limit, get_node, IsPairClear, IsTouchingAndClosing);
}

int FindCrossing(FreePath const& path, unsigned out_lines, OutLines const& lines, int limit)
{
    if (path.GetLength() <= 0. || limit <= 0) return limit + 1;

    // Lines that the stone is already past don't count.
    double const infinity = std::numeric_limits<double>::infinity();
    double const x_max = out_lines & 1u ? infinity : lines.x_limit;
    double const x_min = out_lines & 2u ? -infinity : -lines.x_limit;
    double const y_max = out_lines & 4u ? (out_lines & 8u ? infinity : lines.y_back_board) : lines.y_back_line;
    double const y_min = out_lines & 16u ? -infinity : 0.;

    Node const low = path.At(0);
    auto const is_clear = [=](Node const& from, Node const& to) {
        double const turn = to.turn - from.turn;
        if (turn > kMaxTurn) return false;
        double const ex = to.x - from.x;
        double const ey = to.y - from.y;
        double const margin = GetBulge(std::sqrt(ex * ex + ey * ey), turn) + kPadding;
        // Not near the line, or heading away from it all the time
        double const side = from.speed * turn;
        return (std::max(from.x, to.x) + margin <= x_max || from.vx + side <= 0.)
            && (std::min(from.x, to.x) - margin >= x_min || -from.vx + side <= 0.)
            && (std::max(from.y, to.y) + margin <= y_max || from.vy + side <= 0.)
            && (std::min(from.y, to.y) - margin >= y_min || -from.vy + side <= 0.);
    };
    auto const is_event = [&lines, out_lines](Node const& n) { return (lines.Get(n.x, n.y) & ~out_lines) != 0; };
    return FindFirst(low, limit, [&path](int frame) { return path.At(frame); }, is_clear, is_event);
}

} // namespace digitalcurling::simulators::fcv1event
