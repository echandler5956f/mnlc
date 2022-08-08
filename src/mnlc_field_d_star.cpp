#include "mnlc_field_d_star.h"

// // ci is traversal cost index of cell with corners s, s1, s2
// // bi is traversal cost index of cell with corners s, s1 but not s2
// // f is the cost of traversing the right edge
// // Nc is the number of distinct traversal costs
// // including the infinite cost of traversing an obstacle cell
// // and Mc is the maximum traversal cost of any traversable (i.e. non-obstacle) cell
// void ConstructInterpolationTable()
// {
//     uint8_t ci = 0;
//     while (ci < NC)
//     {
//         uint8_t c = cellcosts[ci];
//         uint8_t bi = 0;
//         while (bi < NC)
//         {
//             uint8_t b = cellcosts[bi];
//             uint8_t f = 1;
//             while (f <= MC)
//             {
//                 std::array<int, 3UL> key = {ci, bi, f};
//                 if (f < b)
//                 {
//                     if (c <= f)
//                     {
//                         _Float32 value = c * std::sqrt(2.0);
//                         interpolTable.insert(std::make_pair(key, value));
//                     }
//                     else
//                     {
//                         _Float32 y = std::min(f / std::sqrt((c ^ 2) - (f ^ 2)), 1.0);
//                         _Float32 value = c * std::sqrt(1.0 + std::pow(y, 2.0)) + f * (1.0 - y);
//                         interpolTable.emplace(key, value);
//                     }
//                 }
//                 else
//                 {
//                     if (c <= b)
//                     {
//                         _Float32 value = c * std::sqrt(2.0);
//                         interpolTable.emplace(key, value);
//                     }
//                     else
//                     {
//                         _Float32 x = 1.0 - std::min(b / std::sqrt((c ^ 2) - (b ^ 2)), 1.0);
//                         _Float32 value = c * std::sqrt(1.0 + std::pow(1.0 - x, 2.0)) + bx;
//                         interpolTable.emplace(key, value);
//                     }
//                 }
//                 f = f + 1;
//             }
//             bi = bi + 1;
//         }
//         ci = ci + 1;
//     }
// }

// // ci is traversal cost index of cell with corners s, s1, s2
// // bi is traversal cost index of cell with corners s, s1 but not s2
// // c is the cell with corners s, s1, s2 (upper right)
// // b is the cell with corners s, s1 but not s2 (bottom right)
// // s1 is the corner to the middle right of s
// // s2 is the corner to the upper right of s
// // sa and sb are consecutive neighbors of s
// // g[s] is the path cost of node s to the goal
// _Float32 ComputeCost(LexiQueue::Node s, LexiQueue::Node sa, LexiQueue::Node sb)
// {
//     _Float32 vs;
//     if (sa is a diagonal neighbor of s)
//     {
//         s1 = sb;
//         s2 = sa;
//     }
//     else
//     {
//         s1 = sa;
//         s2 = sb;
//     }
//     c = cellcosts[ci];
//     b = cellcosts[bi];
//     if (std::min(c, b) == INF)
//     {
//         vs = INF;
//     }
//     else if (g[s1] <= g[s2])
//     {
//         vs = std::min(c, b) + g[s1];
//     }
//     else
//     {
//         _Float32 f = g[s1] - g[s2];
//         const std::array<int, 3UL> &key = {ci, bi, f};
//         if (f > std::min(c, b))
//         {
//             vs = interpolTable.at(key);
//         }
//         else
//         {
//             vs = interpolTable.at(key) + g[s2];
//         }
//     }
//     return vs;
// }

// ComputeKey(LexiQueue::Node s)
// {
//     return [min(g[s], rhs[s]) + h(sstart, s); min(g[s], rhs[s])];
// }

// void UpdateState(LexiQueue::Node s)
// {
//     if (s was not visited before)
//     {
//         g(s) = INF;
//     }
//     if (s != sgoal)
//     {
//         rhs(s) = min(stick, sticktick) in connbrs(s) ComputeCost(s, s0, s00);
//     }
//     if (s in OPEN)
//     {
//         remove s from OPEN;
//     }
//     if (g(s) != rhs(s))
//     {
//         insert s into OPEN with key(s);
//     }
// }

// void ComputeShortestPath()
// {
//     while (min s in OPEN(ComputeKey(s)) < ComputeKey(sstart) OR rhs[sstart] != g[sstart])
//     {
//         peek at state s with the minimum ComputeKey on OPEN;
//         if (g[s] >= rhs[s])
//         {
//             g[s] = rhs[s];
//             remove s from OPEN;
//             for (all stick in nbrs(s))
//             {
//                 if (stick was not visited before)
//                 {
//                     g[stick] = rhs[stick] = INF;
//                 }
//                 rhsold = rhs[stick];
//                 if (rhs[stick] > ComputeCost(stick, s, ccknbr(stick, s)))
//                 {
//                     rhs[stick] = ComputeCost(stick, s, ccknbr(stick, s));
//                     bptr[stick] = s;
//                 }
//                 if (rhs[stick] > ComputeCost(stick, s, cknbr(stick, s)))
//                 {
//                     rhs[stick] = ComputeCost(stick, cknbr(stick, s), s);
//                     bptr(stick) = cknbr(stick, s);
//                 }
//                 if (rhs[s] != rhsold)
//                 {
//                     UpdateState(stick);
//                 }
//             }
//         }
//         else
//         {
//             rhs[s] = min stick in nbrs(s) ComputeCost(s, stick, ccknbr(s, stick));
//             bptr(s) = argmin stick in nbrs(s) ComputeCost(s, stick, ccknbr(s, stick));
//             if (g[s] < rhs[s])
//             {
//                 g[s] = INF;
//                 for (all stick in nbrs(s))
//                 {
//                     if (bptr(stick) == s OR bptr(stick) == cknbr(stick, s))
//                     {
//                         if (rhs[stick] != ComputeCost(stick, bptr(stick), ccknbr(stick, bptr(stick))))
//                         {
//                             if (g[stick] < rhs[stick] OR stick not in OPEN)
//                             {
//                                 rhs[stick] = INF;
//                                 UpdateState(stick);
//                             }
//                             else
//                             {
//                                 rhs[stick] = min sticktick in nbrs(stick) ComputeCost(stick, sticktick, ccknbr(stick, sticktick));
//                                 bptr(stick) = argmin sticktick in nbrs(stick) ComputeCost(stick, sticktick, ccknbr(stick, sticktick));
//                                 UpdateState(stick);
//                             }
//                         }
//                     }
//                 }
//                 UpdateState(s);
//             }
//         }
//     }
// }

// void UpdateCellCost(x, uint8_t c)
// {
//     if (c > current traversal cost of x)
//     {
//         for each (state s on a corner of x)
//         {
//             if (either bptr(s) or ccknbr(s, bptr(s)) is a corner of x)
//             {
//                 if (rhs[s] != ComputeCost(s, bptr(s), ccknbr(s, bptr(s))))
//                 {
//                     if (g[s] < rhs[s] OR s not in OPEN)
//                     {
//                         rhs[s] = INF;
//                         UpdateState(s);
//                     }
//                     else
//                     {
//                         rhs[s] = min stick in nbrs(s) ComputeCost(s, stick, ccknbr(s, stick));
//                         bptr(s) = argmin stick in nbrs(s), ComputeCost(s, stick, ccknbr(s, stick));
//                         UpdateState(s);
//                     }
//                 }
//             }
//         }
//     }
//     else
//     {
//         rhsmin = INF;
//         for each (state s on a corner of x)
//         {
//             if (s was not visited before)
//             {
//                 g[s] = rhs[s] = INF;
//             }
//             else if (rhs[s] < rhsmin)
//             {
//                 rhsmin = rhs[s];
//                 sstar = s;
//             }
//         }
//         if (rhsmin != INF)
//             insert sstar into OPEN with ComputeKey(sstar);
//     }
// }

int main(int argc, char **argv)
{
    // ConstructInterpolationTable();
    // g[sstart] = rhs[sstart] = INF;
    // g[sgoal] = INF;
    // rhs[sgoal] = 0;
    // OPEN = empty;
    // insert sgoal into OPEN with ComputeKey(sgoal);
    // while (1)
    // {
    //     ComputeShortestPath();
    //     Wait for changes in cell traversal costs;
    //     for (all cells x with new traversal costs c)
    //     {
    //         UpdateCellCost(x, c);
    //     }
    // }
    return 0;
}
