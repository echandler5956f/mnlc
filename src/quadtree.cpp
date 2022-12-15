#include "quadtree.h"

using namespace std;
using namespace DStarLite;

#define __UNKNOWN_VAL__ 0xFFu
#define __EVEN_INIT_STATE__ 0x00u
#define __ODD_INIT_STATE__ 0x09u
#define __IS_LEAF__ 0x08u
#define __IS_LEAF_PARENT__ 0x00u
#define __STATE_MSK__ 0x03u
#define __NUM_STENCILS__ 0x0Fu
#define __INTERMEDIARY_PARENT__ 0x00u
#define __PARENT_STENCIL_0__ 0x04u
#define __PARENT_STENCIL_1__ 0x08u
#define __PARENT_STENCIL_2__ 0x0Cu
#define __PARENT_STENCIL_3__ 0x10u
#define __PARENT_STENCIL_4__ 0x14u
#define __PARENT_STENCIL_5__ 0x18u
#define __PARENT_STENCIL_6__ 0x1Cu
#define __PARENT_STENCIL_7__ 0x20u
#define __PARENT_STENCIL_8__ 0x24u
#define __PARENT_STENCIL_9__ 0x28u
#define __PARENT_STENCIL_10__ 0x2Cu
#define __PARENT_STENCIL_11__ 0x30u
#define __PARENT_STENCIL_12__ 0x34u
#define __PARENT_STENCIL_13__ 0x38u
#define __PARENT_STENCIL_14__ 0x3Cu
#define __REFINE_DEPTH__ 0x40u
#define __ZERO__ 0x00u
#define __BIT_ONE__ 0x01u
#define __BIT_TWO__ 0x02u
#define __BYTE_MSK__ 0xFF
#define __NUM_STENCILS__ 0x0Fu
#define __STENCIL_MSK__ 0x3Cu
#define __UP__ 0x01u
#define __RIGHT__ 0x02u
#define __DOWN__ 0x05u
#define __LEFT__ 0x06u
#define __NUM_CHILDREN__ 0x04u
#define __NUM_STATES__ 0x04u
#define __H__ 0x00u
#define __A__ 0x01u
#define __B__ 0x02u
#define __R__ 0x03u
#define __SAME_DEPTH_NBRS__ 0x00
#define __BOTTOM_LEFT__ 0x00u
#define __TOP_LEFT__ 0x01u
#define __TOP_RIGHT__ 0x02u
#define __BOTTOM__RIGHT__ 0x03u
#define __ORDERED_STATE_MSK__ 0x30u
#define __ORDERED_SHIFT_STATE_MSK__ 0x3F
#define __ID_SIZE__ 0x20u

const uint32_t Quadtree::LOWERMASK = 0x55555555u;
const uint8_t Quadtree::PREVSTATEMASK = 0x01u;
const uint32_t Quadtree::X_MASK = 0xFFFF0000u;
const uint32_t Quadtree::Y_MASK = 0x0000FFFFu;
const uint8_t Quadtree::STENCIL_NUM[5] = {7, 10, 4, 10, 13};

const uint8_t Quadtree::PRODUCTION_RULE[16] = {__RIGHT__, __RIGHT__, __DOWN__, __DOWN__, __UP__, __UP__, __LEFT__, __LEFT__, __RIGHT__, __RIGHT__, __DOWN__, __DOWN__, __UP__, __UP__, __LEFT__, __LEFT__};

const uint8_t Quadtree::INDEX_IN_PARENT_TO_GEO[4][4] = {
    {__BOTTOM_LEFT__, __TOP_LEFT__, __TOP_RIGHT__, __BOTTOM__RIGHT__},
    {__BOTTOM_LEFT__, __BOTTOM__RIGHT__, __TOP_RIGHT__, __TOP_LEFT__},
    {__TOP_RIGHT__, __TOP_LEFT__, __BOTTOM_LEFT__, __BOTTOM__RIGHT__},
    {__TOP_RIGHT__, __BOTTOM__RIGHT__, __BOTTOM_LEFT__, __TOP_LEFT__},
};

Quadtree::Quadtree(uint16_t sampleWidth_, uint8_t minLeafDepth_, uint8_t maxLeafDepth_, uint8_t unknownReplacement_, uint16_t maxVariance_)
{
    _sampleWidth = sampleWidth_;
    _minLeafDepth = minLeafDepth_;
    _maxLeafDepth = maxLeafDepth_;
    _unknownReplacement = unknownReplacement_;
    _maxVariance = maxVariance_;
    _sampleDepth = lg32(_sampleWidth);

    _buildLowestBitTable();
}

Quadtree::~Quadtree()
{
}

bool Quadtree::buildNodes(const vector<int8_t, allocator<int8_t>> &data_)
{
    _leafHash.clear();
    _leafParentHash.clear();

    uint32_t length = 0, t_id, mod;
    uint32_t index = _sampleWidth * (_sampleWidth - 1) - 1;
    uint8_t state, orderedStates, mean;
    uint8_t pOcc[4];

    state = _getState(_sampleDepth, length, __BYTE_MSK__);

    switch (PRODUCTION_RULE[state])
    {
    case __UP__:
        index -= _sampleWidth;
        break;
    case __RIGHT__:
        index++;
        break;
    case __DOWN__:
        index += _sampleWidth;
        break;
    case __LEFT__:
        index--;
        break;
    }
    _emplaceNode(_constructID(_sampleDepth, length), __IS_LEAF__ | (state & __STATE_MSK__), __SAME_DEPTH_NBRS__, data_[index]);
    length++;

    while (length < _getNumTiles(_sampleDepth) + 1)
    {
        t_id = _constructID(_sampleDepth, length);
        state = _getState(_sampleDepth, length, state);

        switch (PRODUCTION_RULE[state])
        {
        case __UP__:
            index -= _sampleWidth;
            break;
        case __RIGHT__:
            index++;
            break;
        case __DOWN__:
            index += _sampleWidth;
            break;
        case __LEFT__:
            index--;
            break;
        }

        mod = length & (__BIT_ONE__ | __BIT_TWO__);
        mean = data_[index];
        orderedStates = ((__ORDERED_SHIFT_STATE_MSK__ & orderedStates) << 2) | state;
        _emplaceNode(t_id, __IS_LEAF__ | (state & __STATE_MSK__), __SAME_DEPTH_NBRS__, mean);
        pOcc[mod] = mean;

        if (mod == 3)
        {
            t_id = _constructID(_sampleDepth - 1, length >> 2);
            mean = (pOcc[0] + pOcc[1] + pOcc[2] + pOcc[3]) >> 2;
            orderedStates = (__ORDERED_STATE_MSK__ & orderedStates) >> 4;
            _emplaceNode(t_id, __IS_LEAF_PARENT__ | __PARENT_STENCIL_8__ | orderedStates, __SAME_DEPTH_NBRS__, mean);
        }

        length++;
    }
}

bool Quadtree::updateNodes(const vector<int8_t, allocator<int8_t>> &data_)
{
    vector<uint8_t> cached;
    uint32_t nodeLength, nodeWidth, pos, start, tmp, end, it;
    uint16_t unknownCnt, checkpoint;
    uint8_t firstBit, nodeDepth, mean, occ, variance;
    bool replace;

    for (auto &[id, node] : _leafHash)
    {
        mean = unknownCnt = 0;
        replace = false;
        firstBit = _findFirstBits(id);
        nodeDepth = _extractDepthFromID(id, firstBit);
        nodeLength = _extractLengthFromID(id, firstBit);
        pos = _unhilbert(nodeDepth, nodeLength);
        nodeWidth = 1 << (_sampleDepth - nodeDepth);
        start = _hilbertCoordsToRowMajor(pos, _sampleWidth);
        end = _hilbertCoordsToRowMajor(pos, _sampleWidth, nodeWidth);
        checkpoint = (nodeWidth * nodeWidth) >> 1;
        it = start;
        occ = data_[start];

        if (occ == __UNKNOWN_VAL__)
        {
            unknownCnt++;
            occ = _unknownReplacement;
        }

        mean += occ;
        cached.push_back(occ);

        tmp = start;
        while (it != end)
        {
            if (it - tmp >= nodeWidth - 1)
            {
                it -= (nodeWidth + _sampleWidth - 1);
                tmp = it;
            }
            else
                it++;

            occ = data_[it];

            if (occ == __UNKNOWN_VAL__)
            {
                unknownCnt++;
                occ = _unknownReplacement;
            }

            mean += occ;
            cached.push_back(occ);

            if (unknownCnt > checkpoint)
                replace = true;
        }

        variance = 0;

        if (nodeDepth != _sampleDepth)
            for (uint16_t i = 0; i < cached.size(); i++)
                variance += (static_cast<int>(cached[i] - mean)) * static_cast<int>((cached[i] - mean));

        cached.clear();

        if (replace)
            node._occAgg = _unknownReplacement;
        else
            node._occAgg = mean >> lg32(nodeWidth * nodeWidth);

        if (static_cast<uint8_t>(round((((static_cast<float>(variance) / static_cast<float>(_maxVariance)) *
                                         (static_cast<float>(_maxLeafDepth) - static_cast<float>(_minLeafDepth))) +
                                        static_cast<float>(_minLeafDepth)))) > nodeDepth)
            node._occAgg |= __REFINE_DEPTH__;
        else
            node._occAgg &= (!__REFINE_DEPTH__);
    }

    vector<uint32_t> c;
    uint32_t cd, mask;
    uint8_t state, stencil, cc, gc, shift;

    for (auto &[id, node] : _leafParentHash)
    {
        mask = __BIT_ONE__;
        mean = occ = unknownCnt = cc = gc = cd = 0;
        replace = false;
        c.clear();
        firstBit = _findFirstBits(node._ID);
        nodeDepth = _extractDepthFromID(node._ID, firstBit);
        nodeLength = _extractLengthFromID(node._ID, firstBit);
        state = node._metadata & __STATE_MSK__;
        stencil = ((node._metadata & __STENCIL_MSK__) >> 2);
        for (uint8_t child = 0; child < __NUM_CHILDREN__; child++)
            if ((_stencilChildTable[state][stencil - 1] >> child) & __BIT_ONE__) // -1 because stencil 0 is intermediary parent
                for (uint8_t gchild = 0; gchild < __NUM_CHILDREN__; gchild++)
                {
                    c.push_back((((nodeLength << 2) + child) << 2) + gchild); // skipping over the intermediate parent
                    cd |= __BIT_TWO__ << mask;
                    mask <<= 2;
                    gc++;
                }
            else
            {
                c.push_back((nodeLength << 2) + child);
                cd |= __BIT_ONE__ << mask;
                mask <<= 2;
                cc++;
            }

        checkpoint = ((gc << 2) + cc) >> 1;
        mask = (__BIT_ONE__ | __BIT_TWO__);
        for (uint8_t i = 0; i < cc + gc; i++)
        {
            shift = (((cd & mask) >> (i << 1)) == __BIT_TWO__) ? __BIT_TWO__ : __ZERO__;
            mask <<= 2;
            occ = _leafHash.at(c[i])._occAgg;
            if (occ == __UNKNOWN_VAL__)
            {
                unknownCnt += 1 << shift;
                occ = _unknownReplacement;
            }

            mean += occ << shift;

            if (unknownCnt > checkpoint)
                replace = true;
        }

        if (replace)
            node._occAgg = _unknownReplacement;
        else
            node._occAgg = mean >> 4;
    }

    return true;
}

void Quadtree::_buildLowestBitTable()
{
    for (uint16_t i = 0; i < 256; i++)
        _lowestBitTable[i] = _getLowestSetBit(i);
}

uint32_t Quadtree::_constructID(uint8_t depth, uint32_t length)
{
    uint32_t ID = length << (_getLevelBitOffset(depth) - 1);
    ID |= (1 << (_getLevelBitOffset(depth) - 2));
    return (ID);
}

bool Quadtree::_emplaceNode(uint32_t id, uint8_t metadata, uint8_t lvlDiff_, uint8_t occ)
{
    if ((metadata & __IS_LEAF__))
        _leafHash.emplace(piecewise_construct,
                          forward_as_tuple(id),
                          forward_as_tuple(id, metadata, lvlDiff_, occ));
    else
        _leafParentHash.emplace(piecewise_construct,
                                forward_as_tuple(id),
                                forward_as_tuple(id, metadata, lvlDiff_, occ));
    return true;
}

uint8_t Quadtree::_extractDepthFromID(uint32_t id, uint8_t firstBit)
{
    if (firstBit < __ID_SIZE__)
        return (firstBit >> 1);
    else
        return ((__ID_SIZE__ - _findFirstBits(id)) - 1) >> 1;
}

uint32_t Quadtree::_extractLengthFromID(uint32_t id, uint8_t firstBit)
{
    if (firstBit < __ID_SIZE__)
        return id >> firstBit;
    else
        return id >> _findFirstBits(id);
}

uint8_t Quadtree::_findFirstBits(uint32_t value)
{
    uint8_t *bytes = (uint8_t *)&value;
    if (bytes[0])
        return _lowestBitTable[bytes[0]];
    else if (bytes[1])
        return _lowestBitTable[bytes[1]] + 8;
    else if (bytes[2])
        return _lowestBitTable[bytes[2]] + 16;
    else
        return _lowestBitTable[bytes[3]] + 24;
}

void Quadtree::_findNeighbors(uint32_t id, uint8_t depth, uint32_t length)
{
    uint32_t parent_id, l, pl;
    uint8_t index_in_parent, parent_stencil, d, pd;

    if (depth != 0xFFFFu && length != 0xFFFFFFFFu)
    {
        d = depth;
        l = length;
    }
    else
    {
        uint8_t firstBit = _findFirstBits(id);
        d = _extractDepthFromID(id, firstBit);
        l = _extractLengthFromID(id, firstBit);
    }
    pd = d - 1;
    pl = l >> 2;

    parent_id = _constructID(pd, pl);
    index_in_parent = l & (__BIT_ONE__ | __BIT_TWO__);

    parent_stencil = ((_leafParentHash.at(parent_id)._metadata) & __STENCIL_MSK__) >> 2;

    if (parent_stencil == __INTERMEDIARY_PARENT__)
    {
        uint32_t grandparent_id, gl;
        uint8_t index_in_grandparent, grandparent_stencil, gd;

        gd = pd - 1;
        gl = pl >> 2;

        grandparent_id = _constructID(gd, gl);
        index_in_grandparent = pl & (__BIT_ONE__ | __BIT_TWO__);

        grandparent_stencil = ((_leafParentHash.at(grandparent_id)._metadata) & __STENCIL_MSK__) >> 2;
    }
}

void _neighborRules(uint32_t id, uint8_t depth, uint32_t length, uint8_t state, uint8_t facet, uint8_t direction, uint8_t neighbor_lvl)
{
    /*

    get parent id
    get parent stencil
    if lvl = neighbor lvl {
        if neighbor direction is within stencil {
            return neighbor id
        }

        if state = H {
            if (direction = up) {
                return neighbor id
                }
        }

        if state = A {
            if (direction = right) {
                return neighbor id
                }

        if (facet = bottom right && direction = top right) {
                return neighbor id
                }
        if (facet = top right && direction = bottom right) {
                return neighbor id
                }
        }

        if state = B {
            if (direction = left) {
                return neighbor id
                }

        if (facet = bottom left && direction = top left) {
                return neighbor id
                }
        if (facet = top left && direction = bottom left) {
                return neighbor id
                }
        }

        if state = R {
            if (direction = down) {
                return neighbor id
                }
        }

    }

    */
}

//  /**
//    * Neighbor-finding algorithm with worst-case complexity O(level) and
//    * average-case complexity O(1)
//    */
//   uint32_t neighbor(uint8_t level, uint32_t position, uint32_t state, uint32_t facet) {
//     static const uint32_t stateMaskTable[] = {1, 0, 0, 2};
//     uint rem = position % 4;
//     uint32_t pState = state ^ stateMaskTable[rem];

//     auto neighborIndex = nTable[rem][pState][facet];
//     if (neighborIndex != -1) {
//       uint32_t resultingIndex = position - rem + neighborIndex;
//       return resultingIndex;
//     }

//     size_t quot = position / 4;  // TODO: replace by shift

//     for (size_t i = 1; i < level; ++i) {
//       state = pState;
//       levelTables[i] = &oTable[rem][state][0][0];

//       rem = quot % 4;
//       quot = quot / 4;

//       pState = state ^ stateMaskTable[rem];

//       neighborIndex = nTable[rem][pState][facet];
//       if (neighborIndex != -1) {
//         state = pState ^ stateMaskTable[neighborIndex];
//         quot = quot * 4 + neighborIndex;
//         for (; i > 0; --i) {
//           auto childIndex = levelTables[i][4 * state + facet];
//           quot = quot * 4 + childIndex;
//           state = state ^ stateMaskTable[childIndex];
//         }
//         return quot;
//       }
//     }

//     return -1;
//   }

uint32_t Quadtree::_getNumTiles(uint8_t depth)
{
    return 1 << (depth << 1);
}

uint32_t Quadtree::_getLevelBitOffset(uint8_t depth)
{
    return __ID_SIZE__ - (depth << 1);
}

uint8_t Quadtree::_getLowestSetBit(uint8_t num)
{
    uint32_t mask = 1;
    for (uint8_t cnt = 1; cnt <= 32; cnt++, mask <<= 1)
        if (num & mask)
            return cnt;
    return 0;
}

uint8_t Quadtree::_getState(uint8_t depth, uint32_t length, uint8_t prevState)
{
    uint32_t flipMask = LOWERMASK >> _getLevelBitOffset(depth);
    uint32_t a = length & LOWERMASK;
    uint32_t b = (length >> 1) & LOWERMASK;
    uint32_t newState = ((__builtin_popcount(a & b) & 1) << 1) + (__builtin_popcount(flipMask ^ (a | b)) & 1);
    if (prevState == __BYTE_MSK__)
        return depth & 0x01u == 0x00u ? __EVEN_INIT_STATE__ : __ODD_INIT_STATE__;
    return ((prevState & PREVSTATEMASK) << 2) | newState;
}

uint32_t Quadtree::_hilbertCoordsToRowMajor(uint32_t pos, uint32_t offset1, uint32_t offset2)
{
    return ((offset1 - (pos & Y_MASK) + offset2) * _sampleWidth) + (pos >> (__ID_SIZE__ >> 1) - offset1 + offset2);
}

uint32_t Quadtree::_unhilbert(uint8_t depth, uint32_t length)
{
    uint32_t numSamplesInID = _getNumTiles(_sampleDepth - depth);
    uint32_t l = length;

    uint32_t lo = l & 0x55555555;
    uint32_t hi = l & 0xaaaaaaaa;
    uint32_t marks = (lo ^ hi >> 1) | (hi & lo << 1);

    marks ^= (marks & 0xcccccccc) >> 2;
    marks ^= (marks & 0x30303030) >> 4;
    marks ^= (marks & 0x03000300) >> 8;
    marks ^= (marks & 0x00030000) >> 16;
    marks ^= (marks & 0x00030000) >> 8;
    marks ^= (marks & 0x03030300) >> 4;
    marks ^= (marks & 0x33333330) >> 2;

    l ^= marks & 0xaaaaaaaa;

    l ^= (l & 0xaaaaaaaa) >> 1;

    uint32_t fmask = 0x44444444 ^ (marks & 0x55555555);
    fmask |= fmask << 1;
    l = (l & ~fmask) | (fmask & (((l & 0x55555555) << 1) |
                                 ((l & 0xaaaaaaaa) >> 1)));

    uint16_t x = _unhilbertDeinterleave(l >> 1) * numSamplesInID;
    uint16_t y = _unhilbertDeinterleave(l) * numSamplesInID;

    return ((x << (__ID_SIZE__ >> 1)) | y);
}

uint32_t Quadtree::_unhilbertDeinterleave(uint32_t l)
{
    l = l & 0x55555555;
    l = (l | (l >> 1)) & 0x33333333;
    l = (l | (l >> 2)) & 0x0f0f0f0f;
    l = (l | (l >> 4)) & 0x00ff00ff;
    l = (l | (l >> 8)) & 0x0000ffff;
    return l;
}

Quadtree::BaseNode::BaseNode(uint32_t ID_, uint8_t metadata_, uint8_t lvlDiff_, int8_t occ_)
{
    _ID = ID_;
    _metadata = metadata_;
    _lvlDiff = lvlDiff_;
    _occAgg = occ_;
}

Quadtree::BaseNode::~BaseNode()
{
}