#ifndef mnlc_lexi_queue_H
#define mnlc_lexi_queue_H
#include <functional>
#include <algorithm>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>

namespace LexiQueue
{

    struct Key
    {
        std::tuple<_Float32, _Float32> val;
    };

    struct Node
    {
        std::tuple<int16_t, int16_t> coords;
        Key key;
    };
    class NodeComperator
    {
    public:
        bool operator()(const Node *lhs, const Node *rhs);
    };

    class PriorityQueue final : public std::priority_queue<Node *, std::vector<Node *>, NodeComperator>
    {
    public:
        void erase(const Node *node);
        void reset();
        Key topKey() const;
        Node *pop();
        const bool contains(const Node *node);

    private:
        using std::priority_queue<Node *, std::vector<Node *>, NodeComperator>::pop;
    };
}
#endif