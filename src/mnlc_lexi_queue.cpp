#include "mnlc_lexi_queue.h"

// namespace LexiQueue
// {
//     class NodeComperator
//     {
//     public:
//         bool operator()(const Node *lhs, const Node *rhs)
//         {
//             return lhs->key > rhs->key;
//         }
//     };

//     class PriorityQueue final : public std::priority_queue<Node *, std::vector<Node *>, NodeComperator>
//     {
//     public:
//         void erase(const Node *node)
//         {
//             auto it = std::find_if(c.begin(), c.end(),

//                                    [&node](const Node *element)
//                                    {
//                                        return *element == *node;
//                                    });
//             c.erase(it);
//         }

//         void reset()
//         {
//             c.clear();
//         }

//         Key topKey() const
//         {
//             if (empty())
//             {
//                 return Key();
//             }
//             return top()->key;
//         }

//         Node *pop()
//         {
//             auto topNode = top();
//             pop();
//             return topNode;
//         }

//         const bool contains(const Node *node)
//         {
//             return std::find(c.begin(), c.end(), node) != c.end();
//         }

//     private:
//         using std::priority_queue<Node *, std::vector<Node *>, NodeComperator>::pop;
//     };
// }