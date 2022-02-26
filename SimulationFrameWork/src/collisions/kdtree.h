#ifndef POSITIONBASEDDYNAMICS_KDTREE_H
#define POSITIONBASEDDYNAMICS_KDTREE_H

#include <Eigen/Dense>

#include <vector>
#include <array>
#include <numeric>
#include <queue>
#include <algorithm>
#include <functional>

namespace HINASIM
{
    template<typename HullType>
    class KDTree
    {
    public:
        struct Node
        {
            Node(unsigned int b_, unsigned int n_)
                    : children({{-1, -1}}), begin(b_), n(n_)
            {}

            Node() = default;

            [[nodiscard]] bool is_leaf() const
            { return children[0] < 0 && children[1] < 0; }

            std::array<int, 2> children;

            // Index according entries in entity list.
            unsigned int begin;

            // Number of owned entries.
            unsigned int n;
        };

    public:

        explicit KDTree(std::size_t n, unsigned int max_primitives_per_leaf = -1) : list_(n), max_primitives_per_leaf_(max_primitives_per_leaf)
        {}

    public:
        [[nodiscard]] unsigned int entity(unsigned int i) const
        { return list_[i]; }

        [[nodiscard]] Node const &node(unsigned int i) const
        { return nodes_[i]; }

        [[nodiscard]] HullType const &hull(unsigned int i) const
        { return hulls_[i]; }

    public:
        using TraversalPredicate = std::function<bool(unsigned int node_index, unsigned int depth)>;
        using TraversalCallback = std::function<void(unsigned int node_index, unsigned int depth)>;
        using TraversalPriorityLess = std::function<bool(std::array<int, 2> const &nodes)>;
        struct QueueItem
        {
            unsigned int n, d;
        };
        using TraversalQueue = std::queue<QueueItem>;

        void construct();

        void traverse_depth_first(TraversalPredicate pred, TraversalCallback cb, TraversalPriorityLess const &pless = nullptr) const;

        void traverse_breadth_first(TraversalPredicate const &pred, TraversalCallback const &cb, unsigned int start_node = 0, TraversalPriorityLess const &pless = nullptr, TraversalQueue &pending = TraversalQueue()) const;

        void update();

    protected:
        void construct(unsigned int node, Eigen::AlignedBox<double, 3> const &box, unsigned int b, unsigned int n);

        void traverse_depth_first(unsigned int node, unsigned int depth, TraversalPredicate pred, TraversalCallback cb, TraversalPriorityLess const &pless) const;

        void traverse_breadth_first(TraversalQueue &pending, TraversalPredicate const &pred, TraversalCallback const &cb, TraversalPriorityLess const &pless = nullptr) const;

        unsigned int add_node(unsigned int b, unsigned int n);

    protected:
        [[nodiscard]] virtual Eigen::Vector3d entity_position(unsigned int i) const = 0;

        virtual void compute_hull(unsigned int b, unsigned int n, HullType &hull) const = 0;

        virtual void compute_hull_approx(unsigned int b, unsigned int n, HullType &hull) const
        { compute_hull(b, n, hull); }

    protected:
        std::vector<unsigned int> list_;
        std::vector<Node> nodes_;
        std::vector<HullType> hulls_;
        unsigned int max_primitives_per_leaf_;
    };


    template<typename HullType>
    void KDTree<HullType>::construct()
    {
        nodes_.clear();
        hulls_.clear();
        if (list_.empty())
            return;

        std::iota(list_.begin(), list_.end(), 0);

        auto box = Eigen::AlignedBox<double, 3>{};
        for (int i = 0; i < list_.size(); ++i)
            box.extend(entity_position(i));

        auto ni = add_node(0, static_cast<unsigned int>(list_.size()));
        construct(ni, box, 0, static_cast<unsigned int>(list_.size()));
    }

    template<typename HullType>
    void
    KDTree<HullType>::traverse_depth_first(TraversalPredicate pred, TraversalCallback cb,
                                           TraversalPriorityLess const &pless) const
    {
        if (nodes_.empty())
            return;

        if (pred(0, 0))
            traverse_depth_first(0, 0, pred, cb, pless);
    }

    template<typename HullType>
    void KDTree<HullType>::traverse_breadth_first(TraversalPredicate const &pred, TraversalCallback const &cb, unsigned int start_node, TraversalPriorityLess const &pless, TraversalQueue &pending) const
    {
        cb(start_node, 0);
        if (pred(start_node, 0)) pending.push({start_node, 0});
        traverse_breadth_first(pending, pred, cb, pless);
    }

    template<typename HullType>
    void KDTree<HullType>::update()
    {
        traverse_depth_first(
                [&](unsigned int, unsigned int)
                { return true; },
                [&](unsigned int node_index, unsigned int)
                {
                    auto const &nd = node(node_index);
                    compute_hull_approx(nd.begin, nd.n, hulls_[node_index]);
                }
        );
    }

    template<typename HullType>
    void KDTree<HullType>::construct(unsigned int node, Eigen::AlignedBox<double, 3> const &box, unsigned int b, unsigned int n)
    {
        // If only one element is left end recursion.
        //if (n == 1) return;
        if (n <= max_primitives_per_leaf_)
            return;

        auto d = box.diagonal().eval();
        auto max_dir = 0;
        if (d(1) >= d(0) && d(1) >= d(2))
            max_dir = 1;
        else if (d(2) >= d(0) && d(2) >= d(1))
            max_dir = 2;

#ifdef _DEBUG
        for (auto i = 0u; i < n; ++i)
    {
        if (!box.contains(entity_position(list_[b + i])))
            std::cerr << "ERROR: Bounding box wrong!" << std::endl;
    }
#endif

        // Sort range according to center of the longest side.
        std::sort(list_.begin() + b, list_.begin() + b + n, [&](unsigned int a, unsigned int b)
        {
            return entity_position(a)(max_dir) < entity_position(b)(max_dir);
        });

        unsigned int half_n = n / 2;
        auto n0 = add_node(b, half_n);
        auto n1 = add_node(b + half_n, n - half_n);
        nodes_[node].children[0] = n0;
        nodes_[node].children[1] = n1;

        auto c = static_cast<double >(0.5) * (entity_position(list_[b + half_n - 1])(max_dir) + entity_position(list_[b + half_n])(max_dir));
        auto l_box = box;
        l_box.max()(max_dir) = c;
        auto r_box = box;
        r_box.min()(max_dir) = c;

        construct(nodes_[node].children[0], l_box, b, half_n);
        construct(nodes_[node].children[1], r_box, b + half_n, n - half_n);
    }

    template<typename HullType>
    void
    KDTree<HullType>::traverse_depth_first(unsigned int node_index,
                                           unsigned int depth, TraversalPredicate pred, TraversalCallback cb,
                                           TraversalPriorityLess const &pless) const
    {
        Node const &node = nodes_[node_index];

        cb(node_index, depth);
        auto is_pred = pred(node_index, depth);
        if (!node.is_leaf() && is_pred)
        {
            if (pless && !pless(node.children))
            {
                traverse_depth_first(nodes_[node_index].children[1], depth + 1, pred, cb, pless);
                traverse_depth_first(nodes_[node_index].children[0], depth + 1, pred, cb, pless);
            } else
            {
                traverse_depth_first(nodes_[node_index].children[0], depth + 1, pred, cb, pless);
                traverse_depth_first(nodes_[node_index].children[1], depth + 1, pred, cb, pless);
            }
        }
    }

    template<typename HullType>
    void KDTree<HullType>::traverse_breadth_first(TraversalQueue &pending, TraversalPredicate const &pred, TraversalCallback const &cb, TraversalPriorityLess const &pless) const
    {
        while (!pending.empty())
        {
            auto n = pending.front().n;
            auto d = pending.front().d;
            auto const &node = nodes_[n];
            pending.pop();

            cb(n, d);
            auto is_pred = pred(n, d);
            if (!node.is_leaf() && is_pred)
            {
                if (pless && !pless(node.children))
                {
                    pending.push({static_cast<unsigned int>(node.children[1]), d + 1});
                    pending.push({static_cast<unsigned int>(node.children[0]), d + 1});
                } else
                {
                    pending.push({static_cast<unsigned int>(node.children[0]), d + 1});
                    pending.push({static_cast<unsigned int>(node.children[1]), d + 1});
                }
            }
        }
    }

    template<typename HullType>
    unsigned int KDTree<HullType>::add_node(unsigned int b, unsigned int n)
    {
        HullType hull;
        compute_hull(b, n, hull);
        hulls_.push_back(hull);
        nodes_.push_back({b, n});
        return static_cast<unsigned int>(nodes_.size() - 1);
    }
}

#endif //POSITIONBASEDDYNAMICS_KDTREE_H
