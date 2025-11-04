#pragma once

#include "node.hpp"
#include "nanoflann/nanoflann.hpp"

namespace RRT {
    struct NodeCloud {
        std::vector<Node*>& nodes;

        NodeCloud(std::vector<Node*>& nodes) 
            : nodes(nodes)
        {}

        inline size_t kdtree_get_point_count() const { return nodes.size(); }

        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            if (dim == 0)
                return nodes[idx]->point.x();
            else
                return nodes[idx]->point.y();
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /* bb */) const
        {
            return false;
        }
    };

    class NodeManager {
        public:
        using KDTree_t = nanoflann::KDTreeSingleIndexDynamicAdaptor<
                nanoflann::L2_Simple_Adaptor<double, NodeCloud>,
                NodeCloud, 2
            >;

        NodeCloud nodeCloud;
        std::vector<Node*> nodes;
        size_t size;
        KDTree_t index;

        NodeManager(size_t reserveSize)
            : nodeCloud(nodes), index(2, nodeCloud, {10}), size(0)
        {
            nodes.reserve(reserveSize);
        }

        void addNode(Node* node) {
            nodes.push_back(node);
            index.addPoints(nodes.size() - 1, nodes.size() - 1);
            size++;
        }

        Node* nearestNeighbor(const Vec2d& point) const {
            size_t retIndex;
            double retDistanceSquared;
            double query_point[2] = {point.x(), point.y()};

            nanoflann::KNNResultSet<double> resultSet(1);
            resultSet.init(&retIndex, &retDistanceSquared);
            index.findNeighbors(resultSet, query_point, nanoflann::SearchParameters());

            return nodes[retIndex];
        }

        void radiusSearch(std::vector<Node*>& nodeList, double radius, const Vec2d& point) const {
            const double radiusSqr = radius * radius;
            std::vector<nanoflann::ResultItem<size_t, double>> retIndicesDists;
            double query_point[2] = {point.x(), point.y()};

            nanoflann::RadiusResultSet<double> resultSet(radiusSqr, retIndicesDists);
            index.findNeighbors(resultSet, query_point);

            for (nanoflann::ResultItem<size_t, double> result : retIndicesDists) {
                nodeList.push_back(nodes[result.first]);
            }
        }
    };
};