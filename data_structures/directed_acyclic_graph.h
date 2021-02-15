#ifndef DIRECTED_ACYCLIC_GRAPH_H
#define DIRECTED_ACYCLIC_GRAPH_H

#include <cg3/geometry/point2.h>
#include <cg3/geometry/segment2.h>
#include <cg3/geometry/intersections2.h>
#include "trapezoidalmap_dataset.h"
#include "algorithms/lies_above.h"
#include "trapezoid.h"

/**
 * @brief This class allows to perform the point location task for a trapezoidal map in an efficient way.
 */
class DAG {

    typedef std::pair<size_t, size_t> IndexedSegment2d;

    enum Type {internal_node, leaf}; // Two types of nodes
    enum InternalNodeType {x_node, y_node}; // Two types of internal nodes

    typedef struct {
        InternalNodeType internalNodeType; // (x_node: point / y_node: segment)
        size_t index; // Index of the point or segment (It depends on the value of internalNodeType)
        size_t leftChild; // Index to the left child
        size_t rightChild; // Index to the right child
    } InternalNode;

    union Value { // The value of the union is based on the type (either internal_node or leaf)
        InternalNode internalNode; // We will use internalNode if type == internal_node
        size_t trapezoidIndex; // trapezoidIndex if type == leaf
    };

    typedef struct Node { // DAG's node
        Type type; // (internal_node / leaf)
        Value v; // internalNode or trapezoidIndex
    } Node;

public:
    DAG(const std::vector<cg3::Point2d>& points, const std::vector<IndexedSegment2d>& indexedSegments);
    void init(size_t trapezoidIndex);
    cg3::Point2d getPoint(size_t index) const;
    cg3::Segment2d getSegment(size_t index) const;
    const std::vector<Node>& getNodes() const;
    void update(size_t nodeToDelete, const size_t segmentIndex, std::vector<std::pair<size_t, size_t>>& indices, bool isFirst);
    void update(const size_t segmentIndex, std::vector<size_t>& trapezoidindices, size_t nodeToDelete, Trapezoid& trapezoid);
    size_t find(const cg3::Segment2d& segment) const;
    size_t query(const cg3::Point2d& queryPoint) const;
    void clear();

private:
    size_t root; // Index to the root
    std::vector<Node> nodes; // DAG nodes
    const std::vector<cg3::Point2d>& points; // Points reference
    const std::vector<IndexedSegment2d>& indexedSegments; // Indexed segments reference
};

#endif // DIRECTED_ACYCLIC_GRAPH_H
