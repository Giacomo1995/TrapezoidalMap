#include "directed_acyclic_graph.h"

#include "algorithms/slope.h"

/**
 * Constructor.
 *
 * @param[in] points and indexed segments.
 */
DAG::DAG(const std::vector<cg3::Point2d>& points, const std::vector<TrapezoidalMapDataset::IndexedSegment2d>& indexedSegments) : points{points}, indexedSegments{indexedSegments} {
}


/**
 * Sets the value of the root and creates the first node.
 *
 * @param[in] the index of the trapezoid the root node is going to refer to.
 */
void DAG::init(size_t trapezoidIndex) {
    root = 0;

    Node firstNode;
    firstNode.type = leaf;
    firstNode.v.trapezoidIndex = trapezoidIndex;

    nodes.push_back(firstNode);
}

/**
 * Returns the point associated with the provided index.
 *
 * @param[in] index of the point.
 * @return Point located at the provided index.
 */
cg3::Point2d DAG::getPoint(size_t index) const {
    return points[index];
}

/**
 * Returns the segment associated with the provided index.
 *
 * @param[in] index of the segment.
 * @return Segment.
 */
cg3::Segment2d DAG::getSegment(size_t index) const {
    const IndexedSegment2d& indexedSegment = indexedSegments[index];

    cg3::Point2d leftEndPoint = getPoint(indexedSegment.first);
    cg3::Point2d rightEndPoint = getPoint(indexedSegment.second);
    cg3::Segment2d segment(leftEndPoint, rightEndPoint);

    return segment;
}

/**
 * Returns a reference to the nodes of the DAG.
 *
 * @return Reference to vector of nodes.
 */
const std::vector<DAG::Node>& DAG::getNodes() const {
    return nodes;
}

/**
 * Updates the DAG through the insertion of the new nodes and the deletion of the old one.
 *
 * @param[in] node to delete, the index of the inserted segment, a vector of pairs of indices (trapezoid, associated leaf) and a boolean to understand whether it is the first trapezoid intersected or not.
 */
void DAG::update(size_t nodeToDelete, const size_t segmentIndex, std::vector<IndexedSegment2d>& indices, bool isFirst) {
    const size_t& p = indexedSegments[segmentIndex].first;
    const size_t& q = indexedSegments[segmentIndex].second;

    // Generating the new leaf nodes
    for (const IndexedSegment2d& currentIndex : indices) {
        if (!(currentIndex.second < nodes.size() && nodes[currentIndex.second].v.trapezoidIndex == currentIndex.first)) {
            Node newNode;
            newNode.type = leaf;
            newNode.v.trapezoidIndex = currentIndex.first;
            nodes.push_back(newNode);
        }
    }

    // Generating the internal nodes
    if (indices.size() == 3 && isFirst) { // First trapezoid (without shared points)

        Node sNode = Node{internal_node, {{y_node, segmentIndex, indices[1].second, indices[2].second}}};
        nodes.push_back(sNode);

        Node pNode = Node{internal_node, {{x_node, p, indices[0].second, nodes.size()-1}}};
        nodes[nodeToDelete] = pNode;

    } else if (indices.size() == 3) { // Last trapezoid (without shared points)

        Node sNode = Node{internal_node, {{y_node, segmentIndex, indices[0].second, indices[1].second}}};
        nodes.push_back(sNode);

        Node qNode = Node{internal_node, {{x_node, q, nodes.size()-1, indices[2].second}}};
        nodes[nodeToDelete] = qNode;

    } else { // All the other cases

        Node sNode = Node{internal_node, {{y_node, segmentIndex, indices[0].second, indices[1].second}}};
        nodes[nodeToDelete] = sNode;

    }

}


/**
 * Updates the DAG through the insertion of the new nodes and the deletion of the old one.
 *
 * @param[in] index of the inserted segment, a vector of trapezoid indices, the node to delete and the trapezoid itself.
 */
void DAG::update(const size_t segmentIndex, std::vector<size_t>& trapezoidIndeces, size_t nodeToDelete, Trapezoid& trapezoid) {
    size_t p = indexedSegments[segmentIndex].first;
    size_t q = indexedSegments[segmentIndex].second;

    // Generating the new leaf nodes
    for (const size_t& currentTrapezoidIndex : trapezoidIndeces) {
        Node newNode;
        newNode.type = leaf;
        newNode.v.trapezoidIndex = currentTrapezoidIndex;
        nodes.push_back(newNode);
    }

    // Generating the internal nodes
    if (trapezoidIndeces.size() == 4) { // Insertion of four trapezoids (segment completely contained in a trapezoid)

        Node sNode = Node{internal_node, {{y_node, segmentIndex, nodes.size()-2, nodes.size()-1}}};
        nodes.push_back(sNode);

        Node qNode = Node{internal_node, {{x_node, q, nodes.size()-1, nodes.size()-4}}};
        nodes.push_back(qNode);

        Node pNode = Node{internal_node, {{x_node, p, nodes.size()-6, nodes.size()-1}}};

        nodes[nodeToDelete] = pNode;

    } else if (trapezoidIndeces.size() == 3) { // In this case I need to insert three trapezoids only (due to a shared point)

        if (p == trapezoid.getLeftp() && q != trapezoid.getRightp()) {

            Node sNode = Node{internal_node, {{y_node, segmentIndex, nodes.size()-2, nodes.size()-1}}};
            nodes.push_back(sNode);

            Node qNode = Node{internal_node, {{x_node, q, nodes.size()-1, nodes.size()-4}}};
            nodes[nodeToDelete] = qNode;

        } else {

            Node sNode = Node{internal_node, {{y_node, segmentIndex, nodes.size()-2, nodes.size()-1}}};
            nodes.push_back(sNode);

            Node pNode = Node{internal_node, {{x_node, p, nodes.size()-4, nodes.size()-1}}};
            nodes[nodeToDelete] = pNode;

        }

    } else { // Insertion of two trapezoids (two shared points)

        Node sNode = Node{internal_node, {{y_node, segmentIndex, nodes.size()-2, nodes.size()-1}}};
        nodes[nodeToDelete] = sNode;

    }
}

/**
 * Finds the first trapezoid in which the left endpoint of the inserted segment lies.
 *
 * @param[in] inserted segment.
 * @return Index of the found trapezoid.
 */
size_t DAG::find(const cg3::Segment2d& segment) const {
    const size_t& max = std::numeric_limits<size_t>::max();
    size_t trapezoidIndex = max;
    bool found = false;
    const cg3::Point2d& p = segment.p1();
    size_t index = root;

    while (index != max && !found) {

        switch (nodes[index].type) {

            case leaf:

                trapezoidIndex = nodes[index].v.trapezoidIndex;
                found = true;

                break;

            case internal_node:

                switch(nodes[index].v.internalNode.internalNodeType) {

                    case x_node:

                        if (p.x() < (points[nodes[index].v.internalNode.index]).x())
                            index = nodes[index].v.internalNode.leftChild;
                        else
                            index = nodes[index].v.internalNode.rightChild;

                        break;

                    case y_node:

                        const cg3::Segment2d& s = getSegment(nodes[index].v.internalNode.index);

                        if (s.p1() == segment.p1()) { // Shared point (slope computation needed)

                            if (slope(segment) > slope(s)) // p lies above
                                index = nodes[index].v.internalNode.leftChild;
                            else // p lies below
                                index = nodes[index].v.internalNode.rightChild;

                        } else { // No shared point (standard procedure)

                            if (liesAbove(p, s))
                                index = nodes[index].v.internalNode.leftChild;
                            else
                                index = nodes[index].v.internalNode.rightChild;

                        }

                        break;
                }

                break;

        }

    }

    return trapezoidIndex;
}

/**
 * Finds the trapezoid in which the query point lies.
 *
 * @param[in] the query point.
 * @return Index of the found trapezoid.
 */
size_t DAG::query(const cg3::Point2d& queryPoint) const {
    const size_t& max = std::numeric_limits<size_t>::max();
    size_t trapezoidIndex = max;
    bool found = false;
    size_t index = root;

    while (index != max && !found) {

        switch (nodes[index].type) {

            case leaf:

                trapezoidIndex = nodes[index].v.trapezoidIndex;
                found = true;

                break;

            case internal_node:

                switch(nodes[index].v.internalNode.internalNodeType) {

                    case x_node:

                        if (queryPoint.x() < (points[nodes[index].v.internalNode.index]).x())
                            index = nodes[index].v.internalNode.leftChild;
                        else
                            index = nodes[index].v.internalNode.rightChild;

                        break;

                    case y_node:

                        const cg3::Segment2d& s = getSegment(nodes[index].v.internalNode.index);

                        if (liesAbove(queryPoint, s))
                            index = nodes[index].v.internalNode.leftChild;
                        else
                            index = nodes[index].v.internalNode.rightChild;

                        break;

                }

                break;

        }

    }

    return trapezoidIndex;
}

/**
 * Deletes the items of the DAG.
 */
void DAG::clear() {
    nodes.clear();
}
