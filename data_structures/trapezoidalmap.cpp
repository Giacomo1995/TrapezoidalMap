#include "trapezoidalmap.h"

#include "algorithms/lies_above.h"

/**
 * Constructor.
 *
 * @param[in] minimum and maximum vertex of the Bounding Box in order to generate the first trapezoid.
 */
TrapezoidalMap::TrapezoidalMap(const cg3::Point2d& boundingBoxMin, const cg3::Point2d& boundingBoxMax) : boundingBox(cg3::Point2d(0,0), cg3::Point2d(0,0)), dag(this->getPoints(), this->getIndexedSegments()) {
    const size_t& max = std::numeric_limits<size_t>::max();
    dag.init(0); // It creates the first node (a leaf) of the DAG

    // Vertices of the Bounding Box
    cg3::Point2d p0(boundingBoxMin);
    cg3::Point2d p1(boundingBoxMin.x(), boundingBoxMax.y());
    cg3::Point2d p2(boundingBoxMax);
    cg3::Point2d p3(boundingBoxMax.x(), boundingBoxMin.y());

    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);

    // First trapezoid (corresponding to the Bounding Box)
    Trapezoid firstTrapezoid(max, max, 0,  2, 0);
    firstTrapezoid.setAdjacencies(max, max, max, max);
    trapezoids.push_back(firstTrapezoid);
}

/**
 * Returns the points.
 *
 * @return Reference to the vector of points.
 */
const std::vector<cg3::Point2d>& TrapezoidalMap::getPoints() const {
    return points;
}

/**
 * Returns the indexed segments.
 *
 * @return Reference to the vector of indexed segments.
 */
const std::vector<TrapezoidalMap::IndexedSegment2d>& TrapezoidalMap::getIndexedSegments() const {
    return indexedSegments;
}

/**
 * Returns the trapezoids.
 *
 * @return Reference to the vector of trapezoids.
 */
const std::vector<Trapezoid>& TrapezoidalMap::getTrapezoids() const {
    return trapezoids;
}

/**
 * Returns the associated DAG.
 *
 * @return DAG's reference.
 */
const DAG& TrapezoidalMap::getDag() const {
    return dag;
}

/**
 * Returns the point associated with a certain index.
 *
 * @param[in] index of the point.
 * @return The point with the specified index.
 */
const cg3::Point2d& TrapezoidalMap::getPoint(size_t index) const {
    return points[index];
}

/**
 * Returns the segment associated with the specified indexedSegment.
 *
 * @param[in] indexed segment.
 * @return Segment associated with the specified indexed segment.
 */
cg3::Segment2d TrapezoidalMap::getSegment(const IndexedSegment2d& indexedSegment) const {
    const cg3::Point2d& leftEndPoint = getPoint(indexedSegment.first);
    const cg3::Point2d& rightEndPoint = getPoint(indexedSegment.second);

    return cg3::Segment2d(leftEndPoint, rightEndPoint);
}

/**
 * Returns the segment associated with the specified index.
 *
 * @param[in] indexed segment.
 * @return Segment associated with the specified indexed.
 */
cg3::Segment2d TrapezoidalMap::getSegment(size_t index) const {
    return getSegment(indexedSegments[index]);
}

/**
 * Returns the indexed segment associated with the specified index.
 *
 * @param[in] index of the indexed segment.
 * @return Reference to the indexed segment.
 */
const TrapezoidalMap::IndexedSegment2d& TrapezoidalMap::getIndexedSegment(size_t indexSegment) const {
    return indexedSegments[indexSegment];
}

/**
 * Returns the Bounding Box.
 *
 * @return boundingBox.
 */
const cg3::BoundingBox2& TrapezoidalMap::getBoundingBox() const {
    return boundingBox;
}

/**
 * Performs the point location query.
 *
 * @param[in] query point.
 * @return Index to the found trapezoid.
 */
size_t TrapezoidalMap::pointLocationQuery(const cg3::Point2d& queryPoint) const {
    return dag.query(queryPoint);
}

/**
 * Returns the index of the point if present, max otherwise.
 *
 * @param[in] point reference.
 * @param[out] found boolean.
 * @return Index of the point.
 */
size_t TrapezoidalMap::findPoint(const cg3::Point2d &point, bool &found) {
    std::unordered_map<cg3::Point2d, size_t>::iterator it = pointMap.find(point);

    //Point already in the data structure
    if (it != pointMap.end()) {
        found = true;
        return it->second;
    }
    //Point not in the data structure
    else {
        found = false;
        return std::numeric_limits<size_t>::max();
    }
}

/**
 * Returns the index of the indexed segment if present, max otherwise.
 *
 * @param[in] indexedSegment reference.
 * @param[out] found boolean.
 * @return Index of the indexed segment.
 */
size_t TrapezoidalMap::findIndexedSegment(const IndexedSegment2d& indexedSegment, bool& found) {
    std::unordered_map<IndexedSegment2d, size_t>::iterator it = segmentMap.find(indexedSegment);

    //Segment already in the data structure
    if (it != segmentMap.end()) {
        found = true;
        return it->second;
    }
    //Segment not in the data structure
    else {
        found = false;
        return std::numeric_limits<size_t>::max();
    }
}

/**
 * Adds a new point (if not already present).
 *
 * @param[in] point reference.
 * @return Index of the point.
 */
size_t TrapezoidalMap::addPoint(const cg3::Point2d& point) {
    bool found;
    size_t id = findPoint(point, found);

    bool generalPosition = true;
    if (xCoordSet.find(point.x()) != xCoordSet.end()) {
        generalPosition = false;
    }

    // Point will be inserted
    if (!found && generalPosition) {
        id = points.size();

        //Add point
        points.push_back(point);

        pointMap.insert(std::make_pair(point, id));
        xCoordSet.insert(point.x());
    }

    // Update bounding box
    boundingBox.setMax(cg3::Point2d(
            std::max(point.x(), boundingBox.max().x()),
            std::max(point.y(), boundingBox.max().y())));
    boundingBox.setMin(cg3::Point2d(
            std::min(point.x(), boundingBox.min().x()),
            std::min(point.y(), boundingBox.min().y())));

    return id;
}

/**
 * Adds a new indexedSegment and updates the Trapezoidal Map accordingly.
 *
 * @param[in] segment reference.
 */
void TrapezoidalMap::addSegment(const cg3::Segment2d& segment) {
    size_t segmentId;

    // Ordered segment
    cg3::Segment2d orderedSegment = segment;
    if (segment.p2() < segment.p1()) {
        orderedSegment.setP1(segment.p2());
        orderedSegment.setP2(segment.p1());
    }

    // Adding the points
    size_t indexP1 = addPoint(orderedSegment.p1());
    size_t indexP2 = addPoint(orderedSegment.p2());

    bool found;
    IndexedSegment2d indexedSegment(indexP1, indexP2);
    segmentId = findIndexedSegment(indexedSegment, found);

    if (!found) {
        // Adding the indexed segment
        indexedSegments.push_back(indexedSegment);
        segmentId = indexedSegments.size() - 1;
        segmentMap.insert(std::make_pair(indexedSegment, segmentId));

        std::list<size_t> trapezoidsIntersectedByS; // List of trapezoids intersected by the new segment
        followSegment(orderedSegment, trapezoidsIntersectedByS); // Follow Segment operation
        split(trapezoidsIntersectedByS, segmentId); // Creates the new trapezoids based on the inserctions of the new segment

        trapezoidsIntersectedByS.clear();
    }
}

/**
 * Updates the Trapezoidal Map (along with the associated DAG) based on the new inserted segment.
 *
 * @param[in] trapezoids intersected by the new segment (list of trapezoid indices) and the index of the segment itself.
 */
void TrapezoidalMap::split(std::list<size_t>& trapezoidsIntersectedByS, const size_t& s) {
    const size_t& max = std::numeric_limits<size_t>::max();
    const IndexedSegment2d& indexedSegment = getIndexedSegment(s);
    std::vector<size_t> newTrapezoidIndices;
    std::array<size_t, 4> adjacencies;
    std::vector<size_t> leaves;
    std::list<size_t> nodesToDelete;

    // First and Last trapezoids intersected by the new segment
    std::pair<Trapezoid, Trapezoid> firstAndLast;
    firstAndLast.first = trapezoids[trapezoidsIntersectedByS.front()];

    if (trapezoidsIntersectedByS.size() > 1)
        firstAndLast.second = trapezoids[trapezoidsIntersectedByS.back()];

    for (size_t i : trapezoidsIntersectedByS)
        nodesToDelete.push_back(trapezoids[i].getLeaf()); // Storing the nodes which have to be deleted

    if (trapezoidsIntersectedByS.size() == 1) { // It means that I am inserting a segment completely inside one trapezoid

        Trapezoid trapezoid = trapezoids[trapezoidsIntersectedByS.front()]; // Current trapezoid (with the previous adjacencies)

        if (indexedSegment.first != trapezoid.getLeftp() && indexedSegment.second != trapezoid.getRightp()) { // 4 new Trapezoids

            newTrapezoidIndices.push_back(trapezoidsIntersectedByS.front());
            for (size_t i = trapezoids.size(); i < (trapezoids.size() + 3); ++i)
                newTrapezoidIndices.push_back(i); // Generating the indices for the new trapezoids

            for (size_t i = dag.getNodes().size(); i < (dag.getNodes().size() + 4); ++i)
                leaves.push_back(i); // Generating the leaves' indices for the new trapezoids

            // Creating the new trapezoids
            adjacencies = {newTrapezoidIndices[2], newTrapezoidIndices[3], trapezoid.getLowerLeftNeighbor(), trapezoid.getUpperLeftNeighbor()};
            trapezoids[trapezoidsIntersectedByS.front()] = ( Trapezoid( trapezoid.getTop(), trapezoid.getBottom(), trapezoid.getLeftp(), indexedSegment.first, adjacencies, leaves[0] ) ); // Left trapezoid

            adjacencies = {trapezoid.getUpperRightNeighbor(), trapezoid.getLowerRightNeighbor(), newTrapezoidIndices[3], newTrapezoidIndices[2]};
            trapezoids.push_back( Trapezoid( trapezoid.getTop(), trapezoid.getBottom(), indexedSegment.second, trapezoid.getRightp(), adjacencies, leaves[1] ) ); // Right trapezoid

            adjacencies = {newTrapezoidIndices[1], max, max, newTrapezoidIndices[0]};
            trapezoids.push_back( Trapezoid( trapezoid.getTop(), s, indexedSegment.first, indexedSegment.second, adjacencies, leaves[2] ) ); // Top trapezoid

            adjacencies = {max, newTrapezoidIndices[1], newTrapezoidIndices[0], max};
            trapezoids.push_back( Trapezoid( s, trapezoid.getBottom(), indexedSegment.first, indexedSegment.second, adjacencies, leaves[3] ) ); // Bottom trapezoid

            // Adjacencies' update
            for (size_t i = 0, j = trapezoid.getAdjacencies().size()-1; i < trapezoid.getAdjacencies().size(); ++i, --j) {
                if (trapezoid.getAdjacencies()[i] != max && i < 2)
                    trapezoids[trapezoid.getAdjacencies()[i]].setAdjacency(j, newTrapezoidIndices[1]);
                else if (trapezoid.getAdjacencies()[i] != max && i >= 2)
                    trapezoids[trapezoid.getAdjacencies()[i]].setAdjacency(j, newTrapezoidIndices[0]) ;
            }

            dag.update(s, newTrapezoidIndices, nodesToDelete.front(), firstAndLast.first); // DAG's update

        } else if (indexedSegment.first == trapezoid.getLeftp() && indexedSegment.second == trapezoid.getRightp()) { // 2 new Trapezoids

            newTrapezoidIndices.push_back(trapezoidsIntersectedByS.front());
            for (size_t i = trapezoids.size(); i < (trapezoids.size() + 1); ++i)
                newTrapezoidIndices.push_back(i); // Generating the indices for the new trapezoids

            for (size_t i = dag.getNodes().size(); i < (dag.getNodes().size() + 2); ++i)
                leaves.push_back(i); // Generating the leaves' indices for the new trapezoids

            // Creating the new trapezoids
            adjacencies = {trapezoid.getUpperRightNeighbor(), max, max, trapezoid.getUpperLeftNeighbor()};
            trapezoids[trapezoidsIntersectedByS.front()] = ( Trapezoid( trapezoid.getTop(), s, indexedSegment.first, indexedSegment.second, adjacencies, leaves[0] ) ); // Top trapezoid

            adjacencies = {max, trapezoid.getLowerRightNeighbor(), trapezoid.getLowerLeftNeighbor(), max};
            trapezoids.push_back( Trapezoid( s, trapezoid.getBottom(), indexedSegment.first, indexedSegment.second, adjacencies, leaves[1] ) ); // Bottom trapezoid

            // Adjacencies' update
            if (trapezoid.getUpperRightNeighbor() != max)
                trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(newTrapezoidIndices[0]);

            if (trapezoid.getUpperLeftNeighbor() != max)
                trapezoids[trapezoid.getUpperLeftNeighbor()].setUpperRightNeighbor(newTrapezoidIndices[0]);

            if (trapezoid.getLowerRightNeighbor() != max)
                trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(newTrapezoidIndices[1]);

            if (trapezoid.getLowerLeftNeighbor() != max)
                trapezoids[trapezoid.getLowerLeftNeighbor()].setLowerRightNeighbor(newTrapezoidIndices[1]);

            dag.update(s, newTrapezoidIndices, nodesToDelete.front(), firstAndLast.first); // DAG's update

        } else if (indexedSegment.first == trapezoid.getLeftp() && indexedSegment.second != trapezoid.getRightp()) { // 3 new Trapezoids

            newTrapezoidIndices.push_back(trapezoidsIntersectedByS.front());
            for (size_t i = trapezoids.size(); i < (trapezoids.size() + 2); ++i)
                newTrapezoidIndices.push_back(i); // Generating the indices for the new trapezoids

            for (size_t i = dag.getNodes().size(); i < (dag.getNodes().size() + 3); ++i)
                leaves.push_back(i); // Generating the leaves' indices for the new trapezoids

            // Creating the new trapezoids
            adjacencies = {trapezoid.getUpperRightNeighbor(), trapezoid.getLowerRightNeighbor(), newTrapezoidIndices[2], newTrapezoidIndices[1]};
            trapezoids[trapezoidsIntersectedByS.front()] = ( Trapezoid( trapezoid.getTop(), trapezoid.getBottom(), indexedSegment.second, trapezoid.getRightp(), adjacencies, leaves[0] ) ); // Right trapezoid

            adjacencies = {newTrapezoidIndices[0], max, max, trapezoid.getUpperLeftNeighbor()};
            trapezoids.push_back( Trapezoid( trapezoid.getTop(), s, indexedSegment.first, indexedSegment.second, adjacencies, leaves[1] ) ); // Top trapezoid

            adjacencies = {max, newTrapezoidIndices[0], trapezoid.getLowerLeftNeighbor(), max};
            trapezoids.push_back( Trapezoid( s, trapezoid.getBottom(), indexedSegment.first, indexedSegment.second, adjacencies, leaves[2] ) ); // Bottom trapezoid

            // Adjacencies' update
            if (trapezoid.getUpperRightNeighbor() != max)
                trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(newTrapezoidIndices[0]);

            if (trapezoid.getUpperLeftNeighbor() != max)
                trapezoids[trapezoid.getUpperLeftNeighbor()].setUpperRightNeighbor(newTrapezoidIndices[1]);

            if (trapezoid.getLowerRightNeighbor() != max)
                trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(newTrapezoidIndices[0]);

            if (trapezoid.getLowerLeftNeighbor() != max)
                trapezoids[trapezoid.getLowerLeftNeighbor()].setLowerRightNeighbor(newTrapezoidIndices[2]);

            dag.update(s, newTrapezoidIndices, nodesToDelete.front(), firstAndLast.first); // DAG's update

        } else { // 3 new Trapezoids

            newTrapezoidIndices.push_back(trapezoidsIntersectedByS.front());
            for (size_t i = trapezoids.size(); i < (trapezoids.size() + 2); ++i)
                newTrapezoidIndices.push_back(i); // Generating the indices for the new trapezoids

            for (size_t i = dag.getNodes().size(); i < (dag.getNodes().size() + 3); ++i)
                leaves.push_back(i); // Generating the leaves' indices for the new trapezoids

            // Creating the new trapezoids
            adjacencies = {newTrapezoidIndices[1], newTrapezoidIndices[2], trapezoid.getLowerLeftNeighbor(), trapezoid.getUpperLeftNeighbor()};
            trapezoids[trapezoidsIntersectedByS.front()] = ( Trapezoid( trapezoid.getTop(), trapezoid.getBottom(), trapezoid.getLeftp(), indexedSegment.first, adjacencies, leaves[0] ) ); // Right trapezoid

            adjacencies = {trapezoid.getUpperRightNeighbor(), max, max, newTrapezoidIndices[0]};
            trapezoids.push_back( Trapezoid( trapezoid.getTop(), s, indexedSegment.first, indexedSegment.second, adjacencies, leaves[1] ) ); // Top trapezoid

            adjacencies = {max, trapezoid.getLowerRightNeighbor(), newTrapezoidIndices[0],  max};
            trapezoids.push_back( Trapezoid( s, trapezoid.getBottom(), indexedSegment.first, indexedSegment.second, adjacencies, leaves[2] ) ); // Bottom trapezoid

            // Adjacencies' update
            if (trapezoid.getUpperRightNeighbor() != max)
                trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(newTrapezoidIndices[1]);

            if (trapezoid.getUpperLeftNeighbor() != max)
                trapezoids[trapezoid.getUpperLeftNeighbor()].setUpperRightNeighbor(newTrapezoidIndices[0]);

            if (trapezoid.getLowerRightNeighbor() != max)
                trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(newTrapezoidIndices[2]);

            if (trapezoid.getLowerLeftNeighbor() != max)
                trapezoids[trapezoid.getLowerLeftNeighbor()].setLowerRightNeighbor(newTrapezoidIndices[0]);

            dag.update(s, newTrapezoidIndices, nodesToDelete.front(), firstAndLast.first); // DAG's update
        }
    } else { // In this case the segment is crossing more trapezoids

        Trapezoid trapezoid = firstAndLast.first;
        cg3::Point2d leftp = getPoint(trapezoid.getLeftp());
        cg3::Point2d rightp = getPoint(trapezoid.getRightp());
        cg3::Segment2d segment = getSegment(indexedSegment);
        std::pair<size_t, size_t> previousTrapezoids = {max, max}; // Previous trapezoids for each step of the iteration
        std::vector<std::pair<size_t, size_t>> indices; // Vector of pairs <trapezoidIndex, leafIndex> (needed for DAG's update)
        indices.reserve(3);

        std::list<size_t>::iterator currentNodeToDelete = nodesToDelete.begin();
        std::list<size_t>::iterator currentTrapezoidIndex;
        for (currentTrapezoidIndex = trapezoidsIntersectedByS.begin(); currentTrapezoidIndex != trapezoidsIntersectedByS.end(); ++currentTrapezoidIndex) {

            trapezoid = trapezoids[*currentTrapezoidIndex];
            if (trapezoid.getLeftp() != max)
                leftp = getPoint(trapezoid.getLeftp());

            if (trapezoid.getRightp() != max)
                rightp = getPoint(trapezoid.getRightp());

            if (currentTrapezoidIndex == trapezoidsIntersectedByS.begin()) { // First trapezoid

                newTrapezoidIndices.push_back(*currentTrapezoidIndex);
                for (size_t i = trapezoids.size(); i < (trapezoids.size() + 2); ++i)
                    newTrapezoidIndices.push_back(i);

                for (size_t i = dag.getNodes().size(); i < (dag.getNodes().size() + 3); ++i)
                    leaves.push_back(i);

                adjacencies = {newTrapezoidIndices[1], newTrapezoidIndices[2], trapezoid.getLowerLeftNeighbor(), trapezoid.getUpperLeftNeighbor()};
                trapezoids[trapezoidsIntersectedByS.front()] = ( Trapezoid( trapezoid.getTop(), trapezoid.getBottom(), trapezoid.getLeftp(), indexedSegment.first, adjacencies, leaves[0] ) ); // Left trapezoid

                if (liesAbove(rightp, segment)) { // The rightp of the current trapezoid is above the inserted segment

                    adjacencies = {trapezoid.getUpperRightNeighbor(), trapezoid.getLowerRightNeighbor(), max, newTrapezoidIndices[0]};
                    trapezoids.push_back( Trapezoid( trapezoid.getTop(), s, indexedSegment.first, trapezoid.getRightp(), adjacencies, leaves[1] ) ); // Top trapezoid
                    previousTrapezoids.first = trapezoids.size() - 1;

                    adjacencies = {max, trapezoid.getLowerRightNeighbor(), newTrapezoidIndices[0],  max};
                    trapezoids.push_back( Trapezoid( s, trapezoid.getBottom(), indexedSegment.first, max, adjacencies, leaves[2] ) ); // Bottom trapezoid
                    previousTrapezoids.second = trapezoids.size() - 1;

                    // Adjacencies' update
                    trapezoids[*(std::next(currentTrapezoidIndex))].setLowerLeftNeighbor(newTrapezoidIndices[2]);

                    if (trapezoid.getUpperRightNeighbor() != max)
                        trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(newTrapezoidIndices[1]);

                } else { // The rightp of the current trapezoid is below the inserted segment

                    adjacencies = {trapezoid.getUpperRightNeighbor(), max, max, newTrapezoidIndices[0]};
                    trapezoids.push_back( Trapezoid( trapezoid.getTop(), s, indexedSegment.first, max, adjacencies, leaves[1] ) ); // Top trapezoid
                    previousTrapezoids.first = trapezoids.size() - 1;

                    adjacencies = {trapezoid.getUpperRightNeighbor(), trapezoid.getLowerRightNeighbor(), newTrapezoidIndices[0],  max};
                    trapezoids.push_back( Trapezoid( s, trapezoid.getBottom(), indexedSegment.first, trapezoid.getRightp(), adjacencies, leaves[2] ) ); // Bottom trapezoid
                    previousTrapezoids.second = trapezoids.size() - 1;

                    // Adjacencies' update
                    trapezoids[*(std::next(currentTrapezoidIndex))].setUpperLeftNeighbor(newTrapezoidIndices[1]);

                    if (trapezoid.getLowerRightNeighbor() != max)
                        trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(newTrapezoidIndices[2]);

                }

                // Adjacencies' update
                if (trapezoid.getUpperLeftNeighbor() != max)
                    trapezoids[trapezoid.getUpperLeftNeighbor()].setUpperRightNeighbor(newTrapezoidIndices[0]);

                if (trapezoid.getLowerLeftNeighbor() != max)
                    trapezoids[trapezoid.getLowerLeftNeighbor()].setLowerRightNeighbor(newTrapezoidIndices[0]);

                if (indexedSegment.first == firstAndLast.first.getLeftp()) { // First endpoint shared

                    trapezoids[previousTrapezoids.first].setLeaf(leaves[0]);
                    trapezoids[previousTrapezoids.first].setUpperRightNeighbor(trapezoid.getUpperRightNeighbor());
                    trapezoids[previousTrapezoids.first].setUpperLeftNeighbor(trapezoid.getUpperLeftNeighbor());
                    trapezoids[previousTrapezoids.first].setLeftp(indexedSegment.first);
                    trapezoids[newTrapezoidIndices[0]] = trapezoids[previousTrapezoids.first];
                    previousTrapezoids.first = newTrapezoidIndices[0];

                    trapezoids[previousTrapezoids.second].setLeaf(leaves[1]);
                    trapezoids[previousTrapezoids.second].setLowerRightNeighbor(trapezoid.getLowerRightNeighbor());
                    trapezoids[previousTrapezoids.second].setLowerLeftNeighbor(trapezoid.getLowerLeftNeighbor());
                    trapezoids[previousTrapezoids.second].setLeftp(indexedSegment.first);
                    trapezoids[newTrapezoidIndices[1]] = trapezoids[previousTrapezoids.second];
                    previousTrapezoids.second = newTrapezoidIndices[1];

                    // Ajacencies' update
                    if (trapezoid.getUpperRightNeighbor() != max)
                        trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(previousTrapezoids.first);

                    if (trapezoid.getLowerRightNeighbor() != max)
                        trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(previousTrapezoids.second);

                    if (trapezoid.getLowerLeftNeighbor() != max)
                        trapezoids[trapezoid.getLowerLeftNeighbor()].setLowerRightNeighbor(previousTrapezoids.second);

                    if (trapezoid.getUpperLeftNeighbor() != max)
                        trapezoids[trapezoid.getUpperLeftNeighbor()].setUpperRightNeighbor(previousTrapezoids.first);

                    newTrapezoidIndices.pop_back();
                    leaves.pop_back();
                    trapezoids.pop_back();

                    indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.first, trapezoids[previousTrapezoids.first].getLeaf()) );
                    indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.second, trapezoids[previousTrapezoids.second].getLeaf()) );
                    dag.update(*currentNodeToDelete, s, indices, false); // DAG's update

                } else { // Normal case (without shared points)

                    indices.push_back( std::pair<size_t, size_t> (newTrapezoidIndices[newTrapezoidIndices.size()-3], trapezoids[newTrapezoidIndices[newTrapezoidIndices.size()-3]].getLeaf()));
                    indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.first, trapezoids[previousTrapezoids.first].getLeaf()) );
                    indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.second, trapezoids[previousTrapezoids.second].getLeaf()) );
                    dag.update(*currentNodeToDelete, s, indices, true); // DAG's update

                }

            } else if (currentTrapezoidIndex == (std::prev(trapezoidsIntersectedByS.end()))) { // Last trapezoid

                newTrapezoidIndices.push_back(*currentTrapezoidIndex);
                for (size_t i = trapezoids.size(); i < (trapezoids.size() + 1); ++i)
                    newTrapezoidIndices.push_back(i);

                for (size_t i = dag.getNodes().size(); i < (dag.getNodes().size() + 2); ++i)
                    leaves.push_back(i);

                trapezoids[*currentTrapezoidIndex].setLeftp(indexedSegment.second);
                trapezoids[*currentTrapezoidIndex].setLeaf(leaves.back());

                if (trapezoids[previousTrapezoids.first].getRightp() == max) { // previousTrapezoids.first needs to be merged

                    trapezoids[*currentTrapezoidIndex].setAdjacencies({trapezoid.getUpperRightNeighbor(), trapezoid.getLowerRightNeighbor(), newTrapezoidIndices[newTrapezoidIndices.size()-1], previousTrapezoids.first});

                    // Trapezoid to merge
                    trapezoids[previousTrapezoids.first].setRightp(indexedSegment.second);
                    trapezoids[previousTrapezoids.first].setUpperRightNeighbor(*currentTrapezoidIndex);
                    trapezoids[previousTrapezoids.first].setLowerRightNeighbor(max);

                    trapezoids[previousTrapezoids.second].setUpperRightNeighbor(newTrapezoidIndices[newTrapezoidIndices.size()-1]);

                    adjacencies = {max, *currentTrapezoidIndex, trapezoid.getLowerLeftNeighbor(), previousTrapezoids.second};
                    trapezoids.push_back(Trapezoid(s, trapezoid.getBottom(), trapezoid.getLeftp(), indexedSegment.second, adjacencies, leaves[leaves.size()-2]));
                    previousTrapezoids.second = *currentTrapezoidIndex;

                    // Adjacencies' update
                    if (trapezoid.getLowerLeftNeighbor() != max)
                        trapezoids[trapezoid.getLowerLeftNeighbor()].setLowerRightNeighbor(newTrapezoidIndices[newTrapezoidIndices.size()-1]);

                    if (trapezoid.getLowerRightNeighbor() != max)
                        trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(newTrapezoidIndices[newTrapezoidIndices.size()-2]);

                    if (trapezoid.getUpperRightNeighbor() != max)
                        trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(newTrapezoidIndices[newTrapezoidIndices.size()-2]);

                    if (indexedSegment.second == firstAndLast.second.getRightp()) { // Second endpoint shared

                        trapezoids[previousTrapezoids.second].setAdjacencies(trapezoids.back().getAdjacencies());
                        trapezoids[previousTrapezoids.second].setLowerRightNeighbor(trapezoid.getLowerRightNeighbor());
                        trapezoids[previousTrapezoids.second].setLeaf(trapezoids.back().getLeaf());
                        trapezoids[previousTrapezoids.second].setTop(s);
                        trapezoids[previousTrapezoids.second].setBottom(trapezoid.getBottom());
                        trapezoids[previousTrapezoids.second].setLeftp(trapezoid.getLeftp());
                        trapezoids[previousTrapezoids.second].setRightp(indexedSegment.second);

                        // Adjacencies' update
                        if (trapezoids[previousTrapezoids.second].getLowerLeftNeighbor() != max)
                            trapezoids[trapezoids[previousTrapezoids.second].getLowerLeftNeighbor()].setLowerRightNeighbor(previousTrapezoids.second);

                        if (trapezoids[previousTrapezoids.second].getUpperLeftNeighbor() != max)
                            trapezoids[trapezoids[previousTrapezoids.second].getUpperLeftNeighbor()].setUpperRightNeighbor(previousTrapezoids.second);

                        trapezoids[previousTrapezoids.first].setUpperRightNeighbor(trapezoid.getUpperRightNeighbor());

                        if (trapezoid.getUpperRightNeighbor() != max)
                            trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(previousTrapezoids.first);

                        if (trapezoid.getLowerRightNeighbor() != max)
                            trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(previousTrapezoids.second);

                        leaves.pop_back();
                        newTrapezoidIndices.pop_back();
                        trapezoids.pop_back();

                        indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.first, trapezoids[previousTrapezoids.first].getLeaf()) );
                        indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.second, trapezoids[previousTrapezoids.second].getLeaf()) );

                    } else { // Normal case (no shared points)

                        indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.first, trapezoids[previousTrapezoids.first].getLeaf()) );
                        indices.push_back( std::pair<size_t, size_t> (newTrapezoidIndices.back(), trapezoids[newTrapezoidIndices.back()].getLeaf()));
                        indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.second, trapezoids[previousTrapezoids.second].getLeaf()) );

                    }
                    dag.update(*currentNodeToDelete, s, indices, false); // DAG's update

                } else { // previousTrapezoids.second needs to be merged

                    trapezoids[*currentTrapezoidIndex].setAdjacencies({trapezoid.getUpperRightNeighbor(), trapezoid.getLowerRightNeighbor(), previousTrapezoids.second, newTrapezoidIndices[newTrapezoidIndices.size()-1]});

                    // Trapezoid to merge
                    trapezoids[previousTrapezoids.second].setRightp(indexedSegment.second);
                    trapezoids[previousTrapezoids.second].setUpperRightNeighbor(max);
                    trapezoids[previousTrapezoids.second].setLowerRightNeighbor(*currentTrapezoidIndex);

                    trapezoids[previousTrapezoids.first].setLowerRightNeighbor(newTrapezoidIndices.back());

                    adjacencies = {*currentTrapezoidIndex, max, previousTrapezoids.first, trapezoid.getUpperLeftNeighbor()};
                    trapezoids.push_back(Trapezoid(trapezoid.getTop(), s, trapezoid.getLeftp(), indexedSegment.second, adjacencies, leaves[leaves.size()-2]));
                    previousTrapezoids.first = *currentTrapezoidIndex;

                    // Adjacencies' update
                    if (trapezoid.getUpperLeftNeighbor() != max)
                        trapezoids[trapezoid.getUpperLeftNeighbor()].setUpperRightNeighbor(newTrapezoidIndices[newTrapezoidIndices.size()-1]);

                    if (trapezoid.getLowerRightNeighbor() != max)
                        trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(newTrapezoidIndices[newTrapezoidIndices.size()-2]);

                    if (trapezoid.getUpperRightNeighbor() != max)
                        trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(newTrapezoidIndices[newTrapezoidIndices.size()-2]);

                    if (indexedSegment.second == firstAndLast.second.getRightp()) { // Shared right endpoint

                        trapezoids[previousTrapezoids.first].setAdjacencies(trapezoids.back().getAdjacencies());
                        trapezoids[previousTrapezoids.first].setUpperRightNeighbor(trapezoid.getUpperRightNeighbor());
                        trapezoids[previousTrapezoids.first].setLeaf(trapezoids.back().getLeaf());
                        trapezoids[previousTrapezoids.first].setBottom(s);
                        trapezoids[previousTrapezoids.first].setTop(trapezoid.getTop());
                        trapezoids[previousTrapezoids.first].setLeftp(trapezoid.getLeftp());
                        trapezoids[previousTrapezoids.first].setRightp(indexedSegment.second);

                        // Adjacencies' update
                        if (trapezoids[previousTrapezoids.first].getLowerLeftNeighbor() != max)
                            trapezoids[trapezoids[previousTrapezoids.first].getLowerLeftNeighbor()].setLowerRightNeighbor(previousTrapezoids.first);

                        if (trapezoids[previousTrapezoids.first].getUpperLeftNeighbor() != max)
                            trapezoids[trapezoids[previousTrapezoids.first].getUpperLeftNeighbor()].setUpperRightNeighbor(previousTrapezoids.first);

                        trapezoids[previousTrapezoids.second].setLowerRightNeighbor(trapezoid.getLowerRightNeighbor());

                        if (trapezoid.getUpperRightNeighbor() != max)
                            trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(previousTrapezoids.first);

                        if (trapezoid.getLowerRightNeighbor() != max)
                            trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(previousTrapezoids.second);

                        leaves.pop_back();
                        newTrapezoidIndices.pop_back();
                        trapezoids.pop_back();

                        indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.first, trapezoids[previousTrapezoids.first].getLeaf()) );
                        indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.second, trapezoids[previousTrapezoids.second].getLeaf()) );

                    } else { // Normal case (no shared points)

                        indices.push_back( std::pair<size_t, size_t> (newTrapezoidIndices.back(), trapezoids[newTrapezoidIndices.back()].getLeaf()));
                        indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.second, trapezoids[previousTrapezoids.second].getLeaf()) );
                        indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.first, trapezoids[previousTrapezoids.first].getLeaf()) );

                    }
                    dag.update(*currentNodeToDelete, s, indices, false); // DAG's update

                }

            } else { // General case

                if (liesAbove(rightp, segment)) { // The rightp of the current trapezoid lies above the inserted segment

                    if (trapezoids[previousTrapezoids.first].getRightp() == max) { // previousTrapezoid.first to merge

                        // Trapezoid to merge
                        trapezoids[previousTrapezoids.first].setRightp(trapezoid.getRightp());
                        trapezoids[previousTrapezoids.first].setBottom(s);
                        trapezoids[previousTrapezoids.first].setUpperRightNeighbor(trapezoid.getUpperRightNeighbor());
                        trapezoids[previousTrapezoids.first].setLowerRightNeighbor(trapezoid.getLowerRightNeighbor());

                        // Adjacencies' update
                        if (trapezoid.getUpperRightNeighbor() != max)
                            trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(previousTrapezoids.first);

                        if (trapezoid.getLowerRightNeighbor() != max)
                            trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(previousTrapezoids.first);

                        trapezoids[previousTrapezoids.second].setUpperRightNeighbor(*currentTrapezoidIndex);

                        // The current one "goes below" and will be the next to merge
                        newTrapezoidIndices.push_back(*currentTrapezoidIndex);
                        leaves.push_back(dag.getNodes().size());
                        trapezoids[*currentTrapezoidIndex].setLeaf(leaves.back());
                        trapezoids[*currentTrapezoidIndex].setTop(s);
                        trapezoids[*currentTrapezoidIndex].setRightp(max);
                        trapezoids[*currentTrapezoidIndex].setUpperLeftNeighbor(previousTrapezoids.second);
                        previousTrapezoids.second = *currentTrapezoidIndex;

                    } else { // previousTrapezoid.second needs to be merge next time, as well (so it is not necessary to update it right now)

                        trapezoids[previousTrapezoids.first].setLowerRightNeighbor(*currentTrapezoidIndex);

                        // The current trapezoid "goes above"
                        newTrapezoidIndices.push_back(*currentTrapezoidIndex);
                        leaves.push_back(dag.getNodes().size());
                        trapezoids[*currentTrapezoidIndex].setLeaf(leaves.back());
                        trapezoids[*currentTrapezoidIndex].setBottom(s);
                        trapezoids[*currentTrapezoidIndex].setLowerRightNeighbor(*(std::next(currentTrapezoidIndex)));
                        trapezoids[*currentTrapezoidIndex].setLowerLeftNeighbor(previousTrapezoids.first);
                        previousTrapezoids.first = *currentTrapezoidIndex;

                    }

                } else { // The rightp of the current trapezoid lies below the segment s

                    if (trapezoids[previousTrapezoids.first].getRightp() == max) { // previousTrapezoid.first needs to be merge next time, as well (so it is not necessary to update it right now)

                        trapezoids[previousTrapezoids.second].setUpperRightNeighbor(*currentTrapezoidIndex);

                        newTrapezoidIndices.push_back(*currentTrapezoidIndex);
                        leaves.push_back(dag.getNodes().size());
                        trapezoids[*currentTrapezoidIndex].setLeaf(leaves.back());
                        trapezoids[*currentTrapezoidIndex].setTop(s);
                        trapezoids[*currentTrapezoidIndex].setUpperRightNeighbor(max);
                        trapezoids[*currentTrapezoidIndex].setUpperLeftNeighbor(previousTrapezoids.second);
                        previousTrapezoids.second = *currentTrapezoidIndex;

                    } else { // Merge of previousTrapezoids.second (the bottom one)

                        // Trapezoid to merge
                        trapezoids[previousTrapezoids.second].setRightp(trapezoid.getRightp());
                        trapezoids[previousTrapezoids.second].setUpperRightNeighbor(trapezoid.getUpperRightNeighbor());
                        trapezoids[previousTrapezoids.second].setLowerRightNeighbor(trapezoid.getLowerRightNeighbor());

                        // Adjacencies' update
                        if (trapezoid.getUpperRightNeighbor() != max)
                            trapezoids[trapezoid.getUpperRightNeighbor()].setUpperLeftNeighbor(previousTrapezoids.second);

                        if (trapezoid.getLowerRightNeighbor() != max)
                            trapezoids[trapezoid.getLowerRightNeighbor()].setLowerLeftNeighbor(previousTrapezoids.second);

                        trapezoids[previousTrapezoids.first].setLowerRightNeighbor(*currentTrapezoidIndex);

                        // The current trapezoid "goes above" and it will be next one to merge
                        newTrapezoidIndices.push_back(*currentTrapezoidIndex);
                        leaves.push_back(dag.getNodes().size());
                        trapezoids[*currentTrapezoidIndex].setLeaf(leaves.back());
                        trapezoids[*currentTrapezoidIndex].setBottom(s);
                        trapezoids[*currentTrapezoidIndex].setRightp(max);
                        trapezoids[*currentTrapezoidIndex].setLowerLeftNeighbor(previousTrapezoids.first);
                        previousTrapezoids.first = *currentTrapezoidIndex;

                    }

                }

                indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.first, trapezoids[previousTrapezoids.first].getLeaf()) );
                indices.push_back( std::pair<size_t, size_t> (previousTrapezoids.second, trapezoids[previousTrapezoids.second].getLeaf()) );
                dag.update(*currentNodeToDelete, s, indices, false);

            }

            ++currentNodeToDelete;
            indices.clear();
        }

    }

    leaves.clear();
    nodesToDelete.clear();
    newTrapezoidIndices.clear();
}

/**
 * Performes the follow segment operation.
 *
 * @param[in] reference to the segment.
 * @param[out] list of trapezoids (the ones intersected by the segment).
 */
void TrapezoidalMap::followSegment(const cg3::Segment2d& segment, std::list<size_t>& trapezoidsIntersectedByS) const {
    const size_t& max = std::numeric_limits<size_t>::max();
    const cg3::Point2d& rightEndPoint = segment.p2();
    size_t trapezoidIndex = (dag).find(segment); // Retrieving the trapezoid which cointains the left endpoint of the segment

    trapezoidsIntersectedByS.push_back(trapezoidIndex); // It will be the first trapezoid "intersected"

    while(trapezoidIndex != max && rightEndPoint.x() > points[trapezoids[trapezoidIndex].getRightp()].x()) {

        if (liesAbove(points[trapezoids[trapezoidIndex].getRightp()], segment)) {

            if (trapezoids[trapezoidIndex].getLowerRightNeighbor() != max)
                trapezoidIndex = trapezoids[trapezoidIndex].getLowerRightNeighbor(); // Lower right neighbor
            else
                trapezoidIndex = max;

        } else {

            if (trapezoids[trapezoidIndex].getUpperRightNeighbor() != max)
                trapezoidIndex = trapezoids[trapezoidIndex].getUpperRightNeighbor(); // Upper right neighbor
            else
                trapezoidIndex = max;

        }

        trapezoidsIntersectedByS.push_back(trapezoidIndex);

    }
}

/**
 * Deletes the items of the Trapezoidal Map and performs the insertion of the first trapezoid (the one associated with the Bounding Box).
 */
void TrapezoidalMap::clear() {
    // Saving the points that belongs to the Bounding Box
    cg3::Point2d p0 = points[0];
    cg3::Point2d p1 = points[1];
    cg3::Point2d p2 = points[2];
    cg3::Point2d p3 = points[3];

    // Clear operation
    dag.clear();
    trapezoids.clear();
    points.clear();
    indexedSegments.clear();
    pointMap.clear();
    segmentMap.clear();
    xCoordSet.clear();
    boundingBox.setMin(cg3::Point2d(0,0));
    boundingBox.setMax(cg3::Point2d(0,0));

    // Insertion of the first trapezoid associated with the Bounding Box (this because the TM always must have the first trapezoid)
    size_t max = std::numeric_limits<size_t>::max();
    dag.init(0);

    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);

    Trapezoid firstTrapezoid(max, max, 0,  2, 0);
    firstTrapezoid.setAdjacencies(max, max, max, max);
    trapezoids.push_back(firstTrapezoid);
}
