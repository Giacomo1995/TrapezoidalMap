#ifndef TRAPEZOIDALMAP_H
#define TRAPEZOIDALMAP_H

#include <cg3/geometry/point2.h>
#include <cg3/geometry/segment2.h>
#include <cg3/geometry/bounding_box2.h>
#include "trapezoidalmap_dataset.h"
#include "directed_acyclic_graph.h"
#include "trapezoid.h"

/**
 * @brief This class represents a trapezoidal map.
 */
class TrapezoidalMap {

public:
    typedef std::pair<size_t, size_t> IndexedSegment2d;

    TrapezoidalMap(const cg3::Point2d& boundingBoxMin, const cg3::Point2d& boundingBoxMax);

    const std::vector<cg3::Point2d>& getPoints() const;
    const std::vector<IndexedSegment2d>& getIndexedSegments() const;
    const std::vector<Trapezoid>& getTrapezoids() const;
    const DAG& getDag() const;
    const cg3::Point2d& getPoint(size_t index) const;
    cg3::Segment2d getSegment(const IndexedSegment2d& indexedSegment) const;
    cg3::Segment2d getSegment(size_t index) const;
    const IndexedSegment2d& getIndexedSegment(size_t indexSegment) const;
    const cg3::BoundingBox2& getBoundingBox() const;
    size_t pointLocationQuery(const cg3::Point2d& queryPoint) const;

    size_t findPoint(const cg3::Point2d& point, bool& found);
    size_t findIndexedSegment(const IndexedSegment2d& indexedSegment, bool& found);

    size_t addPoint(const cg3::Point2d& point);
    void addSegment(const cg3::Segment2d& segment);

    void clear();

private:
    void split(std::list<size_t>& trapezoidsIntersectedByS, const size_t& s);
    void followSegment(const cg3::Segment2d& segment, std::list<size_t>& trapezoidsIntersectedByS) const;

    std::vector<cg3::Point2d> points; // Vector of all points inside the Bounding Box
    std::vector<IndexedSegment2d> indexedSegments; // Vector of indexed segments
    std::vector<Trapezoid> trapezoids; // Faces of the Trapezoidal Map

    std::unordered_map<cg3::Point2d, size_t> pointMap;
    std::unordered_map<IndexedSegment2d, size_t> segmentMap;

    std::unordered_set<double> xCoordSet;

    cg3::BoundingBox2 boundingBox;

    DAG dag; // Associated DAG
};


#endif // TRAPEZOIDALMAP_H
