#include "intersection.h"

#include <cg3/geometry/intersections2.h>

/**
 * Computes the insersection between a vertical segment and a non-vertical one.
 *
 * @param[in] a vertical segment (identified just through its x value) and a standard segment.
 * @return The intersection point.
 */
cg3::Point2d intersection(const double& x, const cg3::Segment2d& s2) {

    double m = ((s2.p2().y() - s2.p1().y())) / (s2.p2().x() - s2.p1().x());
    double q = s2.p1().y() - (m * s2.p1().x());

    return cg3::Point2d( x, (m * x) + q );
}
