#include "slope.h"

/**
 * Computes the slope of a given segment.
 *
 * @param[in] A segment.
 * @return double value (slope of the segment).
 */
inline double slope(const cg3::Segment2d& segment) {
    return (segment.p2().y() - segment.p1().y()) / (segment.p2().x() - segment.p1().x());
}
