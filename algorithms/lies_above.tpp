#include "lies_above.h"

/**
 * If the returned value is negative then the point p is above the line segment s, otherwise it is under it.
 *
 * @param[in] A point p and a segment s.
 * @return a boolean value.
 */
inline bool liesAbove(const cg3::Point2d& p, const cg3::Segment2d& s) {
    return ( ( (s.p2().x() - s.p1().x()) * (p.y() - s.p1().y()) ) - ( (s.p2().y() - s.p1().y()) * (p.x() - s.p1().x()) ) ) > 0;
}
