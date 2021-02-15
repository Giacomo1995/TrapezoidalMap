#ifndef LIES_ABOVE_H
#define LIES_ABOVE_H

#include <cg3/geometry/point2.h>
#include <cg3/geometry/segment2.h>

inline bool liesAbove(const cg3::Point2d& p, const cg3::Segment2d& s);

#include "lies_above.tpp"

#endif // LIES_ABOVE_H
