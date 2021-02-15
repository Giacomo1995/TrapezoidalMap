#include "drawable_trapezoidalmap.h"

/**
 * Constructor.
 *
 * @param[in] minimum and maximum vertex of the Bounding Box corresponding to the first trapezoid.
 */
DrawableTrapezoidalMap::DrawableTrapezoidalMap(const cg3::Point2d& boundingBoxMin, const cg3::Point2d& boundingBoxMax) : TrapezoidalMap(boundingBoxMin, boundingBoxMax), trapezoidSize(3) {
    trapezoidsColor.push_back({255, 255, 255});

    this->boundingBoxMin = boundingBoxMin;
    this->boundingBoxMax = boundingBoxMax;
}

/**
 * Draws the trapezoids.
 */
void DrawableTrapezoidalMap::draw() const {
    const size_t& max = std::numeric_limits<size_t>::max();

    size_t colorIndex = 0;
    for (const Trapezoid& t : getTrapezoids()) {

        cg3::Point2d p1(getPoint(t.getLeftp()).x(), boundingBoxMin.y());
        cg3::Point2d p2(getPoint(t.getLeftp()).x(), boundingBoxMax.y());

        cg3::Point2d q1(getPoint(t.getRightp()).x(), boundingBoxMin.y());
        cg3::Point2d q2(getPoint(t.getRightp()).x(), boundingBoxMax.y());

        if (t.getBottom() != max) {
            cg3::Segment2d bottom1(getPoint(getIndexedSegment(t.getBottom()).first), getPoint(getIndexedSegment(t.getBottom()).second));
            p1 = intersection(p1.x(), bottom1);

            cg3::Segment2d bottom2(getPoint(getIndexedSegment(t.getBottom()).first), getPoint(getIndexedSegment(t.getBottom()).second));
            q1 = intersection(q1.x(), bottom2);
        }

        if (t.getTop() != max) {
            cg3::Segment2d top1(getPoint(getIndexedSegment(t.getTop()).first), getPoint(getIndexedSegment(t.getTop()).second));
            p2 = intersection(p2.x(), top1);

            cg3::Segment2d top2(getPoint(getIndexedSegment(t.getTop()).first), getPoint(getIndexedSegment(t.getTop()).second));
            q2 = intersection(q2.x(), top2);
        }

        if (getPoint(t.getLeftp()) != boundingBoxMin) {
            cg3::opengl::drawLine2(p1, p2, cg3::Color(255, 0, 0), trapezoidSize);
        }

        if (getPoint(t.getRightp()) != boundingBoxMax) {
            cg3::opengl::drawLine2(q1, q2, cg3::Color(255, 0, 0), trapezoidSize);
        }

        drawTrapezoid(p1, p2, q2, q1, colorIndex);

        ++colorIndex;
    }
}

/**
 * Centers the scene based on the origin of the Bounding Box.
 *
 * @return Center point.
 */
cg3::Point3d DrawableTrapezoidalMap::sceneCenter() const {
    const cg3::BoundingBox2& boundingBox = this->getBoundingBox();
    return cg3::Point3d(boundingBox.center().x(), boundingBox.center().y(), 0);
}

/**
 * Returns length of the diagonal of the Bounding Box.
 *
 * @return Bounding Box diagonal.
 */
double DrawableTrapezoidalMap::sceneRadius() const {
    const cg3::BoundingBox2& boundingBox = this->getBoundingBox();
    return boundingBox.diag();
}

/**
 * Returns the trapezoid size value.
 *
 * @return trapezoidSize.
 */
unsigned int DrawableTrapezoidalMap::getTrapezoidSize() {
    return this->trapezoidSize;
}

/**
 * Sets the trapezoid size value
 *
 * @param[in] trapezoidSize.
 */
void DrawableTrapezoidalMap::setTrapezoidSize(unsigned int trapezoidSize) {
    this->trapezoidSize = trapezoidSize;
}

/**
 * Get method for the color of a specified trapezoid
 *
 * @param[in] trapezoid index.
 * @return The color of the trapezoid.
 */
const std::array<double, 3>& DrawableTrapezoidalMap::getColor(const size_t trapezoidIndex) const {
    return trapezoidsColor[trapezoidIndex];
}

/**
 * Sets the color of the specified trapezoid.
 *
 * @param[in] trapezoidIndex and color (R, G, B).
 */
void DrawableTrapezoidalMap::setColor(const size_t trapezoidIndex, const std::array<double, 3>& color) {
    if (trapezoidIndex < trapezoidsColor.size())
        trapezoidsColor[trapezoidIndex] = color;
}

/**
 * Sets a random color for the specified trapezoid.
 *
 * @param[in] trapezoidIndex and color (R, G, B).
 */
void DrawableTrapezoidalMap::setRandomColor(const size_t trapezoidIndex) {
    if (trapezoidIndex < trapezoidsColor.size()) {
        double R, G, B;

        R = (((double) std::rand()) / RAND_MAX) * .65 + .1;
        G = (((double) std::rand()) / RAND_MAX) * .65 + .1;
        B = (((double) std::rand()) / RAND_MAX) * .65 + .1;

        trapezoidsColor[trapezoidIndex] = {R, G, B};
    }
}

/**
 * Deletes the items of DrawableTrapezoidalMap and inserts the color of the first trapezoid.
 */
void DrawableTrapezoidalMap::clear() {
    trapezoidsColor.clear();
    TrapezoidalMap::clear();

    trapezoidsColor.push_back({1, 1, 1}); // Bounding Box color
}
