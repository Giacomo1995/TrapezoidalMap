#ifndef DRAWABLE_TRAPEZOIDALMAP_H
#define DRAWABLE_TRAPEZOIDALMAP_H

#include "data_structures/trapezoidalmap.h"

#include <cg3/viewer/interfaces/drawable_object.h>

#include <cg3/utilities/color.h>

#include <cg3/viewer/opengl_objects/opengl_objects2.h>

#include "algorithms/intersection.h"

/**
 * @brief Class to draw the trapezoids of the trapezoidal map.
 */
class DrawableTrapezoidalMap : public TrapezoidalMap, public cg3::DrawableObject {

public:
    DrawableTrapezoidalMap(const cg3::Point2d& boundingBoxMin, const cg3::Point2d& boundingBoxMax);

    void draw() const;
    cg3::Point3d sceneCenter() const;
    double sceneRadius() const;

    unsigned int getTrapezoidSize();
    void setTrapezoidSize(unsigned int trapezoidSize);

    const std::array<double, 3>& getColor(const size_t trapezoidIndex) const;
    void setColor(const size_t trapezoidIndex, const std::array<double, 3>& color);
    void setRandomColor(const size_t trapezoidIndex);

    void clear();


    // INLINE METHODS

    /**
     * Generates the color of the new trapezoids.
     */
    inline void updateColor() {

        double R, G, B;

        if (trapezoidsColor.size() == 1)
            trapezoidsColor.pop_back();

        size_t len = getTrapezoids().size() - trapezoidsColor.size();

        for (size_t i = 0; i < len; ++i) {
            R = (((double) std::rand()) / RAND_MAX) * .65 + .1;
            G = (((double) std::rand()) / RAND_MAX) * .65 + .1;
            B = (((double) std::rand()) / RAND_MAX) * .65 + .1;

            trapezoidsColor.push_back({R, G, B});
        }

    }

    /**
     * Draws the trapezoid identified by the specified vertices.
     *
     * @param[in] vertices of the trapezoid p0, p1, p2, p3 and colorIndex to specify the color.
     */
    inline void drawTrapezoid(const cg3::Point2d& p0, const cg3::Point2d& p1, const cg3::Point2d& p2, const cg3::Point2d& p3, const size_t& colorIndex) const {
        const std::array<double, 3>& color = trapezoidsColor[colorIndex];
        double R = color[0], G = color[1], B = color[2];

        glBegin(GL_POLYGON);

        glColor3f(R, G, B);

        glVertex2f(p0.x(), p0.y());

        glVertex2f(p1.x(), p1.y());

        glVertex2f(p2.x(), p2.y());

        glVertex2f(p3.x(), p3.y());

        glEnd();
    }

private:
    std::vector<std::array<double, 3>> trapezoidsColor;
    unsigned int trapezoidSize;

    cg3::Point2d boundingBoxMin;
    cg3::Point2d boundingBoxMax;

};

#endif // DRAWABLE_TRAPEZOIDALMAP_H
