#ifndef TRAPEZOID_H
#define TRAPEZOID_H

#include <cstdlib>
#include <array>

#define NUM_ADJACENCIES 4

/**
 * @brief This class describes an indexed trapezoid for a trapezoidal map.
 * Each trapezoid has the following indices: top, bottom, leftp rightp, four adjacencies and the corresponding leaf.
 */
class Trapezoid {

public:
    Trapezoid();
    Trapezoid(size_t top, size_t bottom, size_t leftp, size_t rightp, std::array<size_t, NUM_ADJACENCIES> adjacencies, size_t leaf);
    Trapezoid(size_t top, size_t bottom, size_t leftp, size_t rightp, size_t leaf);

    size_t getTop() const;
    void setTop(const size_t value);

    size_t getBottom() const;
    void setBottom(const size_t value);

    size_t getLeftp() const;
    void setLeftp(const size_t value);

    size_t getRightp() const;
    void setRightp(const size_t value);

    const std::array<size_t, NUM_ADJACENCIES>& getAdjacencies() const;
    void setAdjacencies(const std::array<size_t, NUM_ADJACENCIES>& value);
    void setAdjacencies(size_t first, size_t second, size_t third, size_t fourth);

    size_t getLeaf() const;
    void setLeaf(size_t value);

    void setAdjacency(size_t index, size_t value);

    size_t getUpperRightNeighbor() const;
    void setUpperRightNeighbor(size_t upperRightNeighbor);

    size_t getLowerRightNeighbor() const;
    void setLowerRightNeighbor(size_t lowerRightNeighbor);

    size_t getLowerLeftNeighbor() const;
    void setLowerLeftNeighbor(size_t lowerLeftNeighbor);

    size_t getUpperLeftNeighbor() const;
    void setUpperLeftNeighbor(size_t upperLeftNeighbor);

private:
    size_t top; // Index to the top segment
    size_t bottom; // Index to the bottom segment
    size_t leftp; // Index to the left point
    size_t rightp; // Index to the right point
    std::array<size_t, NUM_ADJACENCIES> adjacencies;
    size_t leaf; // Pointer to the corresponding leaf in the DAG
};

#endif // TRAPEZOID_H
