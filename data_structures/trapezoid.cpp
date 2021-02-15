#include "trapezoid.h"

/**
 * Empty contructor.
 */
Trapezoid::Trapezoid() {

}

/**
 * Constructor. It takes the values of all attributes.
 *
 * @param[in] top, bottom, leftp, rightp, adjacencies, leaf.
 */
Trapezoid::Trapezoid(size_t top, size_t bottom, size_t leftp, size_t rightp, std::array<size_t, NUM_ADJACENCIES> adjacencies, size_t leaf) {
    this->top  = top;
    this->bottom = bottom;
    this->leftp = leftp;
    this->rightp = rightp;
    this->adjacencies = adjacencies;
    this->leaf = leaf;
}

/**
 * Constructor. It takes the values of all attributes except for the adjacencies.
 *
 * @param[in] top, bottom, leftp, rightp, leaf.
 */
Trapezoid::Trapezoid(size_t top, size_t bottom, size_t leftp, size_t rightp, size_t leaf) {
    this->top  = top;
    this->bottom = bottom;
    this->leftp = leftp;
    this->rightp = rightp;
    this->leaf = leaf;
}

/**
 * Returns the top value.
 *
 * @return top.
 */
size_t Trapezoid::getTop() const {
    return top;
}

/**
 * Sets the top value.
 *
 * @param[in] top.
 */
void Trapezoid::setTop(const size_t value) {
    top = value;
}

/**
 * Returns the bottom value.
 *
 * @return bottom.
 */
size_t Trapezoid::getBottom() const {
    return bottom;
}

/**
 * Sets the bottom value.
 *
 * @param[in] bottom.
 */
void Trapezoid::setBottom(const size_t value) {
    bottom = value;
}

/**
 * Returns the leftp value.
 *
 * @return leftp.
 */
size_t Trapezoid::getLeftp() const {
    return leftp;
}

/**
 * Sets the leftp value.
 *
 * @param[in] leftp.
 */
void Trapezoid::setLeftp(const size_t value) {
    leftp = value;
}

/**
 * Returns the rightp value.
 *
 * @return rightp.
 */
size_t Trapezoid::getRightp() const {
    return rightp;
}

/**
 * Sets the rightp value.
 *
 * @param[in] rightp.
 */
void Trapezoid::setRightp(const size_t value) {
    rightp = value;
}

/**
 * Returns the adjacencies.
 *
 * @return adjacencies.
 */
const std::array<size_t, NUM_ADJACENCIES>& Trapezoid::getAdjacencies() const {
    return adjacencies;
}

/**
 * Sets the adjacencies' value.
 *
 * @param[in] array of adjacencies.
 */
void Trapezoid::setAdjacencies(const std::array<size_t, NUM_ADJACENCIES>& value) {
    adjacencies = value;
}

/**
 * Sets the adjacencies' value.
 *
 * @param[in] first, second, third and fourth adjacency.
 */
void Trapezoid::setAdjacencies(size_t first, size_t second, size_t third, size_t fourth) {
    adjacencies[0] = first;
    adjacencies[1] = second;
    adjacencies[2] = third;
    adjacencies[3] = fourth;
}

/**
 * Returns the leaf value.
 *
 * @return leaf.
 */
size_t Trapezoid::getLeaf() const {
    return leaf;
}

/**
 * Sets the leaf value.
 *
 * @param[in] leaf.
 */
void Trapezoid::setLeaf(size_t value) {
    leaf = value;
}

/**
 * Sets the adjacency with the specified index.
 *
 * @param[in] index of the adjancency and its value.
 */
void Trapezoid::setAdjacency(size_t index, size_t value) {
    adjacencies[index] = value;
}

/**
 * Returns the upper right neighbor.
 *
 * @return index to the upper right neighbor.
 */
size_t Trapezoid::getUpperRightNeighbor() const {
    return adjacencies[0];
}

/**
 * Sets the upper right neighbor.
 *
 * @param[in] upperRightNeighbor.
 */
void Trapezoid::setUpperRightNeighbor(size_t upperRightNeighbor) {
    adjacencies[0] = upperRightNeighbor;
}

/**
 * Returns the lower right neighbor.
 *
 * @return index to the lower right neighbor.
 */
size_t Trapezoid::getLowerRightNeighbor() const {
    return adjacencies[1];
}

/**
 * Sets the lower right neighbor.
 *
 * @param[in] lowerRightNeighbor.
 */
void Trapezoid::setLowerRightNeighbor(size_t lowerRightNeighbor) {
    adjacencies[1] = lowerRightNeighbor;
}

/**
 * Returns the lower left neighbor.
 *
 * @return index to the lower left neighbor.
 */
size_t Trapezoid::getLowerLeftNeighbor() const {
    return adjacencies[2];
}

/**
 * Sets the lower left neighbor.
 *
 * @param[in] lowerLeftNeighbor.
 */
void Trapezoid::setLowerLeftNeighbor(size_t lowerLeftNeighbor) {
    adjacencies[2] = lowerLeftNeighbor;
}

/**
 * Returns the upper left neighbor.
 *
 * @return index to the upper left neighbor.
 */
size_t Trapezoid::getUpperLeftNeighbor() const {
    return adjacencies[3];
}

/**
 * Sets the upper left neighbor.
 *
 * @param[in] upperLeftNeighbor.
 */
void Trapezoid::setUpperLeftNeighbor(size_t upperLeftNeighbor) {
    adjacencies[3] = upperLeftNeighbor;
}
