#ifndef SNOW_GRIDNODE_H
#define SNOW_GRIDNODE_H


#include "Node.h"


struct SnowGridNode : public Node {

    SnowGridNode(glm::dvec3 const &position, glm::uvec3 const &location) : Node(position), location(location) {}

    glm::uvec3 location;

    glm::dvec3 force{};

    double density0{}; // TODO: Use temporary array instead?

};


#endif //SNOW_GRIDNODE_H
