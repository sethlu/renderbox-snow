#ifndef SNOW_GRIDNODE_H
#define SNOW_GRIDNODE_H


#include "Node.h"


class SnowGridNode : public Node {

    friend class SnowSolver;

public:

    SnowGridNode(glm::dvec3 const &position, glm::uvec3 const &location) : Node(position), location(location) {}

protected:

    double density0;
    glm::uvec3 location;
    glm::dvec3 force;

};


#endif //SNOW_GRIDNODE_H
