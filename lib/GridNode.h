#ifndef SNOW_GRIDNODE_H
#define SNOW_GRIDNODE_H


#include "Node.h"


class GridNode : public Node {

    friend class SnowSolver;

public:

    GridNode(glm::vec3 const &position, glm::uvec3 const &location) : Node(position), location(location) {}

protected:

    float density0;
    glm::uvec3 location;
    glm::vec3 force;

};


#endif //SNOW_GRIDNODE_H
