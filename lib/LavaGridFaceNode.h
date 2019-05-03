#ifndef SNOW_LAVAGRIDFACENODE_H
#define SNOW_LAVAGRIDFACENODE_H


#include "Node.h"


class LavaGridFaceNode : Node {

    friend class LavaSolver;

public:

    LavaGridFaceNode(glm::dvec3 const &position, glm::uvec3 const &location) : Node(position), location(location) {}

protected:

    glm::uvec3 location;
    glm::dvec3 force;
    double thermalConductivity;

    bool colliding;

};


#endif //SNOW_LAVAGRIDFACENODE_H
