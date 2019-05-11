#ifndef SNOW_LAVAGRIDFACENODE_H
#define SNOW_LAVAGRIDFACENODE_H


#include "Node.h"


struct LavaGridFaceNode : public Node {

    LavaGridFaceNode(glm::dvec3 const &position, glm::uvec3 const &location) : Node(position), location(location) {}

    glm::uvec3 location;

    double force{};

    double thermalConductivity{};

    bool colliding{};

};


#endif //SNOW_LAVAGRIDFACENODE_H
