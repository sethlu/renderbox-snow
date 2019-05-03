#ifndef SNOW_LAVAGRIDCELLNODE_H
#define SNOW_LAVAGRIDCELLNODE_H


#include "Node.h"


enum LavaGridCellNodeType {
    EMPTY,
    COLLIDING,
    INTERIOR
};


class LavaGridCellNode : Node {

    friend class LavaSolver;

public:

    LavaGridCellNode(glm::dvec3 const &position, glm::uvec3 const &location) : Node(position), location(location) {}

protected:

    glm::uvec3 location;
    glm::dvec3 force;
    double j;
    double je;
    double jp;
    double specificHeat; // [J/K/kg]
    double temperature; // [K]
    double inv_lambda; // lambda^(-1) rasterized on grid face

    LavaGridCellNodeType type;

    double temperature_next; // [K]

};


#endif //SNOW_LAVAGRIDCELLNODE_H
