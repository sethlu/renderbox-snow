#ifndef SNOW_LAVAGRIDCELLNODE_H
#define SNOW_LAVAGRIDCELLNODE_H


#include "Node.h"


enum LavaGridCellNodeType {
    EMPTY,
    COLLIDING,
    INTERIOR
};


struct LavaGridCellNode : public Node {

    LavaGridCellNode(glm::dvec3 const &position, glm::uvec3 const &location) : Node(position), location(location) {}

    glm::uvec3 location;

    glm::dvec3 force{};

    double j{};
    double je{};
    double jp{};

    double specificHeat{}; // [J/K/kg]

    double temperature{}; // [K]
    double temperature_next{}; // [K]

    double inv_lambda{}; // lambda^(-1) rasterized on grid face

    LavaGridCellNodeType type = EMPTY;

};


#endif //SNOW_LAVAGRIDCELLNODE_H
