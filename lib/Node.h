#ifndef SNOW_NODE_H
#define SNOW_NODE_H


#include <glm/glm.hpp>

#include "logging.h"


class Node {

    friend class SnowSolver;

public:

    Node(glm::dvec3 const &position) : position(position) {}

    glm::dvec3 position;

    glm::dvec3 const &velocity(unsigned int n) const {
        if ((n & 0x1) == 0) {
            return velocity0;
        }
        return velocity1;
    }

    glm::dvec3 &velocity(unsigned int n) {
        if ((n & 0x1) == 0) {
            return velocity0;
        }
        return velocity1;
    }

    // Used for updates
    glm::dvec3 velocity_star;

protected:

    double mass;
    glm::dvec3 velocity0 = glm::dvec3();
    glm::dvec3 velocity1 = glm::dvec3();

};


#endif //SNOW_NODE_H
