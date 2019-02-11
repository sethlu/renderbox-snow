#ifndef SNOW_NODE_H
#define SNOW_NODE_H


#include "Vector.h"
#include "Matrix.h"

#include "logging.h"


class Node {

    friend class SnowSolver;

public:

    Node(glm::vec3 const &position) : position(position) {}

    glm::vec3 position;

    glm::vec3 const &velocity(unsigned int n) const {
        if ((n & 0x1) == 0) {
            return velocity0;
        }
        return velocity1;
    }

protected:

    float mass;
    glm::vec3 velocity0 = glm::vec3();
    glm::vec3 velocity1 = glm::vec3();

    glm::vec3 &velocity(unsigned int n) {
        if ((n & 0x1) == 0) {
            return velocity0;
        }
        return velocity1;
    }

    // Used for updates
    glm::vec3 velocity_star;

};


#endif //SNOW_NODE_H
