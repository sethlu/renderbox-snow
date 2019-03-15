#ifndef SNOW_PARTICLENODE_H
#define SNOW_PARTICLENODE_H


#include "Node.h"


class ParticleNode : public Node {

    friend class SnowSolver;

public:

    ParticleNode(glm::dvec3 const &position, double mass) : Node(position) {
        this->mass = mass;
    }

protected:

    double volume0;
    glm::dmat3 deformElastic = glm::dmat3(1);
    glm::dmat3 deformPlastic = glm::dmat3(1);

    // Used for updates
    double tempWeight;

};


#endif //SNOW_PARTICLENODE_H
