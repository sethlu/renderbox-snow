#ifndef SNOW_PARTICLENODE_H
#define SNOW_PARTICLENODE_H


#include "Node.h"


struct SnowParticleNode : public Node {

    SnowParticleNode(glm::dvec3 const &position, double mass) : Node(position) {
        this->mass = mass;
    }

    double volume0;

    glm::dmat3 deformElastic = glm::dmat3(1);
    glm::dmat3 deformPlastic = glm::dmat3(1);

    // Memoized weights for each update
    double weight[64];
    glm::dvec3 nabla_weight[64];

};


#endif //SNOW_PARTICLENODE_H
