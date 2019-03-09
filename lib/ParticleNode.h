#ifndef SNOW_PARTICLENODE_H
#define SNOW_PARTICLENODE_H


#include "Node.h"


class ParticleNode : public Node {

    friend class SnowSolver;

public:

    ParticleNode(glm::vec3 const &position, float mass) : Node(position) {
        this->mass = mass;
    }

protected:

    float volume0;
    glm::mat3 deformElastic = glm::mat3(1);
    glm::mat3 deformPlastic = glm::mat3(1);

    // Used for updates
    float tempWeight;

};


#endif //SNOW_PARTICLENODE_H
