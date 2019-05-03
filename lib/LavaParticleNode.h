#ifndef SNOW_LAVAPARTICLENODE_H
#define SNOW_LAVAPARTICLENODE_H


#include "Node.h"


class LavaParticleNode : public Node {

    friend class LavaSolver;

public:

    LavaParticleNode(glm::dvec3 const &position, double mass) : Node(position) {
        this->mass = mass;

        // Butter

        criticalCompression = 2.5e-2; // ??
        criticalStretch = 7.5e-3; // ??
        hardeningCoefficient = 10; // ??
        youngsModulus0 = 1.4e6;
        poissonsRatio = 0.2; // ??

        temperature = 20;
        thermalConductivity = 0.2;
        specificHeat = 2.4e3;
        fusionTemperature = 35;
        latentHeatOfFusion = 60e3;

        // Init

        mu0 = youngsModulus0 / (2 * (1 + poissonsRatio));
        lambda0 = youngsModulus0 * poissonsRatio / ((1 + poissonsRatio) * (1 - 2 * poissonsRatio));
    }

    double temperature; // [degC]

    // Material properties

    double criticalCompression;
    double criticalStretch;
    double hardeningCoefficient;
    double youngsModulus0;
    double poissonsRatio;

    double thermalConductivity; // at 25 degC
    double specificHeat; // [J / (kg degC)]
    double fusionTemperature; // [degC]
    double latentHeatOfFusion; // [J / kg]

    // Record keeping

    double latentEnergy = 0; // [J]

protected:

    double mu0;
    double lambda0;
    double volume0;
    glm::dmat3 deformElastic = glm::dmat3(1);
    glm::dmat3 deformPlastic = glm::dmat3(1);

    // Memoized for each update

    // Weights
    double cell_weight[64];
    glm::dvec3 cell_nabla_weight[64];
    double face_x_weight[64];
    glm::dvec3 face_x_nabla_weight[64];
    double face_y_weight[64];
    glm::dvec3 face_y_nabla_weight[64];
    double face_z_weight[64];
    glm::dvec3 face_z_nabla_weight[64];

};


#endif //SNOW_LAVAPARTICLENODE_H
