#ifndef SNOW_SNOWSOLVER_H
#define SNOW_SNOWSOLVER_H


#include <vector>

#include "ParticleNode.h"
#include "GridNode.h"


class SnowSolver {
public:

    SnowSolver(double h, glm::uvec3 const &size);

    std::vector<ParticleNode> particleNodes;

    void update();

    void (*handleNodeCollisionVelocityUpdate)(Node &node);

    unsigned int getTick() {
        return tick;
    }

    unsigned int getGridNodeIndex(unsigned int x, unsigned int y, unsigned int z) {
        return (x * size.y + y) * size.z + z;
    }

    GridNode &gridNode(unsigned int x, unsigned int y, unsigned int z) {
        return gridNodes[getGridNodeIndex(x, y, z)];
    }

    GridNode &gridNode(glm::uvec3 location) {
        return gridNode(location.x, location.y, location.z);
    }

    bool isValidGridNode(unsigned int x, unsigned int y, unsigned int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < size.x && y < size.y && z < size.z;
    }

    static double n(double x) {
        auto absx = fabs(x);
        if (absx < 1) {
            auto x2 = x * x;
            auto absx3 = x2 * absx;
            return 0.5 * absx3 - x2 + 2. / 3;
        } else if (absx < 2) {
            auto x2 = x * x;
            auto absx3 = x2 * absx;
            return -1.0 / 6 * absx3 + x2 - 2 * absx + 4.0 / 3;
        }
        return 0;
    }

    static double del_n(double x) {
        auto absx = fabs(x);
        if (absx < 1) {
            auto x2 = x * x;
            return (3.0 / 2 * x2 - 2 * absx) * (x < 0 ? -1 : 1);
        } else if (absx < 2) {
            auto x2 = x * x;
            return (-1.0 / 2 * x2 + 2 * absx - 2) * (x < 0 ? -1 : 1);
        }
        return 0;
    }

private:

    // Simulation parameters

    // Reference
    double youngsModulus0 = 1.4e5;
    double criticalCompression = 2.5e-2;
    double criticalStretch = 7.5e-3;
    double hardeningCoefficient = 10;

    //  Lower hardening
//    double youngsModulus0 = 1.4e5;
//    double criticalCompression = 2.5e-2;
//    double criticalStretch = 7.5e-3;
//    double hardeningCoefficient = 5;

    // Lower critical stretch
//    double youngsModulus0 = 1.4e5;
//    double criticalCompression = 2.5e-2;
//    double criticalStretch = 5e-3;
//    double hardeningCoefficient = 10;

    // Lower critical compression
//    double youngsModulus0 = 1.4e5;
//    double criticalCompression = 1.9e-2;
//    double criticalStretch = 7.5e-3;
//    double hardeningCoefficient = 10;

    double poissonsRatio = 0.2;
    double alpha = 0.95; // PIC/FLIP

    double h;
    glm::uvec3 size;

    unsigned int tick = 0;
    double delta_t = 5e-3;

    double beta = 0; // {explicit = 0, semi-implicit = 1} integration

    // Dependent values on simulation parameters

    double lambda0;
    double mu0;
    double invh;
    std::vector<GridNode> gridNodes;

    // Helper methods

    void simulationParametersDidUpdate() {
        lambda0 = youngsModulus0 * poissonsRatio / ((1 + poissonsRatio) * (1 - 2 * poissonsRatio));
        mu0 = youngsModulus0 / (2 * (1 + poissonsRatio));
        invh = 1 / h;

        gridNodes.clear();
        for (auto x = 0; x < size.x; x++) {
            for (auto y = 0; y < size.y; y++) {
                for (auto z = 0; z < size.z; z++) {
                    gridNodes.emplace_back(glm::dvec3(x, y, z) * h, glm::uvec3(x, y, z));
                }
            }
        }

        LOG(INFO) << "size=" << size << std::endl;
        LOG(INFO) << "#gridNodes=" << gridNodes.size() << std::endl;
    }

    void implicitVelocityIntegrationMatrix(std::vector<glm::dvec3> &Ax, std::vector<glm::dvec3> const &x);

    double n(glm::dvec3 const &gridPosition, glm::dvec3 const &particlePosition) {
        return n(invh * (particlePosition.x - gridPosition.x)) *
               n(invh * (particlePosition.y - gridPosition.y)) *
               n(invh * (particlePosition.z - gridPosition.z));
    }

    double n(unsigned i, glm::dvec3 const &particlePosition) {
        auto const &gpos = gridNodes[i].position;
        return n(gpos, particlePosition);
    }

    double n(unsigned i, unsigned p) {
        auto const &ppos = particleNodes[p].position;
        return n(i, ppos);
    }

    glm::dvec3 nabla_n(glm::dvec3 const &gridPosition, glm::dvec3 const &particlePosition) {
        auto nx = n(invh * (particlePosition.x - gridPosition.x));
        auto ny = n(invh * (particlePosition.y - gridPosition.y));
        auto nz = n(invh * (particlePosition.z - gridPosition.z));
        auto dnx = del_n(invh * (particlePosition.x - gridPosition.x));
        auto dny = del_n(invh * (particlePosition.y - gridPosition.y));
        auto dnz = del_n(invh * (particlePosition.z - gridPosition.z));

        return {dnx * ny * nz, nx * dny * nz, nx * ny * dnz};
    }

    double weight(GridNode const &i, ParticleNode const &p) {
        return n(i.position, p.position);
    }

    double weight(unsigned i, unsigned p) {
        return n(i, p);
    }

    glm::dvec3 nabla_weight(GridNode const &i, ParticleNode const &p) {
        return nabla_n(i.position, p.position);
    }

    glm::dvec3 nabla_weight(unsigned i, unsigned p) {
        auto const &gpos = gridNodes[i].position;
        auto const &ppos = particleNodes[p].position;
        return nabla_n(gpos, ppos);
    }

};


#endif //SNOW_SNOWSOLVER_H
