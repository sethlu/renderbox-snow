#ifndef SNOW_SNOWSOLVER_H
#define SNOW_SNOWSOLVER_H


#include <vector>

#include "ParticleNode.h"
#include "GridNode.h"
#include "Solver.h"


class SnowSolver : public Solver {
public:

    struct SNOW_SOLVER_STATE_HEADER {
        double youngsModulus0; // 8
        double criticalCompression; // 8
        double criticalStretch; // 8
        double hardeningCoefficient; // 8
        double h; // 8
        glm::uvec3 size; // 12
        unsigned int tick; // 4
        double delta_t; // 8
        double alpha; // 8
        double beta; // 8
        size_t numParticles; // 8
    };

    struct SNOW_SOLVER_STATE_PARTICLE {
        glm::dvec3 position; // 24
        glm::dvec3 velocity; // 24
        double mass; // 8
        double volume0; // 8
        glm::dmat3 deformElastic; // 72
        glm::dmat3 deformPlastic; // 72
    };

    SnowSolver(double h, glm::uvec3 const &size);

    explicit SnowSolver(std::string const &filename);

    std::vector<ParticleNode> particleNodes;

    void propagateSimulationParametersUpdate();

    void update();

    void saveState(std::string const &filename);

    void loadState(std::string const &filename);

    void (*handleNodeCollisionVelocityUpdate)(Node &node);

    unsigned int getTick() {
        return tick;
    }

    double getTime() {
        return tick * delta_t;
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
        auto absx = std::abs(x);
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
        auto absx = std::abs(x);
        if (absx < 1) {
            auto x2 = x * x;
            return (3.0 / 2 * x2 - 2 * absx) * (x < 0 ? -1 : 1);
        } else if (absx < 2) {
            auto x2 = x * x;
            return (-1.0 / 2 * x2 + 2 * absx - 2) * (x < 0 ? -1 : 1);
        }
        return 0;
    }

    // Physical parameters
    // NB: The parameters are not expected to change after simulation begins

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

    // Simulation parameters

    double alpha = 0.95; // PIC/FLIP
    double beta = 0; // {explicit = 0, semi-implicit = 1} integration

    // Grid
    double h;
    glm::uvec3 size;

    // Time
    unsigned int tick = 0;
    double delta_t = 5e-3;

    // Record keeping

    bool simulationParametersDidUpdate = true;

private:

    double poissonsRatio = 0.2;

    // Dependent values on simulation parameters

    double lambda0;
    double mu0;
    double invh;
    std::vector<GridNode> gridNodes;

    // Helper methods

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

        return invh * glm::dvec3(dnx * ny * nz, nx * dny * nz, nx * ny * dnz);
    }

    double weight(GridNode const &i, ParticleNode const &p) {
        return n(i.position, p.position);
    }

    glm::dvec3 nabla_weight(GridNode const &i, ParticleNode const &p) {
        return nabla_n(i.position, p.position);
    }

};


#endif //SNOW_SNOWSOLVER_H
