#ifndef SNOW_LAVASOLVER_H
#define SNOW_LAVASOLVER_H


#include <vector>

#include "LavaParticleNode.h"
#include "LavaGridCellNode.h"
#include "LavaGridFaceNode.h"
#include "Solver.h"


class LavaSolver : public Solver {
public:

    struct LAVA_SOLVER_STATE_HEADER {
        unsigned short type; // LA
        unsigned int headerSize;
        float h;
        glm::uvec3 size;
        unsigned int tick;
        float delta_t;
        float alpha;
        size_t numParticles;
    };

    struct LAVA_SOLVER_STATE_PARTICLE {
        glm::dvec3 position;
        glm::dvec3 velocity;
        float mass;
        float temperature;
        float criticalCompression;
        float criticalStretch;
        float hardeningCoefficient;
        float youngsModulus0;
        float poissonsRatio;
        float thermalConductivity;
        float specificHeat;
        float fusionTemperature;
        float latentHeatOfFusion;
        float latentHeat;
        float volume0;
        glm::dmat3 deformElastic;
        glm::dmat3 deformPlastic;
    };

    LavaSolver(double h, glm::uvec3 const &size);

    explicit LavaSolver(std::string const &filename);

    std::vector<LavaParticleNode> particleNodes;

    void propagateSimulationParametersUpdate();

    void update();

    void saveState(std::string const &filename);

    void loadState(std::string const &filename);

    bool (*isNodeColliding)(Node &node);

    void (*handleNodeCollisionVelocityUpdate)(Node &node);

    unsigned int getTick() {
        return tick;
    }

    double getTime() {
        return tick * delta_t;
    }

    unsigned int getGridCellNodeIndex(unsigned int x, unsigned int y, unsigned int z) {
        return (x * size.y + y) * size.z + z;
    }

    unsigned int getGridFaceXNodeIndex(unsigned int x, unsigned int y, unsigned int z) {
        return (x * size.y + y) * size.z + z;
    }

    unsigned int getGridFaceYNodeIndex(unsigned int x, unsigned int y, unsigned int z) {
        return (x * (size.y + 1) + y) * size.z + z;
    }

    unsigned int getGridFaceZNodeIndex(unsigned int x, unsigned int y, unsigned int z) {
        return (x * size.y + y) * (size.z + 1) + z;
    }

    LavaGridCellNode &gridCellNode(unsigned int x, unsigned int y, unsigned int z) {
        return gridCellNodes[getGridCellNodeIndex(x, y, z)];
    }

    LavaGridFaceNode &gridFaceXNode(unsigned int x, unsigned int y, unsigned int z) {
        return gridFaceXNodes[getGridFaceXNodeIndex(x, y, z)];
    }

    LavaGridFaceNode &gridFaceYNode(unsigned int x, unsigned int y, unsigned int z) {
        return gridFaceYNodes[getGridFaceYNodeIndex(x, y, z)];
    }

    LavaGridFaceNode &gridFaceZNode(unsigned int x, unsigned int y, unsigned int z) {
        return gridFaceZNodes[getGridFaceZNodeIndex(x, y, z)];
    }

    LavaGridFaceNode &gridFaceXNode(glm::uvec3 location) {
        return gridFaceXNodes[getGridFaceXNodeIndex(location.x, location.y, location.z)];
    }

    LavaGridFaceNode &gridFaceYNode(glm::uvec3 location) {
        return gridFaceYNodes[getGridFaceYNodeIndex(location.x, location.y, location.z)];
    }

    LavaGridFaceNode &gridFaceZNode(glm::uvec3 location) {
        return gridFaceZNodes[getGridFaceZNodeIndex(location.x, location.y, location.z)];
    }

    bool isValidGridCellNode(unsigned int x, unsigned int y, unsigned int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < size.x && y < size.y && z < size.z;
    }

    bool isValidGridFaceXNode(unsigned int x, unsigned int y, unsigned int z) {
        return x >= 0 && y >= 0 && z >= 0 && x <= size.x && y < size.y && z < size.z;
    }

    bool isValidGridFaceYNode(unsigned int x, unsigned int y, unsigned int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < size.x && y <= size.y && z < size.z;
    }

    bool isValidGridFaceZNode(unsigned int x, unsigned int y, unsigned int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < size.x && y < size.y && z <= size.z;
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

    static void applyTemperatureDifference(LavaParticleNode &node, double temperatureDifference) {
        // Latent heat of fusion for phase change
        double latentEnergyOfFusion = node.mass * node.latentHeatOfFusion; // [J]

        if (node.temperature - FLT_EPSILON > node.fusionTemperature) {
            // Fluid

            auto newTemperature = node.temperature + temperatureDifference;
            if (newTemperature - FLT_EPSILON > node.fusionTemperature) {
                // Remain as fluid
                node.temperature = newTemperature;
            } else {
                // Freezing

                // Energy due to temperature change after reaching freezing point
                double joules = node.specificHeat * node.mass * (node.fusionTemperature - newTemperature);
                if (joules >= latentEnergyOfFusion) {
                    // Allow phase change
                    node.temperature = newTemperature +
                                       latentEnergyOfFusion / node.mass /
                                       node.specificHeat; // Compensate for phase change
                } else {
                    node.latentHeat = node.latentHeatOfFusion - joules;
                    node.temperature = node.fusionTemperature;
                }
            }
        } else if (node.temperature + FLT_EPSILON < node.fusionTemperature) {
            // Solid

            auto newTemperature = node.temperature + temperatureDifference;
            if (newTemperature + FLT_EPSILON < node.fusionTemperature) {
                // Remain as solid
                node.temperature = newTemperature;
            } else {
                // Melting

                // Energy due to temperature change before after reaching melting point
                double joules = node.specificHeat * node.mass * (newTemperature - node.fusionTemperature);
                if (joules >= latentEnergyOfFusion) {
                    // Allow phase change
                    node.temperature = newTemperature -
                                       latentEnergyOfFusion / node.mass /
                                       node.specificHeat; // Compensate for phase change
                } else {
                    node.latentHeat = joules;
                    node.temperature = node.fusionTemperature;
                }
            }
        } else {
            // Melting/freezing

            auto newLatentEnergy = node.latentHeat + node.specificHeat * node.mass * temperatureDifference;
            if (newLatentEnergy > latentEnergyOfFusion) {
                // Melted

                node.temperature += (newLatentEnergy - latentEnergyOfFusion) / node.mass /
                                    node.specificHeat; // Compensate for phase change
            } else if (newLatentEnergy < 0) {
                // Frozen

                node.temperature += newLatentEnergy / node.mass / node.specificHeat; // Compensate for phase change
            } else {
                // Still in phase-change state
                node.latentHeat = newLatentEnergy;
            }
        }
    }

    // Simulation parameters

    double alpha = 0.95; // PIC/FLIP

    // Grid
    double h;
    glm::uvec3 size;

    // Time
    unsigned int tick = 0;
    double delta_t = 5e-3;

    // Record keeping

    bool simulationParametersDidUpdate = true;

private:

    // Dependent values on simulation parameters

    double invh;

    // Cell-centered
    std::vector<LavaGridCellNode> gridCellNodes;
    // Staggered
    std::vector<LavaGridFaceNode> gridFaceXNodes;
    std::vector<LavaGridFaceNode> gridFaceYNodes;
    std::vector<LavaGridFaceNode> gridFaceZNodes;

    // Helper methods

    void implicitHeatIntegrationMatrix(std::vector<double> &Ax, std::vector<double> const &x);

    void implicitPressureIntegrationMatrix(std::vector<double> &Ax, std::vector<double> const &x);

    double n(glm::dvec3 const &gridPosition, glm::dvec3 const &particlePosition) {
        return n(invh * (particlePosition.x - gridPosition.x)) *
               n(invh * (particlePosition.y - gridPosition.y)) *
               n(invh * (particlePosition.z - gridPosition.z));
    }

    double n(unsigned i, glm::dvec3 const &particlePosition) {
        auto const &gpos = gridCellNodes[i].position;
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

    double weight(LavaGridCellNode const &i, LavaParticleNode const &p) {
        return n(i.position, p.position);
    }

    double weight(LavaGridFaceNode const &i, LavaParticleNode const &p) {
        return n(i.position, p.position);
    }

    glm::dvec3 nabla_weight(LavaGridCellNode const &i, LavaParticleNode const &p) {
        return nabla_n(i.position, p.position);
    }

    glm::dvec3 nabla_weight(LavaGridFaceNode const &i, LavaParticleNode const &p) {
        return nabla_n(i.position, p.position);
    }

};


#endif //SNOW_LAVASOLVER_H
