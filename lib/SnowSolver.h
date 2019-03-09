#ifndef SNOW_SNOWSOLVER_H
#define SNOW_SNOWSOLVER_H


#include <vector>

#include "ParticleNode.h"
#include "GridNode.h"


class SnowSolver {
public:

    SnowSolver(float h, glm::uvec3 const &size);

    std::vector<ParticleNode> particleNodes;

    void update(float delta_t, unsigned int n);

    GridNode &gridNode(unsigned int x, unsigned int y, unsigned int z) {
        return gridNodes[(x * size.y + y) * size.z + z];
    }

    GridNode &gridNode(glm::uvec3 location) {
        return gridNode(location.x, location.y, location.z);
    }

    bool isValidGridNode(unsigned int x, unsigned int y, unsigned int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < size.x && y < size.y && z < size.z;
    }

private:

    float h;
    float invh;
    glm::uvec3 size;
    std::vector<GridNode> gridNodes;

    float youngsModulus0 = 140000;
    float poissonsRatio = 0.2f;
    float lambda0 = youngsModulus0 * poissonsRatio / ((1 + poissonsRatio) * (1 - 2 * poissonsRatio));
    float mu0 = youngsModulus0 / (2 * (1 + poissonsRatio));
    float hardeningCoefficient = 10;

    float criticalCompression = 0.025;
    float criticalStretch = 0.0075;

    float alpha = 0.95; // PIC/FLIP

    void handleNodeCollisionVelocityUpdate(Node &node);

    float n(float x) {
        auto absx = fabs(x);
        if (absx < 1) {
            auto x2 = x * x;
            auto absx3 = x2 * absx;
            return 0.5f * absx3 - x2 + 2.f / 3;
        } else if (absx < 2) {
            auto x2 = x * x;
            auto absx3 = x2 * absx;
            return -1.f / 6 * absx3 + x2 - 2 * absx + 4.f / 3;
        }
        return 0;
    }

    float del_n(float x) {
        auto absx = fabs(x);
        if (absx < 1) {
            auto x2 = x * x;
            return 3.f / 2 * x2 - 2 * absx;
        } else if (absx < 2) {
            auto x2 = x * x;
            return -1.f / 2 * x2 + 2 * absx - 2;
        }
        return 0;
    }

    float n(glm::vec3 const &gridPosition, glm::vec3 const &particlePosition) {
        return n(invh * (particlePosition.x - gridPosition.x)) *
               n(invh * (particlePosition.y - gridPosition.y)) *
               n(invh * (particlePosition.z - gridPosition.z));
    }

    float n(unsigned i, glm::vec3 const &particlePosition) {
        auto const &gpos = gridNodes[i].position;
        return n(gpos, particlePosition);
    }

    float n(unsigned i, unsigned p) {
        auto const &ppos = particleNodes[p].position;
        return n(i, ppos);
    }

    glm::vec3 nabla_n(glm::vec3 const &gridPosition, glm::vec3 const &particlePosition) {
        auto nx = n(invh * (particlePosition.x - gridPosition.x));
        auto ny = n(invh * (particlePosition.y - gridPosition.y));
        auto nz = n(invh * (particlePosition.z - gridPosition.z));
        auto dnx = del_n(invh * (particlePosition.x - gridPosition.x));
        auto dny = del_n(invh * (particlePosition.y - gridPosition.y));
        auto dnz = del_n(invh * (particlePosition.z - gridPosition.z));

        return {dnx * ny * nz, nx * dny * nz, nx * ny * dnz};
    }

    float weight(GridNode const &i, ParticleNode const &p) {
        return n(i.position, p.position);
    }

    float weight(unsigned i, unsigned p) {
        return n(i, p);
    }

    glm::vec3 nabla_weight(GridNode const &i, ParticleNode const &p) {
        return nabla_n(i.position, p.position);
    }

    glm::vec3 nabla_weight(unsigned i, unsigned p) {
        auto const &gpos = gridNodes[i].position;
        auto const &ppos = particleNodes[p].position;
        return nabla_n(gpos, ppos);
    }

};


#endif //SNOW_SNOWSOLVER_H
