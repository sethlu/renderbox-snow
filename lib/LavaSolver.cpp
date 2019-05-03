#include "LavaSolver.h"

#include "conjugate_residual_solver.h"


LavaSolver::LavaSolver(double h, glm::uvec3 const &size) : h(h), size(size) {

}

void LavaSolver::propagateSimulationParametersUpdate() {
    simulationParametersDidUpdate = false;

    invh = 1 / h;

    gridCellNodes.clear();
    for (auto x = 0; x < size.x; x++) {
        for (auto y = 0; y < size.y; y++) {
            for (auto z = 0; z < size.z; z++) {
                gridCellNodes.emplace_back(glm::dvec3(x, y, z) * h, glm::uvec3(x, y, z));
            }
        }
    }

    gridFaceXNodes.clear();
    for (auto x = 0; x <= size.x; x++) {
        for (auto y = 0; y < size.y; y++) {
            for (auto z = 0; z < size.z; z++) {
                gridFaceXNodes.emplace_back(glm::dvec3(x - 0.5, y, z) * h, glm::uvec3(x, y, z));
            }
        }
    }

    gridFaceYNodes.clear();
    for (auto x = 0; x < size.x; x++) {
        for (auto y = 0; y <= size.y; y++) {
            for (auto z = 0; z < size.z; z++) {
                gridFaceYNodes.emplace_back(glm::dvec3(x, y - 0.5, z) * h, glm::uvec3(x, y, z));
            }
        }
    }

    gridFaceZNodes.clear();
    for (auto x = 0; x < size.x; x++) {
        for (auto y = 0; y < size.y; y++) {
            for (auto z = 0; z <= size.z; z++) {
                gridFaceZNodes.emplace_back(glm::dvec3(x, y, z - 0.5) * h, glm::uvec3(x, y, z));
            }
        }
    }

    LOG(INFO) << "size=" << size << std::endl;
    LOG(INFO) << "#gridCellNodes=" << gridCellNodes.size() << std::endl;
    LOG(INFO) << "#gridFaceXNodes=" << gridFaceXNodes.size() << std::endl;
    LOG(INFO) << "#gridFaceYNodes=" << gridFaceYNodes.size() << std::endl;
    LOG(INFO) << "#gridFaceZNodes=" << gridFaceZNodes.size() << std::endl;
}

void LavaSolver::update() {
    LOG(INFO) << "delta_t=" << delta_t << " tick=" << tick << std::endl;

    if (simulationParametersDidUpdate) {
        propagateSimulationParametersUpdate();
    }

    auto numParticleNodes = particleNodes.size();
    auto numGridCellNodes = gridCellNodes.size();
    auto numGridFaceXNodes = gridFaceXNodes.size();
    auto numGridFaceYNodes = gridFaceYNodes.size();
    auto numGridFaceZNodes = gridFaceZNodes.size();

    // 3. Rasterize particle data to grid //////////////////////////////////////////////////////////////////////////////

    // Clear cell nodes
    for (auto i = 0; i < numGridCellNodes; i++) {
        auto &cellNode = gridCellNodes[i];

        cellNode.mass = 0;
        cellNode.velocity(tick) = {};
        cellNode.j = 0;
        cellNode.je = 0;
        cellNode.specificHeat = 0;
        cellNode.temperature = 0;
        cellNode.inv_lambda = 0;

    }
    // Clear face nodes
    for (auto i = 0; i < numGridFaceXNodes; i++) {
        auto &faceNode = gridFaceXNodes[i];

        faceNode.mass = 0;
        faceNode.velocity(tick) = {};
        faceNode.thermalConductivity = 0;
    }
    for (auto i = 0; i < numGridFaceYNodes; i++) {
        auto &faceNode = gridFaceYNodes[i];

        faceNode.mass = 0;
        faceNode.velocity(tick) = {};
        faceNode.thermalConductivity = 0;
    }
    for (auto i = 0; i < numGridFaceZNodes; i++) {
        auto &faceNode = gridFaceZNodes[i];

        faceNode.mass = 0;
        faceNode.velocity(tick) = {};
        faceNode.thermalConductivity = 0;
    }

    for (auto p = 0; p < numParticleNodes; p++) {
        auto &particleNode = particleNodes[p];
        auto gcmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));
        auto gfxmin = glm::ivec3((particleNode.position / h) - glm::dvec3(0.5, 1, 1));
        auto gfymin = glm::ivec3((particleNode.position / h) - glm::dvec3(1, 0.5, 1));
        auto gfzmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1, 1, 0.5));

        double jp = glm::determinant(particleNode.deformPlastic);
        double je = glm::determinant(particleNode.deformElastic);
        double j = glm::determinant(particleNode.deformElastic * particleNode.deformPlastic);

        auto e = exp(particleNode.hardeningCoefficient * (1 - jp));
        auto mu = particleNode.mu0 * e;
        auto lambda = particleNode.lambda0 * e;
        auto inv_lambda = 1 / lambda;

        // Nearby weighted grid cell nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gcmin.x + i / 16;
            auto gy = gcmin.y + (i / 4) % 4;
            auto gz = gcmin.z + i % 4;
            if (!isValidGridCellNode(gx, gy, gz)) continue;
            auto &cellNode = this->gridCellNode(gx, gy, gz);

            // Pre-compute weights
            particleNode.cell_weight[i] = weight(cellNode, particleNode);
            particleNode.cell_nabla_weight[i] = nabla_weight(cellNode, particleNode);

            auto particleWeightedMass = particleNode.mass * particleNode.cell_weight[i];

            cellNode.mass += particleWeightedMass;
            cellNode.velocity(tick) += particleNode.velocity(tick) * particleWeightedMass;
            cellNode.j += j * particleWeightedMass;
            cellNode.je += je * particleWeightedMass;
            cellNode.specificHeat += particleNode.specificHeat * particleWeightedMass;
            cellNode.temperature += particleNode.temperature * particleWeightedMass;
            cellNode.inv_lambda += inv_lambda * particleWeightedMass;
        }

        // Nearby weighted grid face nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfxmin.x + i / 16;
            auto gy = gfxmin.y + (i / 4) % 4;
            auto gz = gfxmin.z + i % 4;
            if (!isValidGridFaceXNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceXNode(gx, gy, gz);

            // Pre-compute weights
            particleNode.face_x_weight[i] = weight(faceNode, particleNode);
            particleNode.face_x_nabla_weight[i] = nabla_weight(faceNode, particleNode);

            auto particleWeightedMass = particleNode.mass * particleNode.cell_weight[i];

            faceNode.mass += particleWeightedMass;
            faceNode.velocity(tick) += particleNode.velocity(tick) * particleWeightedMass;
            faceNode.thermalConductivity += particleNode.thermalConductivity * particleWeightedMass;
        }
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfymin.x + i / 16;
            auto gy = gfymin.y + (i / 4) % 4;
            auto gz = gfymin.z + i % 4;
            if (!isValidGridFaceYNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceYNode(gx, gy, gz);

            // Pre-compute weights
            particleNode.face_y_weight[i] = weight(faceNode, particleNode);
            particleNode.face_y_nabla_weight[i] = nabla_weight(faceNode, particleNode);

            auto particleWeightedMass = particleNode.mass * particleNode.cell_weight[i];

            faceNode.mass += particleWeightedMass;
            faceNode.velocity(tick) += particleNode.velocity(tick) * particleWeightedMass;
            faceNode.thermalConductivity += particleNode.thermalConductivity * particleWeightedMass;
        }
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfzmin.x + i / 16;
            auto gy = gfzmin.y + (i / 4) % 4;
            auto gz = gfzmin.z + i % 4;
            if (!isValidGridFaceZNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceZNode(gx, gy, gz);

            // Pre-compute weights
            particleNode.face_z_weight[i] = weight(faceNode, particleNode);
            particleNode.face_z_nabla_weight[i] = nabla_weight(faceNode, particleNode);

            auto particleWeightedMass = particleNode.mass * particleNode.cell_weight[i];

            faceNode.mass += particleWeightedMass;
            faceNode.velocity(tick) += particleNode.velocity(tick) * particleWeightedMass;
            faceNode.thermalConductivity += particleNode.thermalConductivity * particleWeightedMass;
        }

    }

    for (auto i = 0; i < numGridCellNodes; i++) {
        auto &cellNode = gridCellNodes[i];

        if (cellNode.mass > 0) {
            cellNode.velocity(tick) /= cellNode.mass;
            cellNode.j /= cellNode.mass;
            cellNode.je /= cellNode.mass;
            cellNode.jp = cellNode.j / cellNode.je;
            cellNode.specificHeat /= cellNode.mass;
            cellNode.temperature /= cellNode.mass;
            cellNode.inv_lambda /= cellNode.mass;
        } else {
            cellNode.velocity(tick) = {};
            cellNode.j = 0;
            cellNode.je = 0;
            cellNode.jp = 0;
            cellNode.specificHeat = 0;
            cellNode.temperature = 20; // FIXME: Hardcoded room temperature
            cellNode.inv_lambda = 0;
        }
    }

    for (auto i = 0; i < numGridFaceXNodes; i++) {
        auto &gridFaceNode = gridFaceXNodes[i];

        if (gridFaceNode.mass > 0) {
            gridFaceNode.velocity(tick) /= gridFaceNode.mass;
            gridFaceNode.thermalConductivity /= gridFaceNode.mass;
        } else {
            gridFaceNode.velocity(tick) = {};
            gridFaceNode.thermalConductivity = 0;
        }

        gridFaceNode.colliding = isNodeColliding(gridFaceNode);
    }
    for (auto i = 0; i < numGridFaceYNodes; i++) {
        auto &gridFaceNode = gridFaceYNodes[i];

        if (gridFaceNode.mass > 0) {
            gridFaceNode.velocity(tick) /= gridFaceNode.mass;
            gridFaceNode.thermalConductivity /= gridFaceNode.mass;
        } else {
            gridFaceNode.velocity(tick) = {};
            gridFaceNode.thermalConductivity = 0;
        }

        gridFaceNode.colliding = isNodeColliding(gridFaceNode);
    }
    for (auto i = 0; i < numGridFaceZNodes; i++) {
        auto &gridFaceNode = gridFaceZNodes[i];

        if (gridFaceNode.mass > 0) {
            gridFaceNode.velocity(tick) /= gridFaceNode.mass;
            gridFaceNode.thermalConductivity /= gridFaceNode.mass;
        } else {
            gridFaceNode.velocity(tick) = {};
            gridFaceNode.thermalConductivity = 0;
        }

        gridFaceNode.colliding = isNodeColliding(gridFaceNode);
    }

    // 4. Classify cells ///////////////////////////////////////////////////////////////////////////////////////////////

    int numGellNodesColliding = 0;

    for (auto i = 0; i < numGridCellNodes; i++) {
        auto &cellNode = gridCellNodes[i];

        auto cellColliding = true;
        auto cellInterior = true;

        {
            auto face = gridFaceXNode(cellNode.location);
            cellColliding &= face.colliding;
            cellInterior &= face.mass > 0;
        }
        {
            auto face = gridFaceXNode(cellNode.location + glm::uvec3(1, 0, 0));
            cellColliding &= face.colliding;
            cellInterior &= face.mass > 0;
        }
        {
            auto face = gridFaceYNode(cellNode.location);
            cellColliding &= face.colliding;
            cellInterior &= face.mass > 0;
        }
        {
            auto face = gridFaceYNode(cellNode.location + glm::uvec3(0, 1, 0));
            cellColliding &= face.colliding;
            cellInterior &= face.mass > 0;
        }
        {
            auto face = gridFaceZNode(cellNode.location);
            cellColliding &= face.colliding;
            cellInterior &= face.mass > 0;
        }
        {
            auto face = gridFaceZNode(cellNode.location + glm::uvec3(0, 0, 1));
            cellColliding &= face.colliding;
            cellInterior &= face.mass > 0;
        }

        if (cellColliding) {
            cellNode.type = COLLIDING;
            cellNode.temperature = 100; // FIXME: Hard coded hot colliding surface
            numGellNodesColliding++;
        } else if (cellInterior) {
            cellNode.type = INTERIOR;
        } else {
            cellNode.type = EMPTY;
        }
    }

    LOG(INFO) << "numCellNodesColliding=" << numGellNodesColliding << std::endl;

    // 8. Solve heat equation //////////////////////////////////////////////////////////////////////////////////////////

    std::vector<double> next_temperature(numGridCellNodes);
    std::vector<double> temperature(numGridCellNodes);

    for (auto c = 0; c < numGridCellNodes; c++) {
        auto &cellNode = gridCellNodes[c];

        temperature[c] = cellNode.temperature;
        next_temperature[c] = cellNode.temperature;

    }

    conjugateResidualSolver(this, &LavaSolver::implicitHeatIntegrationMatrix,
                            next_temperature, temperature, 50, 1);

    for (auto c = 0; c < numGridCellNodes; c++) {
        auto &cellNode = gridCellNodes[c];

        cellNode.temperature_next = next_temperature[c];

    }

    // 9. Update particle state from grid //////////////////////////////////////////////////////////////////////////////

    for (auto p = 0; p < numParticleNodes; p++) {
        auto &particleNode = particleNodes[p];
        auto gmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));

        auto temperature_pic = 0.0;
        auto temperature_flip = particleNode.temperature;

        // Nearby weighted grid nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gmin.x + i / 16;
            auto gy = gmin.y + (i / 4) % 4;
            auto gz = gmin.z + i % 4;
            if (!isValidGridCellNode(gx, gy, gz)) continue;
            auto &cellNode = gridCellNode(gx, gy, gz);

            // FIXME: Use tighter weight function

            auto w = particleNode.cell_weight[i];
            auto gt = cellNode.temperature;
            auto gt1 = cellNode.temperature_next;

            temperature_pic += gt1 * w;
            temperature_flip += (gt1 - gt) * w;

        }

        auto temperature_current = particleNode.temperature;
        auto temperature_next = (1 - alpha) * temperature_pic + alpha * temperature_flip;

        applyTemperatureDifference(particleNode, temperature_next - particleNode.temperature);

    }

    tick++;
}

void LavaSolver::implicitHeatIntegrationMatrix(std::vector<double> &Ax,
                                               std::vector<double> const &x) {

    auto numGridCellNodes = gridCellNodes.size();

    for (auto c = 0; c < numGridCellNodes; c++) {
        auto const &cellNode = gridCellNodes[c];

        // Continue if later calculation may cause divide-by-zero error
        if (cellNode.mass == 0 || cellNode.specificHeat == 0) continue;

        double faceNodeValues[6] = {0, 0, 0, 0, 0, 0};

        // x-min boundary
        if (cellNode.location.x == 0) {
            faceNodeValues[0] = 0;
        } else {
            faceNodeValues[0] = x[c] - x[getGridCellNodeIndex(cellNode.location.x - 1,
                                                              cellNode.location.y,
                                                              cellNode.location.z)];
        }

        // x-max boundary
        if (cellNode.location.x == size.x - 1) {
            faceNodeValues[1] = 0;
        } else {
            faceNodeValues[1] = x[getGridCellNodeIndex(cellNode.location.x + 1,
                                                       cellNode.location.y,
                                                       cellNode.location.z)] - x[c];
        }

        // y-min boundary
        if (cellNode.location.y == 0) {
            faceNodeValues[2] = 0;
        } else {
            faceNodeValues[2] = x[c] - x[getGridCellNodeIndex(cellNode.location.x,
                                                              cellNode.location.y - 1,
                                                              cellNode.location.z)];
        }

        // y-max boundary
        if (cellNode.location.y == size.y - 1) {
            faceNodeValues[3] = 0;
        } else {
            faceNodeValues[3] = x[getGridCellNodeIndex(cellNode.location.x,
                                                       cellNode.location.y + 1,
                                                       cellNode.location.z)] - x[c];
        }

        // z-min boundary
        if (cellNode.location.z == 0) {
            faceNodeValues[4] = 0;
        } else {
            faceNodeValues[4] = x[c] - x[getGridCellNodeIndex(cellNode.location.x,
                                                              cellNode.location.y,
                                                              cellNode.location.z - 1)];
        }

        // z-max boundary
        if (cellNode.location.z == size.z - 1) {
            faceNodeValues[5] = 0;
        } else {
            faceNodeValues[5] = x[getGridCellNodeIndex(cellNode.location.x,
                                                       cellNode.location.y,
                                                       cellNode.location.z + 1)] - x[c];
        }

        Ax[c] = x[c] + delta_t * pow(h, 3) / (cellNode.mass * cellNode.specificHeat) *
                       (gridFaceXNode(cellNode.location.x + 1,
                                      cellNode.location.y,
                                      cellNode.location.z).thermalConductivity * faceNodeValues[1] -
                        gridFaceXNode(cellNode.location.x,
                                      cellNode.location.y,
                                      cellNode.location.z).thermalConductivity * faceNodeValues[0] +
                        gridFaceYNode(cellNode.location.x,
                                      cellNode.location.y + 1,
                                      cellNode.location.z).thermalConductivity * faceNodeValues[3] -
                        gridFaceYNode(cellNode.location.x,
                                      cellNode.location.y,
                                      cellNode.location.z).thermalConductivity * faceNodeValues[2] +
                        gridFaceZNode(cellNode.location.x,
                                      cellNode.location.y,
                                      cellNode.location.z + 1).thermalConductivity * faceNodeValues[5] -
                        gridFaceZNode(cellNode.location.x,
                                      cellNode.location.y,
                                      cellNode.location.z).thermalConductivity * faceNodeValues[4]);

    }

}
