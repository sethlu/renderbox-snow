#include "LavaSolver.h"

#include <glm/gtc/type_ptr.hpp>
#include <Dense>

#include "conjugate_residual_solver.h"


typedef Eigen::Matrix<double, 3, 3> eigen_matrix3;
typedef Eigen::Matrix<double, 3, 1> eigen_vector3;


inline void svd(glm::dmat3 const &m, glm::dmat3 &u, glm::dvec3 &e, glm::dmat3 &v) {
    Eigen::Map<eigen_matrix3 const> mmap(glm::value_ptr(m));
    Eigen::Map<eigen_matrix3> umap(glm::value_ptr(u));
    Eigen::Map<eigen_vector3> emap(glm::value_ptr(e));
    Eigen::Map<eigen_matrix3> vmap(glm::value_ptr(v));

    Eigen::JacobiSVD<eigen_matrix3, Eigen::NoQRPreconditioner> svd;
    svd.compute(mmap, Eigen::ComputeFullV | Eigen::ComputeFullU);
    umap = svd.matrixU();
    emap = svd.singularValues();
    vmap = svd.matrixV();
}

inline glm::dmat3 polarRot(glm::dmat3 const &m) {
    glm::dmat3 u;
    glm::dvec3 e;
    glm::dmat3 v;
    svd(m, u, e, v);
    return u * glm::transpose(v);
}

inline void polarDecompose(glm::dmat3 const &m, glm::dmat3 &r, glm::dmat3 &s) {
    glm::dmat3 u;
    glm::dvec3 e;
    glm::dmat3 v;
    svd(m, u, e, v);
    r = u * glm::transpose(v);
    s = v * glm::dmat3(e.x, 0, 0, 0, e.y, 0, 0, 0, e.z) * glm::transpose(v);
}

glm::dmat3 deformationUpdateR(glm::dmat3 m) {
    if (glm::determinant(glm::dmat3(1) + m) > 0) {
        return glm::dmat3(1) + m;
    }
    auto t = deformationUpdateR(0.5 * m);
    return t * t;
}

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
        cellNode.velocity = {};
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
        faceNode.velocity = {};
        faceNode.thermalConductivity = 0;
    }
    for (auto i = 0; i < numGridFaceYNodes; i++) {
        auto &faceNode = gridFaceYNodes[i];

        faceNode.mass = 0;
        faceNode.velocity = {};
        faceNode.thermalConductivity = 0;
    }
    for (auto i = 0; i < numGridFaceZNodes; i++) {
        auto &faceNode = gridFaceZNodes[i];

        faceNode.mass = 0;
        faceNode.velocity = {};
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
            cellNode.velocity += particleNode.velocity * particleWeightedMass;
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

            auto particleWeightedMass = particleNode.mass * particleNode.face_x_weight[i];

            faceNode.mass += particleWeightedMass;
            faceNode.velocity += particleNode.velocity * particleWeightedMass;
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

            auto particleWeightedMass = particleNode.mass * particleNode.face_y_weight[i];

            faceNode.mass += particleWeightedMass;
            faceNode.velocity += particleNode.velocity * particleWeightedMass;
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

            auto particleWeightedMass = particleNode.mass * particleNode.face_z_weight[i];

            faceNode.mass += particleWeightedMass;
            faceNode.velocity += particleNode.velocity * particleWeightedMass;
            faceNode.thermalConductivity += particleNode.thermalConductivity * particleWeightedMass;
        }

    }

    for (auto i = 0; i < numGridCellNodes; i++) {
        auto &cellNode = gridCellNodes[i];

        if (cellNode.mass > 0) {
            cellNode.velocity /= cellNode.mass;
            cellNode.j /= cellNode.mass;
            cellNode.je /= cellNode.mass;
            cellNode.jp = cellNode.j / cellNode.je;
            cellNode.specificHeat /= cellNode.mass;
            cellNode.temperature /= cellNode.mass;
            cellNode.inv_lambda /= cellNode.mass;
        } else {
            cellNode.velocity = {};
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
            gridFaceNode.velocity /= gridFaceNode.mass;
            gridFaceNode.thermalConductivity /= gridFaceNode.mass;
        } else {
            gridFaceNode.velocity = {};
            gridFaceNode.thermalConductivity = 0;
        }

        gridFaceNode.colliding = isNodeColliding(gridFaceNode);
    }
    for (auto i = 0; i < numGridFaceYNodes; i++) {
        auto &gridFaceNode = gridFaceYNodes[i];

        if (gridFaceNode.mass > 0) {
            gridFaceNode.velocity /= gridFaceNode.mass;
            gridFaceNode.thermalConductivity /= gridFaceNode.mass;
        } else {
            gridFaceNode.velocity = {};
            gridFaceNode.thermalConductivity = 0;
        }

        gridFaceNode.colliding = isNodeColliding(gridFaceNode);
    }
    for (auto i = 0; i < numGridFaceZNodes; i++) {
        auto &gridFaceNode = gridFaceZNodes[i];

        if (gridFaceNode.mass > 0) {
            gridFaceNode.velocity /= gridFaceNode.mass;
            gridFaceNode.thermalConductivity /= gridFaceNode.mass;
        } else {
            gridFaceNode.velocity = {};
            gridFaceNode.thermalConductivity = 0;
        }

        gridFaceNode.colliding = isNodeColliding(gridFaceNode);
    }

    // Compute particle volumes and densities

    if (tick == 0) {

        for (auto p = 0; p < numParticleNodes; p++) {
            auto &particleNode = particleNodes[p];
            auto gmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));

            // Nearby weighted grid nodes
            double particleNodeDensity0 = 0;
            for (unsigned int i = 0; i < 64; i++) {
                auto gx = gmin.x + i / 16;
                auto gy = gmin.y + (i / 4) % 4;
                auto gz = gmin.z + i % 4;
                if (!isValidGridCellNode(gx, gy, gz)) continue;
                auto &cellNode = this->gridCellNode(gx, gy, gz);

                particleNodeDensity0 += cellNode.mass / (h * h * h) * particleNode.cell_weight[i];

            }

            particleNode.volume0 = particleNode.mass / particleNodeDensity0;

        }

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

    // 5. MPM velocity update //////////////////////////////////////////////////////////////////////////////////////////

    // TODO: Follow actual equation (23) for velocity explicit update

    // Clear face nodes
    for (auto i = 0; i < numGridFaceXNodes; i++) {
        auto &faceNode = gridFaceXNodes[i];

        faceNode.force = 0;
    }
    for (auto i = 0; i < numGridFaceYNodes; i++) {
        auto &faceNode = gridFaceYNodes[i];

        faceNode.force = 0;
    }
    for (auto i = 0; i < numGridFaceZNodes; i++) {
        auto &faceNode = gridFaceZNodes[i];

        faceNode.force = -9.8 * faceNode.mass;
    }

    // Transfer particle forces to faces
    for (auto p = 0; p < numParticleNodes; p++) {
        auto const &particleNode = particleNodes[p];
        auto gcmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));
        auto gfxmin = glm::ivec3((particleNode.position / h) - glm::dvec3(0.5, 1, 1));
        auto gfymin = glm::ivec3((particleNode.position / h) - glm::dvec3(1, 0.5, 1));
        auto gfzmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1, 1, 0.5));

        auto jp = glm::determinant(particleNode.deformPlastic);
        auto je = glm::determinant(particleNode.deformElastic);

        auto e = exp(particleNode.hardeningCoefficient * (1 - jp));
        auto mu = particleNode.mu0 * e;
        auto lambda = particleNode.lambda0 * e;

        // Set mu to 0 if particle liquid
        if (particleNode.temperature > particleNode.fusionTemperature + FLT_EPSILON) {
            mu = 0;
        }

        // TODO: Use actual derivative

        auto unweightedForce = -particleNode.volume0 *
                               (2 * mu * (particleNode.deformElastic - polarRot(particleNode.deformElastic)) *
                                glm::transpose(particleNode.deformElastic) +
                                glm::dmat3(lambda * (je - 1) * je));

        // Nearby weighted grid face nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfxmin.x + i / 16;
            auto gy = gfxmin.y + (i / 4) % 4;
            auto gz = gfxmin.z + i % 4;
            if (!isValidGridFaceXNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceXNode(gx, gy, gz);

            faceNode.force += (unweightedForce * particleNode.face_x_nabla_weight[i]).x;
        }
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfymin.x + i / 16;
            auto gy = gfymin.y + (i / 4) % 4;
            auto gz = gfymin.z + i % 4;
            if (!isValidGridFaceYNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceYNode(gx, gy, gz);

            faceNode.force += (unweightedForce * particleNode.face_y_nabla_weight[i]).y;
        }
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfzmin.x + i / 16;
            auto gy = gfzmin.y + (i / 4) % 4;
            auto gz = gfzmin.z + i % 4;
            if (!isValidGridFaceZNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceZNode(gx, gy, gz);

            faceNode.force += (unweightedForce * particleNode.face_z_nabla_weight[i]).z;
        }

    }

    for (auto i = 0; i < numGridFaceXNodes; i++) {
        auto &faceNode = gridFaceXNodes[i];

        if (faceNode.force != 0 && faceNode.mass > 0) {
            faceNode.velocity_star = faceNode.velocity + delta_t * faceNode.force / faceNode.mass;
        } else {
            faceNode.velocity_star = {};
        }
    }
    for (auto i = 0; i < numGridFaceYNodes; i++) {
        auto &faceNode = gridFaceYNodes[i];

        if (faceNode.force != 0 && faceNode.mass > 0) {
            faceNode.velocity_star = faceNode.velocity + delta_t * faceNode.force / faceNode.mass;
        } else {
            faceNode.velocity_star = {};
        }
    }
    for (auto i = 0; i < numGridFaceZNodes; i++) {
        auto &faceNode = gridFaceZNodes[i];

        if (faceNode.force != 0 && faceNode.mass > 0) {
            faceNode.velocity_star = faceNode.velocity + delta_t * faceNode.force / faceNode.mass;
        } else {
            faceNode.velocity_star = {};
        }
    }

    // 6. Process grid collisions //////////////////////////////////////////////////////////////////////////////////////

    if (handleNodeCollisionVelocityUpdate) {

        for (auto i = 0; i < numGridFaceXNodes; i++) {
            auto &faceNode = gridFaceXNodes[i];

            handleNodeCollisionVelocityUpdate(faceNode);
        }
        for (auto i = 0; i < numGridFaceYNodes; i++) {
            auto &faceNode = gridFaceYNodes[i];

            handleNodeCollisionVelocityUpdate(faceNode);
        }
        for (auto i = 0; i < numGridFaceZNodes; i++) {
            auto &faceNode = gridFaceZNodes[i];

            handleNodeCollisionVelocityUpdate(faceNode);
        }

    }

    // 7. Project velocities ///////////////////////////////////////////////////////////////////////////////////////////

    std::vector<double> next_quantity(numGridCellNodes);
    std::vector<double> quantity(numGridCellNodes);

    // TODO: Take care of the incompressible part

//    for (auto c = 0; c < numGridCellNodes; c++) {
//        auto &cellNode = gridCellNodes[c];
//
//        quantity[c] = cellNode.;
//        next_quantity[c] = cellNode.temperature;
//
//    }
//
//    conjugateResidualSolver(this, &LavaSolver::implicitHeatIntegrationMatrix,
//                            next_quantity, quantity, 50);
//
//    for (auto c = 0; c < numGridCellNodes; c++) {
//        auto &cellNode = gridCellNodes[c];
//
//        cellNode.temperature_next = next_quantity[c];
//
//    }

    // 8. Solve heat equation //////////////////////////////////////////////////////////////////////////////////////////

    for (auto c = 0; c < numGridCellNodes; c++) {
        auto &cellNode = gridCellNodes[c];

        quantity[c] = cellNode.temperature;
        next_quantity[c] = cellNode.temperature;

    }

    conjugateResidualSolver(this, &LavaSolver::implicitHeatIntegrationMatrix,
                            next_quantity, quantity, 50);

    for (auto c = 0; c < numGridCellNodes; c++) {
        auto &cellNode = gridCellNodes[c];

        cellNode.temperature_next = next_quantity[c];

    }

    // 9. Update particle state from grid //////////////////////////////////////////////////////////////////////////////

    // Velocity

    for (auto p = 0; p < numParticleNodes; p++) {
        auto &particleNode = particleNodes[p];
        auto gcmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));
        auto gfxmin = glm::ivec3((particleNode.position / h) - glm::dvec3(0.5, 1, 1));
        auto gfymin = glm::ivec3((particleNode.position / h) - glm::dvec3(1, 0.5, 1));
        auto gfzmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1, 1, 0.5));

        auto v_pic = glm::dvec3();
        auto v_flip = particleNode.velocity;

        // Nearby weighted grid face nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfxmin.x + i / 16;
            auto gy = gfxmin.y + (i / 4) % 4;
            auto gz = gfxmin.z + i % 4;
            if (!isValidGridFaceXNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceXNode(gx, gy, gz);

            auto w = particleNode.face_x_weight[i];
            auto gv = faceNode.velocity.x;
            auto gv1 = faceNode.velocity_star.x;

            v_pic.x += gv1 * w;
            v_flip.x += (gv1 - gv) * w;

        }
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfymin.x + i / 16;
            auto gy = gfymin.y + (i / 4) % 4;
            auto gz = gfymin.z + i % 4;
            if (!isValidGridFaceYNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceYNode(gx, gy, gz);

            auto w = particleNode.face_y_weight[i];
            auto gv = faceNode.velocity.y;
            auto gv1 = faceNode.velocity_star.y;

            v_pic.y += gv1 * w;
            v_flip.y += (gv1 - gv) * w;

        }
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfzmin.x + i / 16;
            auto gy = gfzmin.y + (i / 4) % 4;
            auto gz = gfzmin.z + i % 4;
            if (!isValidGridFaceZNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceZNode(gx, gy, gz);

            auto w = particleNode.face_z_weight[i];
            auto gv = faceNode.velocity.z;
            auto gv1 = faceNode.velocity_star.z;

            v_pic.z += gv1 * w;
            v_flip.z += (gv1 - gv) * w;

        }

        particleNode.velocity_star = (1 - alpha) * v_pic + alpha * v_flip;

        // 10

        if (handleNodeCollisionVelocityUpdate)
            handleNodeCollisionVelocityUpdate(particleNode);

        particleNode.velocity = particleNode.velocity_star;

        particleNode.position += delta_t * particleNode.velocity;

    }

    // Deformation gradient

    for (auto p = 0; p < numParticleNodes; p++) {
        auto &particleNode = particleNodes[p];
        auto gcmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));
        auto gfxmin = glm::ivec3((particleNode.position / h) - glm::dvec3(0.5, 1, 1));
        auto gfymin = glm::ivec3((particleNode.position / h) - glm::dvec3(1, 0.5, 1));
        auto gfzmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1, 1, 0.5));

        glm::dmat3 nabla_v{};

        // Nearby weighted grid face nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfxmin.x + i / 16;
            auto gy = gfxmin.y + (i / 4) % 4;
            auto gz = gfxmin.z + i % 4;
            if (!isValidGridFaceXNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceXNode(gx, gy, gz);

            nabla_v += glm::outerProduct(glm::dvec3(faceNode.velocity_star.x, 0, 0),
                                         particleNode.face_x_nabla_weight[i]);

        }
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfymin.x + i / 16;
            auto gy = gfymin.y + (i / 4) % 4;
            auto gz = gfymin.z + i % 4;
            if (!isValidGridFaceYNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceYNode(gx, gy, gz);

            nabla_v += glm::outerProduct(glm::dvec3(0, faceNode.velocity_star.y, 0),
                                         particleNode.face_y_nabla_weight[i]);

        }
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gfzmin.x + i / 16;
            auto gy = gfzmin.y + (i / 4) % 4;
            auto gz = gfzmin.z + i % 4;
            if (!isValidGridFaceZNode(gx, gy, gz)) continue;
            auto &faceNode = this->gridFaceZNode(gx, gy, gz);

            nabla_v += glm::outerProduct(glm::dvec3(0, 0, faceNode.velocity_star.z),
                                         particleNode.face_z_nabla_weight[i]);

        }

        // TODO: Use proper multiplier
        // auto multiplier = deformationUpdateR(delta_t * nabla_v);
        auto multiplier = glm::dmat3(1) + delta_t * nabla_v;

        glm::dmat3 deform = particleNode.deformElastic * particleNode.deformPlastic;
        glm::dmat3 deform_prime = multiplier * deform;
        auto deformElastic_prime = multiplier * particleNode.deformElastic;

        glm::dmat3 u;
        glm::dvec3 e;
        glm::dmat3 v;
        svd(deformElastic_prime, u, e, v);
        e = glm::clamp(e, 1 - particleNode.criticalCompression, 1 + particleNode.criticalStretch);

        particleNode.deformElastic = u * glm::dmat3(e.x, 0, 0, 0, e.y, 0, 0, 0, e.z) * glm::transpose(v);
        particleNode.deformPlastic =
                v * glm::dmat3(1 / e.x, 0, 0, 0, 1 / e.y, 0, 0, 0, 1 / e.z) * glm::transpose(u) * deform_prime;

        // Remove deviatoric component if liquid
        if (particleNode.temperature > particleNode.fusionTemperature + FLT_EPSILON) {
            particleNode.deformElastic = pow(glm::determinant(particleNode.deformElastic), 1.0 / 3.0) *
                                         glm::dmat3(1);
        }

    }

    // Temperature

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
