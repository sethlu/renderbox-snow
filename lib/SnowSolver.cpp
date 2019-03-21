#include "SnowSolver.h"

#include <fstream>

#include <glm/gtc/type_ptr.hpp>
#include <Dense>

#include "conjugate_residual_solver.h"


typedef Eigen::Matrix<double, 3, 3> eigen_matrix3;
typedef Eigen::Matrix<double, 3, 1> eigen_vector3;


SnowSolver::SnowSolver(double h, glm::uvec3 const &size) : h(h), size(size) {

}

SnowSolver::SnowSolver(std::string const &filename) {
    loadState(filename);
}

void svd(glm::dmat3 const &m, glm::dmat3 &u, glm::dvec3 &e, glm::dmat3 &v) {
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

glm::dmat3 polarRot(glm::dmat3 const &m) {
    glm::dmat3 u;
    glm::dvec3 e;
    glm::dmat3 v;
    svd(m, u, e, v);
    return u * glm::transpose(v);
}

void polarDecompose(glm::dmat3 const &m, glm::dmat3 &r, glm::dmat3 &s) {
    glm::dmat3 u;
    glm::dvec3 e;
    glm::dmat3 v;
    svd(m, u, e, v);
    r = u * glm::transpose(v);
    s = v * glm::dmat3(e.x, 0, 0, 0, e.y, 0, 0, 0, e.z) * glm::transpose(v);
}

void SnowSolver::propagateSimulationParametersUpdate() {
    simulationParametersDidUpdate = false;

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

void SnowSolver::update() {
    LOG(INFO) << "delta_t=" << delta_t << " tick=" << tick << std::endl;

    if (simulationParametersDidUpdate) {
        propagateSimulationParametersUpdate();
    }

    auto numGridNodes = gridNodes.size();
    auto numParticleNodes = particleNodes.size();

    // 1. Rasterize particle data to the grid //////////////////////////////////////////////////////////////////////////

    LOG(VERBOSE) << "Step 1" << std::endl;

    for (auto i = 0; i < numGridNodes; i++) {
        auto &gridNode = gridNodes[i];

        gridNode.mass = 0;
        gridNode.velocity(tick) = {};

    }

    double totalGridNodeMass = 0;

    for (auto p = 0; p < numParticleNodes; p++) {
        auto &particleNode = particleNodes[p];
        auto gmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));

        // Nearby weighted grid nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gmin.x + i / 16;
            auto gy = gmin.y + (i / 4) % 4;
            auto gz = gmin.z + i % 4;
            if (!isValidGridNode(gx, gy, gz)) continue;
            auto &gridNode = this->gridNode(gx, gy, gz);

            // Pre-compute weights
            particleNode.weight[i] = weight(gridNode, particleNode);
            particleNode.nabla_weight[i] = nabla_weight(gridNode, particleNode);

            auto particleWeightedMass = particleNode.mass * particleNode.weight[i];

            gridNode.mass += particleWeightedMass;
            gridNode.velocity(tick) += particleNode.velocity(tick) * particleWeightedMass; // Translational momentum

            totalGridNodeMass += particleWeightedMass;
        }

    }

    LOG(VERBOSE) << "sum(gridNode.mass)=" << totalGridNodeMass << std::endl;

    for (auto i = 0; i < numGridNodes; i++) {
        auto &gridNode = gridNodes[i];

        // Compute velocity
        if (glm::length(gridNode.velocity(tick)) > 0 && gridNode.mass > 0) {
            gridNode.velocity(tick) /= gridNode.mass;
        } else {
            gridNode.velocity(tick) = {};
        }

    }

    // 2. Compute particle volumes and densities ///////////////////////////////////////////////////////////////////////

    if (tick == 0) {

        LOG(VERBOSE) << "Step 2" << std::endl;

        double totalDensity = 0;

        for (auto i = 0; i < numGridNodes; i++) {
            auto &gridNode = gridNodes[i];

            gridNode.density0 = gridNode.mass / (h * h * h);
            totalDensity += gridNode.density0;

        }

        LOG(VERBOSE) << "avg(gridNode.density0)=" << totalDensity / gridNodes.size() << std::endl;

        totalDensity = 0;

        for (auto p = 0; p < numParticleNodes; p++) {
            auto &particleNode = particleNodes[p];
            auto gmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));

            // Nearby weighted grid nodes
            double particleNodeDensity0 = 0;
            for (unsigned int i = 0; i < 64; i++) {
                auto gx = gmin.x + i / 16;
                auto gy = gmin.y + (i / 4) % 4;
                auto gz = gmin.z + i % 4;
                if (!isValidGridNode(gx, gy, gz)) continue;
                auto &gridNode = this->gridNode(gx, gy, gz);

                particleNodeDensity0 += gridNode.density0 * particleNode.weight[i];

            }

            particleNode.volume0 = particleNode.mass / particleNodeDensity0;
            totalDensity += particleNodeDensity0;
        }

        LOG(VERBOSE) << "avg(particleNodeDensity0)=" << totalDensity / particleNodes.size() << std::endl;

    }

    // 3. Compute grid forces //////////////////////////////////////////////////////////////////////////////////////////
    // 4. Update velocities on grid ////////////////////////////////////////////////////////////////////////////////////
    // 5. Grid-based body collisions ///////////////////////////////////////////////////////////////////////////////////

    LOG(VERBOSE) << "Step 3, 4, 5, 6" << std::endl;

    // 3

    for (auto i = 0; i < numGridNodes; i++) {
        auto &gridNode = gridNodes[i];

        gridNode.force = glm::dvec3(0, 0, -9.8 * gridNode.mass);

    }

    for (auto p = 0; p < numParticleNodes; p++) {
        auto const &particleNode = particleNodes[p];
        auto gmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));

        auto jp = glm::determinant(particleNode.deformPlastic);
        auto je = glm::determinant(particleNode.deformElastic);

        auto e = exp(hardeningCoefficient * (1 - jp));
        auto mu = mu0 * e;
        auto lambda = lambda0 * e;

        auto unweightedForce = -particleNode.volume0 *
                               (2 * mu * (particleNode.deformElastic - polarRot(particleNode.deformElastic)) *
                                glm::transpose(particleNode.deformElastic) +
                                glm::dmat3(lambda * (je - 1) * je));

        // Nearby weighted grid nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gmin.x + i / 16;
            auto gy = gmin.y + (i / 4) % 4;
            auto gz = gmin.z + i % 4;
            if (!isValidGridNode(gx, gy, gz)) continue;
            auto &gridNode = this->gridNode(gx, gy, gz);

            gridNode.force += unweightedForce * particleNode.nabla_weight[i];

        }

    }

    for (auto i = 0; i < numGridNodes; i++) {
        auto &gridNode = gridNodes[i];

        // 4

        gridNode.velocity_star = gridNode.velocity(tick);
        if (glm::length(gridNode.force) > 0 && gridNode.mass > 0) {
            gridNode.velocity_star += delta_t * gridNode.force / gridNode.mass;
        }

        // 5

        if (handleNodeCollisionVelocityUpdate)
            handleNodeCollisionVelocityUpdate(gridNode);

    }

    // 6. Solve the linear system //////////////////////////////////////////////////////////////////////////////////////

    if (beta > 0) {

        std::vector<glm::dvec3> velocity_star(gridNodes.size());
        std::vector<glm::dvec3> velocity_next(gridNodes.size());

        for (auto i = 0; i < numGridNodes; i++) {
            auto &gridNode = gridNodes[i];

            velocity_star[i] = gridNodes[i].velocity_star;
            velocity_next[i] = gridNodes[i].velocity_star;

        }

        conjugateResidualSolver(this, &SnowSolver::implicitVelocityIntegrationMatrix,
                                velocity_next, velocity_star, 500, 1e-10);

        for (auto i = 0; i < numGridNodes; i++) {
            auto &gridNode = gridNodes[i];

            gridNodes[i].velocity(tick + 1) = velocity_next[i];

        }

    } else {

        for (auto i = 0; i < numGridNodes; i++) {
            auto &gridNode = gridNodes[i];

            gridNode.velocity(tick + 1) = gridNode.velocity_star;

        }

    }

    // 7. Update deformation gradient //////////////////////////////////////////////////////////////////////////////////
    // 8. Update particle velocities ///////////////////////////////////////////////////////////////////////////////////
    // 9. Particle-based body collisions ///////////////////////////////////////////////////////////////////////////////
    // 10. Update particle positions ///////////////////////////////////////////////////////////////////////////////////

    LOG(VERBOSE) << "Step 7, 8, 9, 10" << std::endl;

    for (auto p = 0; p < numParticleNodes; p++) {
        auto &particleNode = particleNodes[p];
        auto gmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));

        // 7

        glm::dmat3 deform = particleNode.deformElastic * particleNode.deformPlastic;

        glm::dmat3 nabla_v{};

        // Nearby weighted grid nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gmin.x + i / 16;
            auto gy = gmin.y + (i / 4) % 4;
            auto gz = gmin.z + i % 4;
            if (!isValidGridNode(gx, gy, gz)) continue;
            auto &gridNode = this->gridNode(gx, gy, gz);

            nabla_v += glm::outerProduct(gridNode.velocity(tick + 1), particleNode.nabla_weight[i]);

        }

        glm::dmat3 multiplier = glm::dmat3(1) + delta_t * nabla_v;

        glm::dmat3 deform_prime = multiplier * deform;
        glm::dmat3 deformElastic_prime = multiplier * particleNode.deformElastic;

        glm::dmat3 u;
        glm::dvec3 e;
        glm::dmat3 v;
        svd(deformElastic_prime, u, e, v);
        e = glm::clamp(e, 1 - criticalCompression, 1 + criticalStretch);

        particleNode.deformElastic = u * glm::dmat3(e.x, 0, 0, 0, e.y, 0, 0, 0, e.z) * glm::transpose(v);
        particleNode.deformPlastic =
                v * glm::dmat3(1 / e.x, 0, 0, 0, 1 / e.y, 0, 0, 0, 1 / e.z) * glm::transpose(u) * deform_prime;

        // 8

        auto v_pic = glm::dvec3();
        auto v_flip = particleNode.velocity(tick);

        // Nearby weighted grid nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gmin.x + i / 16;
            auto gy = gmin.y + (i / 4) % 4;
            auto gz = gmin.z + i % 4;
            if (!isValidGridNode(gx, gy, gz)) continue;
            auto &gridNode = this->gridNode(gx, gy, gz);

            auto w = particleNode.weight[i];
            auto gv = gridNode.velocity(tick);
            auto gv1 = gridNode.velocity(tick + 1);

            v_pic += gv1 * w;
            v_flip += (gv1 - gv) * w;

        }

        particleNode.velocity_star = (1 - alpha) * v_pic + alpha * v_flip;

        // 9

        if (handleNodeCollisionVelocityUpdate)
            handleNodeCollisionVelocityUpdate(particleNode);

        particleNode.velocity(tick + 1) = particleNode.velocity_star;

        // 10

        particleNode.position += delta_t * particleNode.velocity(tick + 1);

    }

    tick++;

}

double ddot(glm::dmat3 a, glm::dmat3 b) {
    return a[0][0] * b[0][0] + a[0][1] * b[0][1] + a[0][2] * b[0][2] +
           a[1][0] * b[1][0] + a[1][1] * b[1][1] + a[1][2] * b[1][2] +
           a[2][0] * b[2][0] + a[2][1] * b[2][1] + a[2][2] * b[2][2];
}

void
SnowSolver::implicitVelocityIntegrationMatrix(std::vector<glm::dvec3> &Av_next, std::vector<glm::dvec3> const &v_next) {
    LOG_ASSERT(Av_next.size() == v_next.size() && v_next.size() == gridNodes.size());

    auto numGridNodes = gridNodes.size();
    auto numParticleNodes = particleNodes.size();

    // x^n+1

    std::vector<glm::dvec3> x_next(numGridNodes);

    for (auto i = 0; i < numGridNodes; i++) {
        x_next[i] = gridNodes[i].position + delta_t * v_next[i];
    }

    // del_f

    std::vector<glm::dvec3> del_f(numGridNodes);

    for (auto p = 0; p < numParticleNodes; p++) {
        auto const &particleNode = particleNodes[p];
        auto gmin = glm::ivec3((particleNode.position / h) - glm::dvec3(1));

        // del_deformElastic

        glm::dmat3 del_deformElastic{};

        // Nearby weighted grid nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gmin.x + i / 16;
            auto gy = gmin.y + (i / 4) % 4;
            auto gz = gmin.z + i % 4;
            if (!isValidGridNode(gx, gy, gz)) continue;
            auto &gridNode = this->gridNode(gx, gy, gz);

            del_deformElastic += glm::outerProduct(v_next[getGridNodeIndex(gx, gy, gz)],
                                                   particleNode.nabla_weight[i]);

        }

        del_deformElastic = delta_t * del_deformElastic * particleNode.deformElastic;

        // del_polarRotDeformElastic

        glm::dmat3 r, s;
        polarDecompose(particleNode.deformElastic, r, s);

        auto rtdf_dftr = (glm::transpose(r) * del_deformElastic - glm::transpose(del_deformElastic) * r);
        auto rtdr = glm::inverse(glm::dmat3(s[0][0] + s[1][1], s[2][1], -s[2][0],
                                            s[1][2], s[0][0] + s[2][2], s[0][1],
                                            -s[2][0], s[1][0], s[2][2] + s[1][1])) *
                    glm::dvec3(rtdf_dftr[1][0], rtdf_dftr[2][0], rtdf_dftr[2][1]);

        auto del_polarRotDeformElastic =
                r * glm::dmat3(0, -rtdr.x, -rtdr.y,
                               rtdr.x, 0, -rtdr.z,
                               rtdr.y, rtdr.z, 0);

        // jp, je, mu, lambda

        auto jp = glm::determinant(particleNode.deformPlastic);
        auto je = glm::determinant(particleNode.deformElastic);

        auto e = exp(hardeningCoefficient * (1 - jp));
        auto mu = mu0 * e;
        auto lambda = lambda0 * e;

        auto cofactor_deformElastic = je * glm::transpose(glm::inverse(particleNode.deformElastic));

        // del_je

        // Take Frobenius inner product
        auto del_je = ddot(cofactor_deformElastic, del_deformElastic);

        // del_cofactor_deformElastic

        auto &cde = cofactor_deformElastic;

        auto del_cofactor_deformElastic = glm::dmat3(
                ddot(glm::dmat3(0, 0, 0,
                                0, cde[2][2], -cde[2][1],
                                0, -cde[1][2], cde[1][1]),
                     del_deformElastic),
                ddot(glm::dmat3(0, 0, 0,
                                -cde[2][2], 0, cde[2][0],
                                cde[1][2], 0, -cde[1][0]),
                     del_deformElastic),
                ddot(glm::dmat3(0, 0, 0,
                                cde[2][1], -cde[2][0], 0,
                                -cde[1][1], cde[1][0], 0),
                     del_deformElastic),

                ddot(glm::dmat3(0, -cde[2][2], cde[2][1],
                                0, 0, 0,
                                0, cde[0][2], -cde[0][1]),
                     del_deformElastic),
                ddot(glm::dmat3(cde[2][2], 0, -cde[2][0],
                                0, 0, 0,
                                -cde[0][2], 0, cde[0][0]),
                     del_deformElastic),
                ddot(glm::dmat3(-cde[2][1], cde[2][0], 0,
                                0, 0, 0,
                                cde[0][1], -cde[0][0], 0),
                     del_deformElastic),

                ddot(glm::dmat3(0, cde[1][2], -cde[1][1],
                                0, -cde[0][2], cde[0][1],
                                0, 0, 0),
                     del_deformElastic),
                ddot(glm::dmat3(-cde[1][2], 0, cde[1][0],
                                cde[0][2], 0, -cde[0][0],
                                0, 0, 0),
                     del_deformElastic),
                ddot(glm::dmat3(cde[1][1], -cde[1][0], 0,
                                -cde[0][1], cde[0][0], 0,
                                0, 0, 0),
                     del_deformElastic));

        // Accumulate to del_f

        auto unweightedDelForce =
                -particleNode.volume0 * (2 * mu * (del_deformElastic - del_polarRotDeformElastic) +
                                         lambda * (cofactor_deformElastic * del_je +
                                                   (je - 1) * del_cofactor_deformElastic)) *
                glm::transpose(particleNode.deformElastic);

        // Nearby weighted grid nodes
        for (unsigned int i = 0; i < 64; i++) {
            auto gx = gmin.x + i / 16;
            auto gy = gmin.y + (i / 4) % 4;
            auto gz = gmin.z + i % 4;
            if (!isValidGridNode(gx, gy, gz)) continue;
            auto &gridNode = this->gridNode(gx, gy, gz);

            del_f[getGridNodeIndex(gx, gy, gz)] += unweightedDelForce * particleNode.nabla_weight[i];

        }

    }

    // Av_next

    for (auto i = 0; i < numGridNodes; i++) {
        Av_next[i] = v_next[i];
        if (gridNodes[i].mass > 0) {
            Av_next[i] -= beta * delta_t * del_f[i] / gridNodes[i].mass;
        }
    }

}

void SnowSolver::saveState(std::string const &filename) {
    std::ofstream file;
    file.open(filename, std::ofstream::binary | std::ofstream::trunc);

    SNOW_SOLVER_STATE_HEADER solverStateHeader{
            youngsModulus0,
            criticalCompression,
            criticalStretch,
            hardeningCoefficient,
            h,
            size,
            tick,
            delta_t,
            alpha,
            beta,
            particleNodes.size()
    };

    file.write(reinterpret_cast<char *>(&solverStateHeader), sizeof(SNOW_SOLVER_STATE_HEADER));

    SNOW_SOLVER_STATE_PARTICLE particleState{};
    for (auto const &particleNode : particleNodes) {
        particleState.position = particleNode.position;
        particleState.velocity = particleNode.velocity(tick);
        particleState.mass = particleNode.mass;
        particleState.volume0 = particleNode.volume0;
        particleState.deformElastic = particleNode.deformElastic;
        particleState.deformPlastic = particleNode.deformPlastic;

        file.write(reinterpret_cast<char *>(&particleState), sizeof(SNOW_SOLVER_STATE_PARTICLE));
    }

    file.close();
}

void SnowSolver::loadState(std::string const &filename) {
    std::ifstream file(filename, std::ifstream::binary);

    ParticleNode emptyParticleNode{{},
                                   {}};

    SNOW_SOLVER_STATE_HEADER solverStateHeader{};
    file.read(reinterpret_cast<char *>(&solverStateHeader), sizeof(SNOW_SOLVER_STATE_HEADER));
    youngsModulus0 = solverStateHeader.youngsModulus0;
    criticalCompression = solverStateHeader.criticalCompression;
    criticalStretch = solverStateHeader.criticalStretch;
    hardeningCoefficient = solverStateHeader.hardeningCoefficient;
    h = solverStateHeader.h;
    size = solverStateHeader.size;
    tick = solverStateHeader.tick;
    delta_t = solverStateHeader.delta_t;
    alpha = solverStateHeader.alpha;
    beta = solverStateHeader.beta;
    particleNodes.resize(solverStateHeader.numParticles, emptyParticleNode);

    SNOW_SOLVER_STATE_PARTICLE particleState{};
    for (auto &particleNode : particleNodes) {
        file.read(reinterpret_cast<char *>(&particleState), sizeof(SNOW_SOLVER_STATE_PARTICLE));

        particleNode.position = particleState.position;
        particleNode.velocity(tick) = particleState.velocity;
        particleNode.mass = particleState.mass;
        particleNode.volume0 = particleState.volume0;
        particleNode.deformElastic = particleState.deformElastic;
        particleNode.deformPlastic = particleState.deformPlastic;
    }

    file.close();

    simulationParametersDidUpdate = true;
}
