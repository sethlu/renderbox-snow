#include "SnowSolver.h"

#include <glm/gtc/type_ptr.hpp>
#include <Dense>

#include "logging.h"


typedef Eigen::Matrix<float,3,3> eigen_matrix3;
typedef Eigen::Matrix<float,3,1> eigen_vector3;


SnowSolver::SnowSolver(float h, glm::uvec3 const &size) : h(h), invh(1 / h), size(size) {
    LOG(INFO) << "size=" << size << std::endl;

    for (auto x = 0; x < size.x; x++) {
        for (auto y = 0; y < size.y; y++) {
            for (auto z = 0; z < size.z; z++) {
                gridNodes.emplace_back(glm::vec3(x, y, z) * h, glm::uvec3(x, y, z));
            }
        }
    }

    LOG(INFO) << "#gridNodes=" << gridNodes.size() << std::endl;
}

void svd(glm::mat3 const &m, glm::mat3 &u, glm::vec3 &e, glm::mat3 &v) {
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

glm::mat3 polarRot(glm::mat3 const &m) {
    glm::mat3 u;
    glm::vec3 e;
    glm::mat3 v;
    svd(m, u, e, v);
    return u * v;
}

void SnowSolver::update(float delta_t, unsigned int n) {
    LOG(VERBOSE) << "delta_t=" << delta_t << " n=" << n << std::endl;

    // 1. Rasterize particle data to the grid //////////////////////////////////////////////////////////////////////////

    LOG(VERBOSE) << "Step 1" << std::endl;

    for (auto &gridNode : gridNodes) {
        gridNode.mass = 0;
        gridNode.velocity(n) = {};
    }

    for (auto &particleNode : particleNodes) {

        // Nearby weighted grid nodes
        auto gmin = glm::uvec3((particleNode.position / h) - glm::vec3(1));
        auto gmax = glm::uvec3((particleNode.position / h) + glm::vec3(2));
        for (auto gx = gmin.x; gx <= gmax.x; gx++) {
            for (auto gy = gmin.y; gy <= gmax.y; gy++) {
                for (auto gz = gmin.z; gz <= gmax.z; gz++) {
                    if (!isValidGridNode(gx, gy, gz)) continue;
                    auto &gridNode = this->gridNode(gx, gy, gz);

                    auto particleWeightedMass = particleNode.mass * weight(gridNode, particleNode);

                    gridNode.mass += particleWeightedMass;
                    gridNode.velocity(n) += particleNode.velocity(n) * particleWeightedMass;

                }
            }
        }

    }

    for (auto &gridNode : gridNodes) {
        if (gridNode.mass > 0) {
            gridNode.velocity(n) /= gridNode.mass;
        } else {
            gridNode.velocity(n) = {};
        }
    }

    // 2. Compute particle volumes and densities ///////////////////////////////////////////////////////////////////////

    LOG(VERBOSE) << "Step 2" << std::endl;

    if (n == 0) {

        for (auto &gridNode : gridNodes) {
            gridNode.density0 = gridNode.mass / (h * h * h);
        }

        for (auto &particleNode : particleNodes) {
            auto particledNodeDensity0 = 0.f;
            for (auto const &gridNode : gridNodes) {
                particledNodeDensity0 += gridNode.density0 * weight(gridNode, particleNode);
            }
            particleNode.volume0 = particleNode.mass / particledNodeDensity0;
        }

    }

    // 3. Compute grid forces //////////////////////////////////////////////////////////////////////////////////////////
    // 4. Update velocities on grid ////////////////////////////////////////////////////////////////////////////////////
    // 5. Grid-based body collisions ///////////////////////////////////////////////////////////////////////////////////
    // 6. Solve the linear system //////////////////////////////////////////////////////////////////////////////////////

    LOG(VERBOSE) << "Step 3, 4, 5, 6" << std::endl;

    // 3

    for (auto &gridNode : gridNodes) {
        gridNode.force = glm::vec3(0, 0, - 9.8 * gridNode.mass);
    }

    for (auto const &particleNode : particleNodes) {

        auto jp = glm::determinant(particleNode.deformPlastic);
        auto je = glm::determinant(particleNode.deformElastic);

        auto e = exp(hardeningCoefficient * (1 - jp));
        auto mu = mu0 * e;
        auto lambda = lambda0 * e;

        auto unweightedForce = particleNode.volume0 *
                               (2 * mu * (particleNode.deformElastic - polarRot(particleNode.deformElastic)) *
                                glm::transpose(particleNode.deformElastic) +
                                lambda * (je - 1) * je * glm::mat3(1));

        // Nearby weighted grid nodes
        auto gmin = glm::uvec3((particleNode.position / h) - glm::vec3(1));
        auto gmax = glm::uvec3((particleNode.position / h) + glm::vec3(2));
        for (auto gx = gmin.x; gx <= gmax.x; gx++) {
            for (auto gy = gmin.y; gy <= gmax.y; gy++) {
                for (auto gz = gmin.z; gz <= gmax.z; gz++) {
                    if (!isValidGridNode(gx, gy, gz)) continue;
                    auto &gridNode = this->gridNode(gx, gy, gz);

                    gridNode.force += unweightedForce * nabla_weight(gridNode, particleNode);

                }
            }
        }

    }

    for (auto &gridNode : gridNodes) {

        // 4

        gridNode.velocity_star = gridNode.velocity(n);
        if (gridNode.mass > 0) {
            gridNode.velocity_star += delta_t * gridNode.force / gridNode.mass;
        }

        // 5

        handleNodeCollisionVelocityUpdate(gridNode);

        // 6

        gridNode.velocity(n + 1) = gridNode.velocity_star;

    }

    // 7. Update deformation gradient //////////////////////////////////////////////////////////////////////////////////
    // 8. Update particle velocities ///////////////////////////////////////////////////////////////////////////////////
    // 9. Particle-based body collisions ///////////////////////////////////////////////////////////////////////////////
    // 10. Update particle positions ///////////////////////////////////////////////////////////////////////////////////

    LOG(VERBOSE) << "Step 7, 8, 9, 10" << std::endl;

    for (auto &particleNode : particleNodes) {

        // 7

        glm::mat3 deform = particleNode.deformElastic * particleNode.deformPlastic;

        glm::vec3 nabla_v = glm::vec3();

        // Nearby weighted grid nodes
        auto gmin = glm::uvec3((particleNode.position / h) - glm::vec3(1));
        auto gmax = glm::uvec3((particleNode.position / h) + glm::vec3(2));
        for (auto gx = gmin.x; gx <= gmax.x; gx++) {
            for (auto gy = gmin.y; gy <= gmax.y; gy++) {
                for (auto gz = gmin.z; gz <= gmax.z; gz++) {
                    if (!isValidGridNode(gx, gy, gz)) continue;
                    auto &gridNode = this->gridNode(gx, gy, gz);

                    nabla_v += gridNode.velocity(n + 1) * nabla_weight(gridNode, particleNode);

                }
            }
        }

        glm::mat3 multiplier = glm::mat3(1) + delta_t * glm::mat3(nabla_v.x, 0, 0, 0, nabla_v.y, 0, 0, 0, nabla_v.z);

        glm::mat3 deform_prime = multiplier * deform;
        glm::mat3 deformElastic_prime = multiplier * particleNode.deformElastic;

        glm::mat3 u;
        glm::vec3 e;
        glm::mat3 v;
        svd(deformElastic_prime, u, e, v);
        e = glm::clamp(e, 1 - criticalCompression, 1 + criticalStretch);

        particleNode.deformElastic = u * glm::mat3(e.x, 0, 0, 0, e.y, 0, 0, 0, e.z) * v;
        particleNode.deformPlastic = glm::inverse(particleNode.deformElastic) * deform_prime;

        // 8

        auto v_pic = glm::vec3();
        auto v_flip = particleNode.velocity(n);

        // Nearby weighted grid nodes
        for (auto gx = gmin.x; gx <= gmax.x; gx++) {
            for (auto gy = gmin.y; gy <= gmax.y; gy++) {
                for (auto gz = gmin.z; gz <= gmax.z; gz++) {
                    if (!isValidGridNode(gx, gy, gz)) continue;
                    auto &gridNode = this->gridNode(gx, gy, gz);

                    auto w = weight(gridNode, particleNode);
                    auto gv = gridNode.velocity(n);
                    auto gv1 = gridNode.velocity(n + 1);

                    v_pic += gv1 * w;
                    v_flip += (gv1 - gv) * w;

                }
            }
        }

        particleNode.velocity_star = (1 - alpha) * v_pic + alpha * v_flip;

        // 9

        handleNodeCollisionVelocityUpdate(particleNode);
        particleNode.velocity(n + 1) = particleNode.velocity_star;

        // 10

        particleNode.position += delta_t * particleNode.velocity(n + 1);

    }

}

/**
 * Updates node.velocity_star
 */
void SnowSolver::handleNodeCollisionVelocityUpdate(Node &node) {

    // Hard-coded floor collision & it's not moving anywhere
    if (node.position.z <= 0) {
        auto v_co = glm::vec3(0); // Velocity of collider object
        auto n = glm::vec3(0, 0, 1); // Normal
        auto mu = 1.f; // Coefficient of friction

        // Relative velocity to collider object
        auto v_rel = node.velocity_star - v_co;

        auto v_n = glm::dot(v_rel, n);
        if (v_n >= 0) {
            // No collision
            return;
        }

        // Tangential velocity
        auto v_t = v_rel - n * v_n;

        // Sticking impulse
        if (glm::length(v_t) <= - mu * v_n) {
            v_rel = glm::vec3(0);
        } else {
            v_rel = v_t + mu * v_n + glm::normalize(v_t);
        };

        node.velocity_star = v_rel + v_co;

    }

}
