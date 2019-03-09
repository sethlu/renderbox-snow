#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE conjugate_residual_method

#include <boost/test/unit_test_suite.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>
#include <ostream>

namespace tt = boost::test_tools;

#include "../lib/conjugate_residual_solver.h"

// A[3x3]
void A(std::vector<float> &Ax, std::vector<float> const &x) {
    Ax[0] = 2 * x[0] + x[1] + x[2];
    Ax[1] = x[0] + 2 * x[1] + x[2];
    Ax[2] = x[0] + x[1] + 2 * x[2];
}

// B[3x3]
void B(std::vector<glm::vec3> &Ax, std::vector<glm::vec3> const &x) {
    Ax[0] = glm::mat3(2, 1, 1, 1, 2, 1, 1, 1, 2) * x[0];
}

template<typename T>
std::ostream &operator<<(std::ostream &os, std::vector<T> const &vector) {
    for (auto const &vec3 : vector) {
        os << vec3 << ", ";
    }
    return os;
}

BOOST_AUTO_TEST_SUITE(test_conjugate_gradient_method)

    BOOST_AUTO_TEST_CASE(test1) {

        // b
        std::vector<float> b = {1, 1, 1};

        // x - initial guess
        std::vector<float> x = {0, 0, 0};

        // Solve Ax = b
        conjugateResidualSolver(A, x, b, 2000, 0.001);

        BOOST_TEST(x[0] == 0.25);
        BOOST_TEST(x[1] == 0.25);
        BOOST_TEST(x[2] == 0.25);

    }

    BOOST_AUTO_TEST_CASE(testb) {

        // b
        std::vector<glm::vec3> b = {
                {1, 1, 1}
        };

        // x - initial guess
        std::vector<glm::vec3> x = {
                {0, 0, 0}
        };

        // Solve Ax = b
        conjugateResidualSolver(B, x, b, 2000, 0.001);

        BOOST_TEST(x[0][0] == 0.25);
        BOOST_TEST(x[0][1] == 0.25);
        BOOST_TEST(x[0][2] == 0.25);

    }

BOOST_AUTO_TEST_SUITE_END()
