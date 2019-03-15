#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE snowlib

#include <boost/test/unit_test_suite.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>
#include <ostream>

namespace tt = boost::test_tools;

#include "../lib/conjugate_residual_solver.h"
#include "../lib/SnowSolver.h"


// A[3x3]
void A(std::vector<double> &Ax, std::vector<double> const &x) {
    Ax[0] = 2 * x[0] + x[1] + x[2];
    Ax[1] = x[0] + 2 * x[1] + x[2];
    Ax[2] = x[0] + x[1] + 2 * x[2];
}

// B[3x3]
void B(std::vector<glm::dvec3> &Ax, std::vector<glm::dvec3> const &x) {
    Ax[0] = glm::dmat3(2, 1, 1, 1, 2, 1, 1, 1, 2) * x[0];
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
        std::vector<double> b = {1, 1, 1};

        // x - initial guess
        std::vector<double> x = {0, 0, 0};

        // Solve Ax = b
        conjugateResidualSolver(A, x, b, 2000, 0.001);

        BOOST_TEST(x[0] == 0.25);
        BOOST_TEST(x[1] == 0.25);
        BOOST_TEST(x[2] == 0.25);

    }

    BOOST_AUTO_TEST_CASE(testb) {

        // b
        std::vector<glm::dvec3> b = {
                {1, 1, 1}
        };

        // x - initial guess
        std::vector<glm::dvec3> x = {
                {0, 0, 0}
        };

        // Solve Ax = b
        conjugateResidualSolver(B, x, b, 2000, 0.001);

        BOOST_TEST(x[0][0] == 0.25);
        BOOST_TEST(x[0][1] == 0.25);
        BOOST_TEST(x[0][2] == 0.25);

    }

BOOST_AUTO_TEST_SUITE_END()


BOOST_AUTO_TEST_SUITE(test_n)

    BOOST_AUTO_TEST_CASE(test1) {

        auto n0 = SnowSolver::n(0);
        auto n0p5 = SnowSolver::n(0.5);
        auto n1 = SnowSolver::n(1);
        auto n1p5 = SnowSolver::n(1.5);
        auto n2 = SnowSolver::n(2);
        auto n2p5 = SnowSolver::n(2.5);

        std::cout << "n0=" << n0 << std::endl;
        std::cout << "n0p5=" << n0p5 << std::endl;
        std::cout << "n1=" << n1 << std::endl;
        std::cout << "n1p5=" << n1p5 << std::endl;
        std::cout << "n2=" << n2 << std::endl;
        std::cout << "n2p5=" << n2p5 << std::endl;

        BOOST_TEST(SnowSolver::n(-0.5) == n0p5);
        BOOST_TEST(SnowSolver::n(-1) == n1);
        BOOST_TEST(SnowSolver::n(-1.5) == n1p5);
        BOOST_TEST(SnowSolver::n(-2) == n2);
        BOOST_TEST(SnowSolver::n(-2.5) == n2p5);

    }

    BOOST_AUTO_TEST_CASE(test2) {

        auto n0 = SnowSolver::del_n(0);
        auto n0p5 = SnowSolver::del_n(0.5);
        auto n1 = SnowSolver::del_n(1);
        auto n1p5 = SnowSolver::del_n(1.5);
        auto n2 = SnowSolver::del_n(2);
        auto n2p5 = SnowSolver::del_n(2.5);

        std::cout << "n0=" << n0 << std::endl;
        std::cout << "n0p5=" << n0p5 << std::endl;
        std::cout << "n1=" << n1 << std::endl;
        std::cout << "n1p5=" << n1p5 << std::endl;
        std::cout << "n2=" << n2 << std::endl;
        std::cout << "n2p5=" << n2p5 << std::endl;

        BOOST_TEST(SnowSolver::del_n(-2) == 0);
        BOOST_TEST(SnowSolver::del_n(-1.5) > 0);
        BOOST_TEST(SnowSolver::del_n(-1) > 0);
        BOOST_TEST(SnowSolver::del_n(-0.5) > 0);
        BOOST_TEST(SnowSolver::del_n(0) == 0);
        BOOST_TEST(SnowSolver::del_n(0.5) < 0);
        BOOST_TEST(SnowSolver::del_n(1) < 0);
        BOOST_TEST(SnowSolver::del_n(1.5) < 0);
        BOOST_TEST(SnowSolver::del_n(2) == 0);

        BOOST_TEST(SnowSolver::del_n(-0.5) == -n0p5);
        BOOST_TEST(SnowSolver::del_n(-1) == -n1);
        BOOST_TEST(SnowSolver::del_n(-1.5) == -n1p5);
        BOOST_TEST(SnowSolver::del_n(-2) == -n2);
        BOOST_TEST(SnowSolver::del_n(-2.5) == -n2p5);

    }


BOOST_AUTO_TEST_SUITE_END()
