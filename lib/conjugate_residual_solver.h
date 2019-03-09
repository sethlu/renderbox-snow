#ifndef SNOW_CONJUGATERESIDUALSOLVER_H
#define SNOW_CONJUGATERESIDUALSOLVER_H


#include <vector>

#include "Vector.h"
#include "logging.h"


// Vector dot product
template<typename V>
V operator*(std::vector<V> const &a, std::vector<V> const &b) {
    LOG_ASSERT(a.size() == b.size());

    V result = {};
    for (size_t i = 0, n = a.size(); i < n; i++) {
        result += a[i] * b[i];
    }

    return result;
}

// Vector of vec3 dot product
float operator*(std::vector<glm::vec3> const &a, std::vector<glm::vec3> const &b) {
    LOG_ASSERT(a.size() == b.size());

    float result = {};
    for (size_t i = 0, n = a.size(); i < n; i++) {
        result += glm::dot(a[i], b[i]);
    }

    return result;
}

// Scalar multiply to vector
template<typename V>
std::vector<V> operator*(float a, std::vector<V> const &b) {

    std::vector<V> result(b.size());
    for (size_t i = 0, n = b.size(); i < n; i++) {
        result[i] = a * b[i];
    }

    return result;
}

// Vector addition
template<typename V>
std::vector<V> operator+(std::vector<V> const &a, std::vector<V> const &b) {
    LOG_ASSERT(a.size() == b.size());

    std::vector<V> result(a.size());
    for (size_t i = 0, n = a.size(); i < n; i++) {
        result[i] = a[i] + b[i];
    }

    return result;
}

// Vector subtraction
template<typename V>
std::vector<V> operator-(std::vector<V> const &a, std::vector<V> const &b) {
    LOG_ASSERT(a.size() == b.size());

    std::vector<V> result(a.size());
    for (size_t i = 0, n = a.size(); i < n; i++) {
        result[i] = a[i] - b[i];
    }

    return result;
}

// Vector tolerance check
template<typename V>
bool operator>(std::vector<V> const &a, float tolerance) {

    V result = {};
    for (size_t i = 0, n = a.size(); i < n; i++) {
        result += a[i] * a[i];
    }

    return result > tolerance * tolerance;
}

// Vector of vec3 tolerance check
bool operator>(std::vector<glm::vec3> const &a, float tolerance) {

    float result = {};
    for (size_t i = 0, n = a.size(); i < n; i++) {
        result += glm::dot(a[i], a[i]);
    }

    return result > tolerance * tolerance;
}

/**
 * Solves Ax = b
 * The initial guess is passed in as x
 * The result will be written in x
 */
template<typename V>
void conjugateResidualSolver(void (*A)(std::vector<V> &Ax, std::vector<V> const &x), std::vector<V> &x,
                             std::vector<V> const &b, unsigned int k, float tolerance) {
    std::vector<V> Ax(b.size());

    // Initial guess x

    // Calculate Ax
    A(Ax, x);

    auto r = b - Ax;
    auto p = r;

    std::vector<V> Ar(b.size());
    A(Ar, r);
    auto dot_r_Ar = r * Ar;

    std::vector<V> Ap(b.size());
    A(Ap, p);

    while (k-- > 0 && r > tolerance) {

        // r_k^T Ar_k
        auto dot_r_Ar_k = dot_r_Ar;

        // a_k
        auto a = dot_r_Ar_k / (Ap * Ap);

        // x_k+1
        x = x + a * p;

        // r_k+1
        r = r - a * Ap;

        // Ar_k+1
        A(Ar, r);
        dot_r_Ar = r * Ar;
        // b_k
        auto beta = dot_r_Ar / dot_r_Ar_k;

        // p_k+1
        p = r + beta * p;

        // Ap_k+1
        Ap = Ar + beta * Ap;

    }

}


#endif //SNOW_CONJUGATERESIDUALSOLVER_H
