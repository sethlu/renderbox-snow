#ifndef SNOW_COMMON_H
#define SNOW_COMMON_H


static std::unique_ptr<SnowSolver> solver;

static std::unique_ptr<SnowSolver> ghostSolver; // Alternative solver for diffing purposes


inline double randNumber(double lo, double hi) {
    return lo + rand() / (RAND_MAX / (hi - lo));
}


#endif //SNOW_COMMON_H
