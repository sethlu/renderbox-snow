#ifndef SNOW_COMMON_H
#define SNOW_COMMON_H


static std::unique_ptr<SnowSolver> solver;


inline double randNumber(double lo, double hi) {
    return lo + rand() / (RAND_MAX / (hi - lo));
}


#endif //SNOW_COMMON_H
