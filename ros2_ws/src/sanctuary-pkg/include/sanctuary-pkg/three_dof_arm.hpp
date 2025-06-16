#pragma once

#include <vector>
#include <cmath>

class ThreeDofArm {
public:
    ThreeDofArm(double L1, double L2, double L3);
    std::vector<double> solve_fk(std::vector<double> q);
    std::vector<std::vector<double>> solve_ik(std::vector<double> X);

private:
    double L1, L2, L3;
};

// Utility functions
double get_error(std::vector<double> v1, std::vector<double> v2);
bool tests(double tol);
