#include "sanctuary-pkg/three_dof_arm.hpp"

ThreeDofArm::ThreeDofArm(double L1, double L2, double L3)
    : L1(L1), L2(L2), L3(L3) {}

std::vector<double> ThreeDofArm::solve_fk(std::vector<double> q) {
    double theta1 = q[0];
    double theta2 = q[1];
    double theta3 = q[2];
    double x1 = L1 * cos(theta1);
    double y1 = L1 * sin(theta1);
    double x2 = x1 + L2 * cos(theta1 + theta2);
    double y2 = y1 + L2 * sin(theta1 + theta2);
    double x3 = x2 + L3 * cos(theta1 + theta2 + theta3);
    double y3 = y2 + L3 * sin(theta1 + theta2 + theta3);
    double phi = theta1 + theta2 + theta3;

    return {x3, y3, phi};
}

std::vector<std::vector<double>> ThreeDofArm::solve_ik(std::vector<double> X) {
    double x3 = X[0];
    double y3 = X[1];
    double phi = X[2];

    double x2 = x3 - L3 * cos(phi);
    double y2 = y3 - L3 * sin(phi);

    double c2 = (pow(x2, 2) + pow(y2, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2);
    double theta2_1 = acos(c2);
    double theta2_2 = -acos(c2);

    double theta1_1 = atan2(y2, x2) - atan2(L2 * sin(theta2_1), L1 + L2 * cos(theta2_1));
    double theta1_2 = atan2(y2, x2) - atan2(L2 * sin(theta2_2), L1 + L2 * cos(theta2_2));

    double theta3_1 = phi - (theta1_1 + theta2_1);
    double theta3_2 = phi - (theta1_2 + theta2_2);

    return {{theta1_1, theta2_1, theta3_1}, {theta1_2, theta2_2, theta3_2}};
}

