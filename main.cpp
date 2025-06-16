//
//  main.cpp
//  sanctuary
//
//  Created by Hamidreza Azimian on 2025-06-14.
//


#include <iostream>
using namespace std;


class ThreeDofArm{
    public:
    //Constructor
    ThreeDofArm(double L1, double L2, double L3):L1(L1),L2(L2),L3(L3){};
    double L1, L2, L3;

    vector<double> solve_fk(vector<double> q){
        // Solves FK
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
        
        vector<double> X = {x3, y3, phi};
        return X;
        
    }

    vector<vector<double>> solve_ik(vector<double> X){
        // Solves IK

        double x3 = X[0];
        double y3 = X[1];
        double phi = X[2];
        // TODO: validate input pose values
        // assuming that it can be reached get wrist position
        double x2 = x3 - L3 * cos(phi);
        double y2 = y3 - L3 * sin(phi);

        // account for elbow up and down solutions
        double theta2_1 = acos((pow(x2,2) + pow(y2,2) - pow(L1,2) - pow(L2,2))/(2 * L1 * L2));
        double theta1_1 = atan2(y2,x2) - atan2(L2 * sin(theta2_1), L1 + L2 * cos(theta2_1));

        double theta2_2 = -1 * acos((pow(x2,2) + pow(y2,2) - pow(L1,2) - pow(L2,2))/(2 * L1 * L2));
        double theta1_2 = atan2(y2,x2) - atan2(L2 * sin(theta2_2), L1 + L2 * cos(theta2_2));

        double theta3_1 = phi - (theta1_1 + theta2_1);
        double theta3_2 = phi - (theta1_2 + theta2_2);

        vector<double> q1 = {theta1_1, theta2_1, theta3_1};
        vector<double> q2 = {theta1_2, theta2_2, theta3_2};
        vector<vector<double>> q_solutions = {q1, q2};
        return q_solutions;



    }

};

double get_error(vector<double> v1, vector<double> v2){
    // calculate sqr root of sum of squared error of two vectors of the same size
    // TODO: check if the sizes are equal
    double sq_e(0.0);
    for(int i=0; i< v1.size(); i++){
        sq_e = sq_e + (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }
    return sqrt(sq_e);
}

bool tests(double tol){
    //instantiate the robot
    ThreeDofArm arm = ThreeDofArm(0.3, 0.3, 0.1);
    // case 1
    vector<double> q({M_PI/3, -M_PI/6, 0});
    vector<double> X(arm.solve_fk(q));
    vector<vector<double>> q_solutions(arm.solve_ik(X));
    // return false if neither of solutions match
    if(get_error(q, q_solutions[0]) > tol && get_error(q, q_solutions[1]) > tol){
        return false;
    }
    // case 2
    q = {-M_PI/8, M_PI/4, M_PI/2};
    X = arm.solve_fk(q);
    q_solutions = arm.solve_ik(X);
    // return false if neither of solutions match
    if(get_error(q, q_solutions[0]) > 1e-3 && get_error(q, q_solutions[1]) > 1e-3){
        return false;
    }
    return true;
};



int main(int argc, const char * argv[]) {
    // iterates through all test cases and return true only if all are passed
    if(tests(1e-4)){
        cout << "All Tests Passed!" << std::endl;
    }
    else{
        cout << "At least One Test Failed!" << std::endl;
        }
    return 0;
}
