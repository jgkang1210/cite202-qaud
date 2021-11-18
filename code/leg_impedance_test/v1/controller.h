#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 2, 2> Matrix2d;
typedef Matrix<double, 2, 1> Vector2d;
typedef Matrix<double, 1, 2> RowVector2d;


class Controller
{
private:
    // mass, moment of inertia, position of COM
    double m1;
    double m2;
    double I1;
    double I2;
    double c1;
    double c2;

    // length of leg
    double l1 = 0.2;
    double l2 = 0.22;

    // numerical stability
    double gamma = 0.001;

    // spring constants, damping ratio
    double kpr = 2;
    double kdr = 2;
    double kpt = 0.12;//0.025;
    double kdt = 0.12;//0.025;

    // desired set point of r and theta
    double dr = 0.4;
    double drd = 0;
    double dt = 0;
    double dtd = 0;

    // gravity
    double g = 9.81;

    // current data
    double theta1;
    double theta2;
    double omega1;
    double omega2;

    Matrix2d M;
    Vector2d C;
    Vector2d G;

    Matrix2d jacobian(double theta2);
public:
    Controller();
    Vector2d leg2spring(double theta1, double theta2, double omega1, double omega2);
    void getMCG();
};

