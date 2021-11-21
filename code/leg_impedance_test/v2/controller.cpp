#include "controller.h"
#include <cstdio>

// typedef Matrix<double, 2, 2> Matrix2d;
// typedef Matrix<double, 2, 1> Vector2d;
// typedef Matrix<double, 1, 2> RowVector2d;

Controller::Controller()
{
    m1 = 1.207;
    m2 = 0.706;
    I1 = 1.20959;
    I2 = 0.362747;
    c1 = 0.1235;
    c2 = 0.03856;

    l1 = 0.2;
    l2 = 0.22;
    gamma = 0.001;

    kpr = 1;
    kpt = 1;
    kdr = 0;//0.025;
    kdt = 0;//0.025;

    dr = 0.4;
    drd = 0;
    dt = 0;
    dtd = 0;

    theta1 = 0;
    theta2 = 0;
    omega1 = 0;
    omega2 = 0;
}

Matrix2d Controller::jacobian(double theta2_)
{
    double Jr11 = 0;
    double Jr12 = -(l1 * l2 * sin(theta2_)) / pow((pow(l1, 2) + pow(l2, 2) + 2 * l1 * l2 * cos(theta2_)), 0.5);
    double Jt11 = 1;
    double Jt12 = (l2 * (l2 + l1 * cos(theta2_))) / (pow(l1, 2) + pow(l2, 2) + 2 * l1 * l2 * cos(theta2_));

    RowVector2d J_r(Jr11, Jr12);
    RowVector2d J_t(Jt11, Jt12);

    Matrix2d J;
    J.row(0) = J_r;
    J.row(1) = J_t;

    return J;
}

Vector2d Controller::leg2spring(double theta1_, double theta2_, double omega1_, double omega2_)
{
    // update theta and omega
    theta1 = theta1_;
    theta2 = theta2_;
    omega1 = omega1_;
    omega2 = omega2_;

    // get jacobian matrix
    Matrix2d J = jacobian(theta2);

    // update M, C, G matrix
    getMCG();

    //damped Jacobian
    Matrix2d damped_eye = gamma * MatrixXd::Identity(2, 2);
    J = J + damped_eye;

    double r = pow((pow(l1, 2) + pow(l2, 2) + 2 * l1 * l2 * cos(theta2)), 0.5);
    double theta = M_PI / 2 + atan2(l2 * sin((3 * M_PI) / 2 + theta1 + theta2) + l1 * sin((3 * M_PI) / 2 + theta1), l2 * cos((3 * M_PI) / 2 + theta1 + theta2) + l1 * cos((3 * M_PI) / 2 + theta1));
    Vector2d omega(omega1, omega2);
    Vector2d qdot = J * omega;
    double rdot = qdot.coeff(0, 0);
    double thetadot = qdot.coeff(1, 0);

    double F_r = kpr * (dr - r) + kdr * (drd - rdot);
    double F_t = kpt * (dt - theta) + kdt * (dtd - thetadot);

    Vector2d F(F_r, F_t);
    //cout << F_r << "," << F_t<< endl;

    // tau = J^T * F
    Vector2d tau = J.transpose() * F + G + C;
    /*printf("tj0x : %15f, tj0y : %15f, tj1y : %15f, tj1y : %15f", transpose_J[0].x, transpose_J[0].y, transpose_J[1].x, transpose_J[1].y);
    printf("tj0x : %15f, tj0y : %15f, tj1y : %15f, tj1y : %15f", J[0].x, J[0].y, J[1].x, J[1].y);
    printf("rt : %15f %15f\n", r, theta);*/
    //printf("torque : %15f %15f\n", tau[0], tau[1]);
   /* printf("angle : %15f %15f\n", theta1, theta2);
    printf("force : %15f %15f\n", F_r, F_t);*/
    /* cout << "1 : " << r << "                                      2 : " << theta << endl;
    cout << "1 : " << tau[0] << "                                      2 : " << tau[1] << endl;*/
    
    return tau;
}

void Controller::getMCG()
{
    double M11 = I1 + I2 + (m2 * (2 * pow((c2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + l1 * cos(theta1)), 2) + 2 * pow((c2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + l1 * sin(theta1)), 2))) / 2 + (m1 * (2 * pow(c1, 2) * pow(cos(theta1), 2) + 2 * pow(c1, 2) * pow(sin(theta1), 2))) / 2;
    double M12 = I2 + (m2 * (2 * c2 * (c2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + l1 * cos(theta1)) * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + 2 * c2 * (c2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + l1 * sin(theta1)) * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)))) / 2;
    double M21 = I2 + (m2 * (2 * c2 * (c2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + l1 * cos(theta1)) * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + 2 * c2 * (c2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + l1 * sin(theta1)) * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)))) / 2;
    double M22 = I2 + (m2 * (2 * pow(c2, 2) * pow((cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)), 2) + 2 * pow(c2, 2) * pow((cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)), 2))) / 2;
    double C1 = -(m2 * omega2 * (2 * (c2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + l1 * cos(theta1)) * (c2 * omega1 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + c2 * omega2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1))) - 2 * (c2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + l1 * sin(theta1)) * (c2 * omega1 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + c2 * omega2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2))) + 2 * c2 * (omega1 * (c2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + l1 * cos(theta1)) + c2 * omega2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2))) * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) - 2 * c2 * (omega1 * (c2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + l1 * sin(theta1)) + c2 * omega2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1))) * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)))) / 2;
    double C2 = (m2 * (2 * (omega1 * (c2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + l1 * cos(theta1)) + c2 * omega2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2))) * (c2 * omega1 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + c2 * omega2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1))) - 2 * (omega1 * (c2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + l1 * sin(theta1)) + c2 * omega2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1))) * (c2 * omega1 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + c2 * omega2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2))))) / 2 - (m2 * omega2 * (2 * c2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) * (c2 * omega1 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + c2 * omega2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1))) - 2 * c2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) * (c2 * omega1 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + c2 * omega2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2))) + 2 * c2 * (omega1 * (c2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + l1 * cos(theta1)) + c2 * omega2 * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2))) * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) - 2 * c2 * (omega1 * (c2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + l1 * sin(theta1)) + c2 * omega2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1))) * (cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)))) / 2;
    double G1 = g * m2 * (c2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + l1 * sin(theta1)) + c1 * g * m1 * sin(theta1);
    double G2 = c2 * g * m2 * (cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1));

    M << M11, M12,
        M21, M22;
    C << C1, C2;
    G << G1, G2;
}