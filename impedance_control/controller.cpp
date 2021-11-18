#include "controller.h"
#include <cstdio>
controller::controller()
{
    l1 = 0.2;
    l2 = 0.22;
    gamma = 0.001;


    kpr = 2;
    kpt = 2;
    kdr = 0.12;//0.025;
    kdt = 0.12;//0.025;

    dr = 0.4;
    drd = 0;
    dt = 0;
    dtd = 0;
}
mat2 controller::jacobian(double theta2)
{
    double Jr11 = 0;
    double Jr12 = -(l1 * l2 * sin(theta2)) / pow((pow(l1, 2) + pow(l2, 2) + 2 * l1 * l2 * cos(theta2)), 0.5);
    double Jt11 = 1;
    double Jt12 = (l2 * (l2 + l1 * cos(theta2))) / (pow(l1, 2) + pow(l2, 2) + 2 * l1 * l2 * cos(theta2));
    vec2 J_r(Jr11, Jr12);
    vec2 J_t(Jt11, Jt12);
    mat2 J(1.0f);
    J[0] = J_r;
    J[1] = J_t;
    return J;
}
vec2 controller::leg2spring(double theta1, double theta2, double omega1, double omega2)
{
    mat2 J = jacobian(theta2);
    //damped Jacobian
    mat2 eye(gamma);
    J = J + eye;
    double r = pow((pow(l1, 2) + pow(l2, 2) + 2 * l1 * l2 * cos(theta2)), 0.5);
    double theta = M_PI / 2 + atan2(l2 * sin((3 * M_PI) / 2 + theta1 + theta2) + l1 * sin((3 * M_PI) / 2 + theta1), l2 * cos((3 * M_PI) / 2 + theta1 + theta2) + l1 * cos((3 * M_PI) / 2 + theta1));
    /*cout << "r: " << r << endl;
    cout << "t: " << theta << endl;*/
    vec2 omega(omega1, omega2);
    vec2 qdot = vec2product(J, omega);
    double rdot = qdot[0];
    double thetadot = qdot[1];
    
    double F_r = kpr * (dr - r) + kdr * (drd - rdot);
    double F_t = kpt * (dt - theta) + kdt * (dtd - thetadot);
 
    vec2 F(F_r, F_t);
    //cout << F_r << "," << F_t<< endl;
    //transpose
    mat2 transpose_J(1.0f);
    transpose_J[0].x = J[0].x;
    transpose_J[0].y = J[1].x;
    transpose_J[1].x = J[0].y;
    transpose_J[1].y = J[1].y;

    vec2 tau = vec2product(transpose_J, F);
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

vec2 controller::vec2product(mat2 J, vec2 w)
{
    double vec11 = J[0].x * w.x + J[0].y * w.y;
    double vec22 = J[1].x * w.x + J[1].y * w.y;

    vec2 vec(vec11, vec22);

    return vec;
}