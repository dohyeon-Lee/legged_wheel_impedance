#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <glm/glm.hpp>
using namespace std;
using namespace glm;
class controller
{
private:
    double l1;
    double l2;
    double gamma;

    double kpr;
    double kdr;
    double kpt;
    double kdt;

    double dr;
    double drd;
    double dt;
    double dtd;

    mat2 jacobian(double theta2);
public:
    controller();
    vec2 leg2spring(double theta1, double theta2, double omega1, double omega2);
    vec2 vec2product(mat2 J, vec2 w);
};

