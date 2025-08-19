#include <cmath>
#include <cstdio>
#include <iostream>
using namespace std;

// ---- geometry constants (set to match your robot) ----
// NOTE: Some references define Base/Effector as *circle radius*,
//       here we are using SIDE LENGTH of equilateral triangle.
double Base_Side     = 100.0;   // base triangle SIDE length (mm)
double Effector_Side = 45.0;    // effector triangle SIDE length (mm)
double r1            = 200.0;   // upper arm length (mm)
double L2            = 328.0;   // forearm length (mm)

// ---- globals ----
double X0, Y0, Z0;
double X1, Y1, Z1, X2, Y2, Z2, X3, Y3, Z3;
double Theta1, Theta2, Theta3;
double D, r2, YA, YB, YC, YD, ZB;
double a, b, a1, b1, c1;

// ---- routines ----
double inverse_kinematics_motor1()
{
    X1 = X0; Y1 = Y0; Z1 = Z0;

    r2 = sqrt(L2*L2 - X1*X1);
    YA = (-Base_Side / (sqrt(3.0) * 2.0));
    YC = (Y1 - (Effector_Side / (sqrt(3.0) * 2.0)));
    YD = (Y1 - (Effector_Side / (sqrt(3.0) * 2.0)));

    a = (r1*r1 - r2*r2 - YA*YA + YD*YD + Z1*Z1) / (2.0*Z1);
    b = (YA - YD) / Z1;

    a1 = (b*b + 1.0);
    b1 = (2.0*a*b - 2.0*YA);
    c1 = (a*a - r1*r1);

    D = 4.0 * (-2.0*a*b*YA + YA*YA + b*b*r1*r1 - a*a + r1*r1);

    if (D > 0.0) {
        YB = ((-b1) - sqrt(D)) / (2.0 * a1);
        ZB = a + b * YB;
        // atan2 fixes quadrant issue
        Theta1 = atan2(-ZB, (YA - YB)) * 180.0 / M_PI;
    } else {
        printf("Point does not exist\n");
    }
    return 0.0;
}

double inverse_kinematics_motor2()
{
    X2 = (X0 * (-0.5)) + (Y0 * sqrt(3.0) / 2.0);
    Y2 = (Y0 * (-0.5)) - (X0 * sqrt(3.0) / 2.0);
    Z2 = Z0;

    r2 = sqrt(L2*L2 - X2*X2);
    YA = (-Base_Side / (sqrt(3.0) * 2.0));
    YC = (Y2 - (Effector_Side / (sqrt(3.0) * 2.0)));
    YD = (Y2 - (Effector_Side / (sqrt(3.0) * 2.0)));

    a = (r1*r1 - r2*r2 - YA*YA + YD*YD + Z2*Z2) / (2.0*Z2);
    b = (YA - YD) / Z2;

    a1 = (b*b + 1.0);
    b1 = (2.0*a*b - 2.0*YA);
    c1 = (a*a - r1*r1);

    D = 4.0 * (-2.0*a*b*YA + YA*YA + b*b*r1*r1 - a*a + r1*r1);

    if (D > 0.0) {
        YB = ((-b1) - sqrt(D)) / (2.0 * a1);
        ZB = a + b * YB;
        Theta2 = atan2(-ZB, (YA - YB)) * 180.0 / M_PI;
    } else {
        printf("Point does not exist\n");
    }
    return 0.0;
}

double inverse_kinematics_motor3()
{
    X3 = (X0 * (-0.5)) - (Y0 * sqrt(3.0) / 2.0);
    Y3 = (Y0 * (-0.5)) + (X0 * sqrt(3.0) / 2.0);
    Z3 = Z0;

    r2 = sqrt(L2*L2 - X3*X3);
    YA = (-Base_Side / (sqrt(3.0) * 2.0));
    YC = (Y3 - (Effector_Side / (sqrt(3.0) * 2.0)));
    YD = (Y3 - (Effector_Side / (sqrt(3.0) * 2.0)));

    a = (r1*r1 - r2*r2 - YA*YA + YD*YD + Z3*Z3) / (2.0*Z3);
    b = (YA - YD) / Z3;

    a1 = (b*b + 1.0);
    b1 = (2.0*a*b - 2.0*YA);
    c1 = (a*a - r1*r1);

    D = 4.0 * (-2.0*a*b*YA + YA*YA + b*b*r1*r1 - a*a + r1*r1);

    if (D > 0.0) {
        YB = ((-b1) - sqrt(D)) / (2.0 * a1);
        ZB = a + b * YB;
        Theta3 = atan2(-ZB, (YA - YB)) * 180.0 / M_PI;
    } else {
        printf("Point does not exist\n");
    }
    return 0.0;
}

// ---- driver ----
int main()
{
    cout << "Enter X Y Z (mm), e.g. 0 0 -320: ";
    if (!(cin >> X0 >> Y0 >> Z0)) return 0;

    inverse_kinematics_motor1();
    inverse_kinematics_motor2();
    inverse_kinematics_motor3();

    cout << "Theta1: " << Theta1 << " deg\n";
    cout << "Theta2: " << Theta2 << " deg\n";
    cout << "Theta3: " << Theta3 << " deg\n";
    return 0;
}
