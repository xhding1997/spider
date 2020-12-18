#include "coordinates_ctrl.h"
#include "matrix.h"

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <sstream>
#include <unistd.h>

using namespace std;
using namespace Eigen;

//逆运动学,输入末端位置求关节角度2
extern double * CalculateCoordinatesToAngles(double Coordinate[3][6])
{
    //leg0
    a0 = 2 * L2 * (sqrt(Coordinate[0][0] * Coordinate[0][0] + Coordinate[1][0] * Coordinate[1][0]) - 0.890835 * L1);
    b0 = 2 * (Coordinate[2][0] + 0.454326 * L1) * L2;
    c0 = pow((sqrt(Coordinate[0][0] * Coordinate[0][0] + Coordinate[1][0] * Coordinate[1][0]) - 0.890835 * L1),2) + pow((Coordinate[2][0] + 0.454326 * L1),2) + pow(L2,2) - pow(L3,2);
    d0 = c0 / -sqrt(a0 * a0 + b0 * b0 - c0 * c0);
    e0 = -a0 / b0;

    Theta[0][0] = atan(Coordinate[1][0] / Coordinate[0][0]);
    Theta[1][0] = atan((d0 + e0)/(1 - d0 * e0));
    Theta[2][0] = acos((c0 - 2 * pow(L2,2))/(2 * L2 * L3));

    //leg1
    a1 = 2 * L2 * (sqrt(Coordinate[0][1] * Coordinate[0][1] + Coordinate[1][1] * Coordinate[1][1]) - 0.890835 * L1);
    b1 = 2 * (Coordinate[2][1] + 0.454326 * L1) * L2;
    c1 = pow((sqrt(Coordinate[0][1] * Coordinate[0][1] + Coordinate[1][1] * Coordinate[1][1]) - 0.890835 * L1),2) + pow((Coordinate[2][1] + 0.454326 * L1),2) + pow(L2,2) - pow(L3,2);
    d1 = c1 / -sqrt(a1 * a1 + b1 * b1 - c1 * c1);
    e1 = -a1 / b1;

    Theta[0][1] = atan(Coordinate[1][1] / Coordinate[0][1]);
    Theta[1][1] = atan((d1 + e1)/(1 - d1 * e1));
    Theta[2][1] = acos((c1 - 2 * pow(L2,2))/(2 * L2 * L3));
    //leg2
    a2 = 2 * L2 * (sqrt(Coordinate[0][2] * Coordinate[0][2] + Coordinate[1][2] * Coordinate[1][2]) - 0.890835 * L1);
    b2 = 2 * (Coordinate[2][2] + 0.454326 * L1) * L2;
    c2 = pow((sqrt(Coordinate[0][2] * Coordinate[0][2] + Coordinate[1][2] * Coordinate[1][2]) - 0.890835 * L1),2) + pow((Coordinate[2][2] + 0.454326 * L1),2) + pow(L2,2) - pow(L3,2);
    d2 = c2 / -sqrt(a2 * a2 + b2 * b2 - c2 * c2);
    e2 = -a2 / b2;

    Theta[0][2] = atan(Coordinate[1][2] / Coordinate[0][2]);
    Theta[1][2] = atan((d2 + e2)/(1 - d2 * e2));
    Theta[2][2] = acos((c2 - 2 * pow(L2,2))/(2 * L2 * L3));
    //leg3
    a3 = 2 * L2 * (sqrt(Coordinate[0][3] * Coordinate[0][3] + Coordinate[1][3] * Coordinate[1][3]) - 0.890835 * L1);
    b3 = 2 * (Coordinate[2][3] + 0.454326 * L1) * L2;
    c3 = pow((sqrt(Coordinate[0][3] * Coordinate[0][3] + Coordinate[1][3] * Coordinate[1][3]) - 0.890835 * L1),2) + pow((Coordinate[2][3] + 0.454326 * L1),2) + pow(L2,2) - pow(L3,2);
    d3 = c3 / -sqrt(a3 * a3 + b3 * b3 - c3 * c3);
    e3 = -a3 / b3;

    Theta[0][3] = atan(Coordinate[1][3] / Coordinate[0][3]);
    Theta[1][3] = atan((d3 + e3)/(1 - d3 * e3));
    Theta[2][3] = acos((c3 - 2 * pow(L2,2))/(2 * L2 * L3));
    //leg4
    a4 = 2 * L2 * (sqrt(Coordinate[0][4] * Coordinate[0][4] + Coordinate[1][4] * Coordinate[1][4]) - 0.890835 * L1);
    b4 = 2 * (Coordinate[2][4] + 0.454326 * L1) * L2;
    c4 = pow((sqrt(Coordinate[0][4] * Coordinate[0][4] + Coordinate[1][4] * Coordinate[1][4]) - 0.890835 * L1),2) + pow((Coordinate[2][4] + 0.454326 * L1),2) + pow(L2,2) - pow(L3,2);
    d4 = c4 / -sqrt(a4 * a4 + b4 * b4 - c4 * c4);
    e4 = -a4 / b4;

    Theta[0][4] = atan(Coordinate[1][4] / Coordinate[0][4]);
    Theta[1][4] = atan((d4 + e4)/(1 - d4 * e4));
    Theta[2][4] = acos((c4 - 2 * pow(L2,2))/(2 * L2 * L3));
    //leg5
    a5 = 2 * L2 * (sqrt(Coordinate[0][5] * Coordinate[0][5] + Coordinate[1][5] * Coordinate[1][5]) - 0.890835 * L1);
    b5 = 2 * (Coordinate[2][5] + 0.454326 * L1) * L2;
    c5 = pow((sqrt(Coordinate[0][5] * Coordinate[0][5] + Coordinate[1][5] * Coordinate[1][5]) - 0.890835 * L1),2) + pow((Coordinate[2][5] + 0.454326 * L1),2) + pow(L2,2) - pow(L3,2);
    d5 = c5 / -sqrt(a5 * a5 + b5 * b5 - c5 * c5);
    e5 = -a5 / b5;

    Theta[0][5] = atan(Coordinate[1][5] / Coordinate[0][5]);
    Theta[1][5] = atan((d5 + e5)/(1 - d5 * e5));
    Theta[2][5] = acos((c5 - 2 * pow(L2,2))/(2 * L2 * L3));

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            Angle[i][j] = 180 * Theta[i][j] / M_PI ;
        }
    }

    for (int j = 0; j < 6; ++j) {
        if(Angle[2][j] > 0 )
        {
            Angle[2][j] = -Angle[2][j];
        }
    }

//输出角度值（Angle[3][6]）
    cout << "Angle:" << endl ;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            cout << Angle[i][j] << " " ;
        }
        cout << endl ;
    }
    cout << endl ;
}

//六条腿to坐标&动作指令2
extern string MoveLegsToCoordinates(double Coordinate[3][6])
{
//Angle to P
    CalculateCoordinatesToAngles(Coordinate);

    for (int j = 0; j < 6; ++j) {
        Pnum[0][j] = round((Angle[0][j] + 90) * 2000/180) + 500;
    }
    for (int j = 0; j < 6; ++j) {
        if (Angle[1][j] < 0.01)
            Pnum[1][j] = round((abs(Angle[1][j]) + 90) * 2000/180) + 500;
        else
            Pnum[1][j] = round((Angle[1][j]) * 2000/180) + 500;
    }
    for (int j = 0; j < 6; ++j) {
        Pnum[2][j] = round((abs(Angle[2][j] - 45)) * 2000/180) + 500;
    }

//    cout << "Pnum(p number):" << endl ;
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 6; ++j) {
//            cout << Pnum[i][j] << " " ;
//        }
//        cout << endl ;
//    }
//    cout << endl ;

    string Pcmd[3][6] = {{"0","3","6","9","12","15"},{"1","4","7","10","13","16"},{"2","5","8","11","14","17"}};
    stringstream q;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            q << "P" << Pnum[i][j];
            Pcmd[i][j] = q.str();
            q.str("");
        }
    }
//    cout << "Pcmd(P string):" << endl ;
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 6; ++j) {
//            cout << Pcmd[i][j] << " " ;
//        }
//        cout << endl ;
//    }
    string T = "T0500";
    string Vcmd[3][6] = {{"V000","V003","V006","V009","V012","V015"},
                         {"V001","V004","V007","V010","V013","V016"},
                         {"V002","V005","V008","V011","V014","V017"}};
    string VPT[3][6] = {{"V000P1500T1000","V003P1500T1000","V006P1500T1000","V009P1500T1000","V012P1500T1000","V015P1500T1000"},
                        {"V001P2000T1000","V004P2000T1000","V007P2000T1000","V010P2000T1000","V013P2000T1000","V016P2000T1000"},
                        {"V002P2000T1000","V005P2000T1000","V008P2000T1000","V011P2000T1000","V014P2000T1000","V017P2000T1000"}};
    cout << "VP(VP string):" << endl ;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            VPT[i][j] = Vcmd[i][j] + Pcmd[i][j];
            cout << VPT[i][j] << " " ;
        }
        cout << endl ;
    }
    string CMD;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            //CMD = CMD + VPT[i][j];
            CMD.append(VPT[i][j]);
        }
    }
    CMD = CMD + T;
    cout << endl << CMD << endl ;
    return CMD;
}

//单腿to坐标2
extern void MoveA_legToCoordinate(int LegNumber , double LegCoordinate[3][1])
{
    if (LegNumber < 0 or LegNumber >5)
        cout << "The wrong leg number was entered!" << endl;
    else
        for (int i = 0; i < 3; ++i)
        {
            Coordinate[i][LegNumber] = LegCoordinate[i][0];
        }
//    cout << "Angle:" << endl ;
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 6; ++j) {
//            cout << Angle[i][j] << " " ;
//        }
//        cout << endl ;
//    }
    cout << endl ;
}
