#include "angle_ctrl.h"
#include "matrix.h"

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <sstream>
#include <unistd.h>

using namespace std;
using namespace Eigen;

//运动学,输入关节角度求末端位置1
extern double * CalculateAnglesToCoordinates(double Angle[3][6])
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            Theta[i][j] = Angle[i][j] * M_PI / 180;
        }
    }
//leg0
    Coordinate[0][0] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * cos(Theta[0][0]);
    Coordinate[1][0] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * sin(Theta[0][0]);
    Coordinate[2][0] = -0.454326 * L1 + L2 * sin(Theta[1][0]) + L3 * sin(Theta[1][0] + Theta[2][0]);
//leg1
    Coordinate[0][1] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * cos(Theta[0][0]);
    Coordinate[1][1] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * sin(Theta[0][0]);
    Coordinate[2][1] = -0.454326 * L1 + L2 * sin(Theta[1][0]) + L3 * sin(Theta[1][0] + Theta[2][0]);
//leg2
    Coordinate[0][2] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * cos(Theta[0][0]);
    Coordinate[1][2] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * sin(Theta[0][0]);
    Coordinate[2][2] = -0.454326 * L1 + L2 * sin(Theta[1][0]) + L3 * sin(Theta[1][0] + Theta[2][0]);
//leg3
    Coordinate[0][3] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * cos(Theta[0][0]);
    Coordinate[1][3] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * sin(Theta[0][0]);
    Coordinate[2][3] = -0.454326 * L1 + L2 * sin(Theta[1][0]) + L3 * sin(Theta[1][0] + Theta[2][0]);
//leg4
    Coordinate[0][4] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * cos(Theta[0][0]);
    Coordinate[1][4] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * sin(Theta[0][0]);
    Coordinate[2][4] = -0.454326 * L1 + L2 * sin(Theta[1][0]) + L3 * sin(Theta[1][0] + Theta[2][0]);
//leg5
    Coordinate[0][5] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * cos(Theta[0][0]);
    Coordinate[1][5] = (0.890835 * L1 + L2 * cos(Theta[1][0]) + L3 * cos(Theta[1][0] + Theta[2][0])) * sin(Theta[0][0]);
    Coordinate[2][5] = -0.454326 * L1 + L2 * sin(Theta[1][0]) + L3 * sin(Theta[1][0] + Theta[2][0]);

//输出坐标值Coordinate[3][6]
//    cout << "Coordinate:" << endl ;
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 6; ++j) {
//            cout << Coordinate[i][j] << " " ;
//        }
//        cout << endl ;
//    }
//    cout << endl ;
}

//六条腿to角度&动作指令1
extern string MoveLegsToAngles(double Angle[3][6])
{
//Angle to P
    for (int j = 0; j < 6; ++j) {
        Pnum[0][j] = round((Angle[0][j] + 90) * 2000/180) + 500;
    }
    for (int j = 0; j < 6; ++j) {
        if (Angle[1][j] < 0.1)
            Pnum[1][j] = round((abs(Angle[1][j]) + 90) * 2000/180) + 500;
        else
            Pnum[1][j] = round((Angle[1][j]) * 2000/180) + 500;
    }
    for (int j = 0; j < 6; ++j) {
        Pnum[2][j] = round((abs(Angle[2][j] - 45)) * 2000/180) + 500;
    }

    string Pcmd[3][6] = {{"0","3","6","9","12","15"},{"1","4","7","10","13","16"},{"2","5","8","11","14","17"}};
    stringstream q;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            q << "P" << Pnum[i][j];
            Pcmd[i][j] = q.str();
            q.str("");
        }
    }

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

//单腿to角度1
extern void MoveA_legToAngle(int LegNumber , double LegAngle[3][1])
{
    if (LegNumber < 0 or LegNumber >5)
        cout << "The wrong leg number was entered!" << endl;
    else
        for (int i = 0; i < 3; ++i)
        {
            Angle[i][LegNumber] = LegAngle[i][0];
        }
    cout << "Angle:" << endl ;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            cout << Angle[i][j] << " " ;
        }
        cout << endl ;
    }
    cout << endl ;
}
