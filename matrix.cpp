#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <sstream>
#include <unistd.h>

#include "matrix.h"
#include "angle_ctrl.h"
#include "coordinates_ctrl.h"
#include "reachable.h"

using namespace Eigen;
using namespace std;

double L1 = 56.13, L2 = 89.91, L3 = 154.90;
double Coordinate[3][6] = {{139.913,139.913,139.913,139.913,139.913,139.913},{0,0,0,0,0,0},{-180.401,-180.401,-180.401,-180.401,-180.401,-180.401}};
double Theta[3][6] = {0};
double a0,a1,a2,a3,a4,a5, b0,b1,b2,b3,b4,b5, c0,c1,c2,c3,c4,c5, d0,d1,d2,d3,d4,d5, e0,e1,e2,e3,e4,e5;
double Angle[3][6] = {{0,0,0,0,0,0},{0,0,0,0,0,0},{-90,-90,-90,-90,-90,-90}};
//double * CalculateAnglesToCoordinates(double Angle[3][6]);
//double * CalculateCoordinatesToAngles(double Coordinate[3][6]);
//double IfReachable(double Coordinate[3][6]);
//string MoveLegsToCoordinates(double Coordinate[3][6]);
//string MoveLegsToAngles(double Angle[3][6]);
double Pnum[3][6] = {{1500,1500,1500,1500,1500,1500},{1500,1500,1500,1500,1500,1500},{2000,2000,2000,2000,2000,2000}};
//void MoveA_legToAngle(int LegNumber , double LegAngle[3][1]);
//void MoveA_legToCoordinate(int LegNumber , double LegCoordinate[3][1]);

//double FindCloseReachable(double coordinates[]);
//double coordinates[3] = {123,456,789};
double CloseCoordinates[3] = {0};

string cmd_body()
{
//角度to坐标
//    Angle[0][0] = 0;   Angle[0][1] = 0; Angle[0][2] = 0;   Angle[0][3] = 0; Angle[0][4] = 0; Angle[0][5] = 0;
//    Angle[1][0] = 0;   Angle[1][1] = 0; Angle[1][2] = 0;   Angle[1][3] = 0; Angle[1][4] = 0; Angle[1][5] = 0;
//    Angle[2][0] = -90; Angle[2][1] = 0; Angle[2][2] = -90; Angle[2][3] = 0; Angle[2][4] = 0; Angle[2][5] = 0;
//    CalculateAnglesToCoordinates(Angle);
//
//    cout << "Coordinate:" << endl ;
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 6; ++j) {
//            cout << Coordinate[i][j] << " " ;
//        }
//        cout << endl ;
//    }
//    cout << endl ;

//坐标to角度
//    Coordinate[0][0] = 0;Coordinate[0][1] = 0;Coordinate[0][2] = 0;Coordinate[0][3] = 0;Coordinate[0][4] = 0;Coordinate[0][5] = 0;
//    Coordinate[1][0] = 0;Coordinate[1][1] = 0;Coordinate[1][2] = 0;Coordinate[1][3] = 0;Coordinate[1][4] = 0;Coordinate[1][5] = 0;
//    Coordinate[2][0] = 0;Coordinate[2][1] = 0;Coordinate[2][2] = 0;Coordinate[2][3] = 0;Coordinate[2][4] = 0;Coordinate[2][5] = 0;
//    CalculateCoordinatesToAngles(Coordinate);
//
//    cout << "Angle:" << endl ;
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 6; ++j) {
//            cout << Angle[i][j] << " " ;
//        }
//        cout << endl ;
//    }
//    cout << endl ;

//判断是否可达
//    Coordinate[0][0] = 0;Coordinate[0][1] = 0;Coordinate[0][2] = 0;Coordinate[0][3] = 0;Coordinate[0][4] = 0;Coordinate[0][5] = 0;
//    Coordinate[1][0] = 0;Coordinate[1][1] = 0;Coordinate[1][2] = 0;Coordinate[1][3] = 0;Coordinate[1][4] = 0;Coordinate[1][5] = 0;
//    Coordinate[2][0] = 0;Coordinate[2][1] = 0;Coordinate[2][2] = 0;Coordinate[2][3] = 0;Coordinate[2][4] = 0;Coordinate[2][5] = 0;
//    IfReachable(Coordinate);

//坐标to动作指令
//0号腿 0 0 -45
//    Coordinate[0][0] = 249.443;Coordinate[0][1] = 0;Coordinate[0][2] = 0;Coordinate[0][3] = 0;Coordinate[0][4] = 0;Coordinate[0][5] = 0;
//    Coordinate[1][0] = 0;Coordinate[1][1] = 0;Coordinate[1][2] = 0;Coordinate[1][3] = 0;Coordinate[1][4] = 0;Coordinate[1][5] = 0;
//    Coordinate[2][0] = -135.032;Coordinate[2][1] = 0;Coordinate[2][2] = 0;Coordinate[2][3] = 0;Coordinate[2][4] = 0;Coordinate[2][5] = 0;
//0号腿 0 0 -90
//    Coordinate[0][0] = 139.913;Coordinate[0][1] = 0;Coordinate[0][2] = 0;Coordinate[0][3] = 0;Coordinate[0][4] = 0;Coordinate[0][5] = 0;
//    Coordinate[1][0] = 0;Coordinate[1][1] = 0;Coordinate[1][2] = 0;Coordinate[1][3] = 0;Coordinate[1][4] = 0;Coordinate[1][5] = 0;
//    Coordinate[2][0] = -180.401;Coordinate[2][1] = 0;Coordinate[2][2] = 0;Coordinate[2][3] = 0;Coordinate[2][4] = 0;Coordinate[2][5] = 0;
//    return MoveLegsToCoordinates(Coordinate);

//角度to动作指令
//    Angle[0][0] = -60; Angle[0][1] = 0; Angle[0][2] = 0; Angle[0][3] = 0; Angle[0][4] = 0; Angle[0][5] = 0;
//    Angle[1][0] = -45; Angle[1][1] = 0; Angle[1][2] = 0; Angle[1][3] = 0; Angle[1][4] = 0; Angle[1][5] = 0;
//    Angle[2][0] = -90; Angle[2][1] = 0; Angle[2][2] = 0; Angle[2][3] = 0; Angle[2][4] = 0; Angle[2][5] = 0;
//    return MoveLegsToAngles(Angle);

//单腿角度更新&动作指令输出
//    double LegAngle[3][1] = {{60},{45},{-45}};
//    MoveA_legToAngle(2,LegAngle);
//    return MoveLegsToAngles(Angle);
//    sleep(3);
//    LegAngle[0][0] = 60;LegAngle[0][0] = 45;LegAngle[0][0] = -45;
//    MoveA_legToAngle(5,LegAngle);
//    MoveLegsToAngles(Angle);

//单腿坐标更新&动作指令输出
//    double LegCoordinate[3][1] = {{249.443},{0},{-135.032}};
//    MoveA_legToCoordinate(5,LegCoordinate);
//    return MoveLegsToCoordinates(Coordinate);

//寻找最近点
//在大球外的一些点（3,30,-400）,（25,0,-400）,（3,-30,-400）,（3,80,-400）,（3,80,500）,（500,0,400）
//在大球内的一些点（3,30,100）,（25,0,-100）,（3,-30,-100）,（3,-80,20）,（3,-80,-20）
    double TestCoordinates[3]={25,0,-400};
    FindCloseReachable(TestCoordinates);
    double LegCoordinate[3][1] = {{CloseCoordinates[0]},{CloseCoordinates[1]},{CloseCoordinates[2]}};
    cout << endl << LegCoordinate[0][0] << endl << LegCoordinate[1][0] << endl << LegCoordinate[2][0] ;
    MoveA_legToCoordinate(5,LegCoordinate);
    return MoveLegsToCoordinates(Coordinate);
}

/*
//运动学,输入关节角度求末端位置1
double * CalculateAnglesToCoordinates(double Angle[3][6])
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
*/

/*
//逆运动学,输入末端位置求关节角度2
double * CalculateCoordinatesToAngles(double Coordinate[3][6])
{
    //leg0
//    a0 = 2 * L2 * (sqrt(Coordinate[0][0] * Coordinate[0][0] + Coordinate[1][0] * Coordinate[1][0]) - 0.890835 * L1);
//    b0 = 2 * (Coordinate[2][0] + 0.454326 * L1) * L2;
//    c0 = pow((sqrt(Coordinate[0][0] * Coordinate[0][0] + Coordinate[1][0] * Coordinate[1][0]) - 0.890835 * L1),2) + pow((Coordinate[2][0] + 0.454326 * L1),2) + pow(L2,2) - pow(L3,2);
//
//    Theta[0][0] = atan2(Coordinate[1][0],Coordinate[0][0]);
//    Theta[1][0] = atan2(c0,-sqrt(a0 * a0 + b0 * b0 - c0 * c0)) - atan2(a0,b0);
//    Theta[2][0] = acos((c0 - 2 * pow(L2,2))/(2 * L2 * L3));

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
*/

/*
//给定坐标,判断是否可达3
double IfReachable(double Coordinate[3][6])
{
    CalculateCoordinatesToAngles(Coordinate);

    if (Angle[0][0] < 60.1 && Angle[0][0] > -60.1 && Angle[1][0] < 80.1 && Angle[1][0] > -90.1 && Angle[2][0] < 1 && Angle[2][0] > -135.1 )
    {
        cout << "Reachable :)" << endl ;
        return 1 ;
    } else
    {
        cout << "Unreachable :(" << endl ;
        return 0 ;
    }
}
*/

/*
//六条腿to坐标&动作指令2
string MoveLegsToCoordinates(double Coordinate[3][6])
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
            CMD = CMD + VPT[i][j];
        }
    }
    CMD = CMD + T;
    cout << endl << CMD << endl ;
    return CMD;
}
*/

/*
//六条腿to角度&动作指令1
string MoveLegsToAngles(double Angle[3][6])
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
            CMD = CMD + VPT[i][j];
        }
    }
    CMD = CMD + T;
    cout << endl << CMD << endl ;
    return CMD;
}
*/

/*
//单腿to角度1
void MoveA_legToAngle(int LegNumber , double LegAngle[3][1])
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
*/

/*
//单腿to坐标2
void MoveA_legToCoordinate(int LegNumber , double LegCoordinate[3][1])
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
*/

/*
//寻找最近点3
double FindCloseReachable(double coordinates[3])
{
    double x0=coordinates[0],y0=coordinates[1],z0=coordinates[2],t0,m[3],a,b,c,x1,y1,z1,t1,t,l,x2,y2,z2;
    t0=atan(y0/x0);
    if(t0>M_PI/3) {t0=M_PI/3;}
    if(t0<-M_PI/3) {t0=-M_PI/3;}
    l=sqrt(x0*x0+y0*y0);
    x1=l*cos(t0);
    y1=l*sin(t0);
    if((pow(coordinates[0]-50*cos(t0),2)+pow(coordinates[1]-50*sin(t0),2)+pow(coordinates[2]+25.48,2))>=59931.9361)//在第一层理论边界外  大球外
    {
        if(l<50)//同时在第一层边界和第二层边界外  小球内
        {
            x1=50*cos(t0);
            y1=50*sin(t0);
            if(z0>0) {z2=219.33,x2=x1,y2=y1;}
            else {z2=-270.29,x2=x1,y2=y1;}
        }
        else//在第一层边界外 大球外,在第二层边界外  小球外 指向 最大球边界交点
        { a=x1*x1+y1*y1+z0*z0;
            b=50.96*z1-100*cos(t0)*x1-100*sin(t0);
            c=2500+25.48*25.48-59931.9361;
            t=((-b+sqrt(b*b-4*a*c))/(2*a));
            x2=x1*t;
            y2=y1*t;
            z2=z0*t;
        }
    }
    else//在第一层理论边界内 大球内
    {
        if(l<50)//在第一层边界内,大球内 在第二层边界内 小球内  ThetaB 90 -90 上下只指 区域6
        {
            x1=50*cos(t0);
            y1=50*sin(t0);
            if(z0>0) {z2=219.33,x2=x1,y2=y1;}
            else {z2=-270.29,x2=x1,y2=y1;}
        }
        else//在第一层边界内,在第二层边界内  区域 3 4
        {
            x2=x1;
            y2=y1;
            z2=z0;
        }
    }
    CloseCoordinates[0]=x2;
    CloseCoordinates[1]=y2;
    CloseCoordinates[2]=z2;
    cout << CloseCoordinates[0] << endl << CloseCoordinates[1] << endl << CloseCoordinates[2] << endl ;
}
 */