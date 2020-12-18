#include "reachable.h"
#include "matrix.h"
#include "coordinates_ctrl.h"
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <sstream>
#include <unistd.h>

using namespace std;
using namespace Eigen;

//给定坐标,判断是否可达3
extern double IfReachable(double Coordinate[3][6])
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

//寻找最近点3
extern double FindCloseReachable(double coordinates[3])
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
//    cout << CloseCoordinates[0] << endl << CloseCoordinates[1] << endl << CloseCoordinates[2] << endl ;
}

