//
// Created by bie on 2020/12/16.
//

#include <string>

#ifndef BODYCTRL_ANGLE_CTRL_H
#define BODYCTRL_ANGLE_CTRL_H

#endif //BODYCTRL_ANGLE_CTRL_H

extern double * CalculateAnglesToCoordinates(double Angle[3][6]);
extern std::string MoveLegsToAngles(double Angle[3][6]);
extern void MoveA_legToAngle(int LegNumber , double LegAngle[3][1]);

