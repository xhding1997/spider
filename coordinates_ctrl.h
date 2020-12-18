//
// Created by bie on 2020/12/18.
//

#include <string>

#ifndef BODYCTRL_COORDINATES_CTRL_H
#define BODYCTRL_COORDINATES_CTRL_H

#endif //BODYCTRL_COORDINATES_CTRL_H

extern double * CalculateCoordinatesToAngles(double Coordinate[3][6]);
extern std::string MoveLegsToCoordinates(double Coordinate[3][6]);
extern void MoveA_legToCoordinate(int LegNumber , double LegCoordinate[3][1]);