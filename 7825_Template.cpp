////////////////////////////////////////////////////////////////
// ELEX 7825 Template project for BCIT
// Created Sept 9, 2020 by Craig Hennessey
// Last updated Sept 26, 2021
////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include "uArm.h"

using namespace std;
using namespace cv;

using namespace dnn;
using namespace aruco;

#include <opencv2/face/facemark.hpp>

#include "Robot.h"

void lab7()
{
  CuArm uarm;
  uarm.init_com("COM4");
  uarm.init_robot();
}

int main(int argc, char* argv[])
{
  CRobot robot;
  int input_select = -1;

  while (input_select != 0)
  {
    cout << "\n*****************************************************";
    cout << "\n(1) Lab 1 - ";
    cout << "\n(2) Lab 2 - ";
    cout << "\n(3) Lab 3 - ";
    cout << "\n(4) Lab 4 - ";
    cout << "\n(5) Lab 5 - ";
    cout << "\n(6) Lab 6 - ";
    cout << "\n(7) Lab 7 - ";
    cout << "\n(8) Demo draw";
    cout << "\n(9) Demo cam";
    cout << "\n(0) Exit";
    cout << "\n>> ";

    cin >> input_select;
    switch (input_select)
    {
    //case 1: lab1(); break;
    //case 2: lab2(); break;
    //case 3: lab3(); break;
    //case 4: lab4(); break;
    //case 5: lab5(); break;
    //case 6: lab6(); break;
    case 7: lab7(); break;
    case 8: robot.draw(); break;
    case 9: robot.calibrate_board(0); break;
    }
  }

  return 1;
}
