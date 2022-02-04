#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_range.h>
#include <sensor_bearing.h>

using namespace std;
using namespace arpro;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);

  // init robot at (0,0,0)
  Robot robot("R2D2", 0,0,0);

  // 2.2 Q5:
  robot.initWheel(0.07, 0.3, 10);
  envir.addRobot(robot);

  //3.1:
  Robot robot2("range", 0,0,0);
  robot2.initWheel(0.07, 0.3, 10);
  envir.addRobot(robot2);
  RangeSensor rangesensor(robot2, 0.1, 0, 0, 0.1, 0.1);
  RangeSensor x1(robot2, 0.1, 0, 0, 0.1, 0.1);


  //3.2:
  Robot robot3("bear", 0,0,0);
  robot3.initWheel(0.05, 0.3, 10); // smaller wheels
  envir.addRobot(robot3);
  BearingSensor bearingsensor(robot3, 0.1, 0, 0, 0.1, 0.1);

  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target()); // 2.2 Q1: the motion to go to the target is defined in robot.cpp

    robot2.goTo(envir.target());

    robot3.moveWithSensor(Twist(0.4, 0, 0));


  }

  // plot trajectory
  envir.plot();

}
