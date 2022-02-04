//3.2 Bearing sensor
#ifndef SENSOR_BEARING_H
#define SENSOR_BEARING_H

#include <iostream>
#include <math.h>
#include <sensor.h>
#include <string>
#include <envir.h>
#include <robot.h>
#include <geom.h>

using namespace arpro;
using namespace std;

namespace arpro {
class BearingSensor : public Sensor{
public:
    BearingSensor(Robot &_robot, double _x, double _y, double _theta, double g, double sm):
        Sensor(_robot, _x, _y, _theta){}

    void update(const Pose &_p){
        for(auto other : envir_->robots_){
            if(other != robot_){
                s_ = atan2(other->pose().y - _p.y, other->pose().x - _p.x) - _p.theta;
                break;
            }
        }
       if(s_ < -M_PI)
           s_ += 2*M_PI;
       else if (s_ > M_PI) {
           s_ -= 2*M_PI;
       }
    }

    void correctTwist(Twist &_v){
        _v.w = _v.w - g_*s_;
    }
protected:
    const double g_ = 0.5;

};

}


#endif // SENSOR_BEARING_H
