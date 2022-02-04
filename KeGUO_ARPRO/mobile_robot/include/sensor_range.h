//3.1 Range sensor
#ifndef SENSOR_RANGE_H
#define SENSOR_RANGE_H

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
class RangeSensor : public Sensor{
public:
    RangeSensor(Robot &_robot, double _x, double _y, double _theta, double g, double sm):
        Sensor(_robot, _x, _y, _theta){
        // Rangesensor constructor will call the sensor constructor
        g_ = g;
        sm_ = sm;

    }

    void update(const Pose &p_){
        Pose p1,p2;
        double c = cos(p_.theta);
        double s = sin(p_.theta);

        s_ = 100; // initialize s_ with a high value

        for (int i = 0; i < envir_->walls.size(); ++i){
            p1 = envir_ -> walls[i];
            p2 = envir_ -> walls[(i+1)%envir_->walls.size()];

            double d = (p1.x*p2.y - p1.x*p_.y - p2.x*p1.y + p2.x*p_.y + p_.x*p1.y - p_.x*p2.y)/(p1.x*s - p2.x*s - p1.y*c +p2.y*c);
            if((d>0) && (d<s_)){
                s_ = d;
            }
        }
        printf("Current Measurement: %f\n", s_);
        printf("----------------\n");

    }
    void correctTwist(Twist &_v){
        // 3.1 Q5:
        if(s_ < 3){ // when the robot is close to the wall, change the twist
            if(_v.vx > g_*(s_ - sm_))
                _v.vx = g_*(s_ - sm_);
        }
    };
protected:
    double g_;
    double sm_;
};

}


#endif // SENSOR_RANGE_H
