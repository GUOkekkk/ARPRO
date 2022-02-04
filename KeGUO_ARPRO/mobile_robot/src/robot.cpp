

#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

Robot::Robot(string _name, double _x, double _y, double _theta) // 2.2 Q2: define a Robot constructor in class Robot, like initialization in defining a class
// the arguments are passed directly
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}



void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}

void Robot::rotateWheels(double _left, double _right)
{
    // to fill up after defining an initWheel method
    // 2.2 Q7:
    if(wheels_init_){
    // 2.3 Q2:
    if((abs(_left) >= w_limit_) || (abs(_right)>=w_limit_))
    {
        double a = max(abs(_left)/w_limit_, abs(_right)/w_limit_);
        a = max(a,1.0);
        _left = _left/a;
        _right = _right/a;
        cout<<"The velocity setpoint is too high"<<endl;
    }
    // 2.2 Q6:
    double _theta = pose_.theta + (rm*(_left - _right)/(2*bm)) * dt_;
    double _vx = cos(_theta) * (rm*(_left + _right)/2);
    double _vy = sin(_theta) * (rm*(_left + _right)/2);
    double _omega = (rm*(_left - _right)/(2*bm));

    Robot::moveXYT(_vx, _vy, _omega);
    }else
        cout<<"Please initialize the wheels"<<endl;





}


// 2.2 Q3:
// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    // to fill up
    //just use equations in Q3
    //const double x_dot = _v*cos(pose_.theta);
    //const double y_dot = _v*sin(pose_.theta);
    //const double thete_dot = _omega;
    //moveXYT(x_dot, y_dot, thete_dot);

    // 2.3 Q3:
    double left = (_v + bm*_omega)/rm;
    double right = (_v - bm*_omega)/rm;
    rotateWheels(left, right);


}




// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{
    // to fill up, sensor measurement and twist checking
    // 3.1 Q3:
    for(auto &sensor:sensors_){
        sensor->updateFromRobotPose(pose_);
        sensor->correctRobotTwist(_twist);
    }



    // uses XYT motion (perfect but impossible in practice)
    // to fill up, use V-W motion when defined
    // 2.2 Q5:
    // moveXYT(_twist.vx, _twist.vy,_twist.w);
    double x_dot = _twist.vx;
    double y_dot = _twist.vy;
    double theta_dot = _twist.w;

    double alpha = 20;
    double v = x_dot;// = _twist.vx;
    double omega = alpha*y_dot + theta_dot;

    moveVW(v, omega);
}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

