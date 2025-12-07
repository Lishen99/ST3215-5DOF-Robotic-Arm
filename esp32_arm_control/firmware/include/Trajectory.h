#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class Trajectory {
public:
    Trajectory();
    
    void setTarget(Matrix<5> start_pose, Matrix<5> end_pose, float duration);
    Matrix<5> getPose(float time);
    bool isFinished(float time);

private:
    Matrix<5> _start_pose;
    Matrix<5> _end_pose;
    float _duration;
    float _start_time;
};

#endif
