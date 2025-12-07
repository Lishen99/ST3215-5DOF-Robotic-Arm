#include "Trajectory.h"

Trajectory::Trajectory() : _duration(0) {}

void Trajectory::setTarget(Matrix<5> start_pose, Matrix<5> end_pose, float duration) {
    _start_pose = start_pose;
    _end_pose = end_pose;
    _duration = duration;
}

bool Trajectory::isFinished(float time) {
    return time >= _duration;
}

Matrix<5> Trajectory::getPose(float time) {
    if (time >= _duration) return _end_pose;
    if (time <= 0) return _start_pose;
    
    float t = time / _duration;
    
    // Quintic easing or simple Linear for now
    // Linear:
    // return _start_pose + (_end_pose - _start_pose) * t;
    
    // Smoothstep (Cubic): 3t^2 - 2t^3
    float smooth_t = t * t * (3.0f - 2.0f * t);
    
    return _start_pose + (_end_pose - _start_pose) * smooth_t;
}
