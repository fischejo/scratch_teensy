// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

#define RPM_TO_RPS 1/60

class Kinematics
{
    public:
       
        struct rpm
        {
            float motor1;
            float motor2;
        };
        
        struct velocities
        {
            float linear_x;
            float linear_y;
            float angular_z;
        };


        Kinematics();

        velocities getVelocities(float rpm1, float rpm2);

        rpm getRPM(float linear_x, float linear_y, float angular_z);


    private:
       
        float max_rpm_;
        float wheels_y_distance_;
        float wheel_circumference_;
};

#endif