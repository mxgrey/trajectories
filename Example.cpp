/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <cstdio>
#include <Eigen/Core>
#include "Trajectory.h"
#include "Path.h"
#include <vector>

using namespace std;
using namespace Eigen;

static void print_limit_violation(const std::string& type, 
                                  size_t joint_index,
                                  double limit, double value,
                                  size_t traj_index)
{
    std::cout << "Joint (" << joint_index << ") violated its "
              << type << " (" << limit << ") with a value of " << value
              << " at index " << traj_index << "!\n";
}

const double eps = 1;

int main() {
	list<VectorXd> waypoints;
//	VectorXd waypoint(3);
//	waypoint << 0.0, 0.0, 0.0;
//	waypoints.push_back(waypoint);
//	waypoint << 0.0, 0.2, 1.0;
//	waypoints.push_back(waypoint);
//	waypoint << 0.0, 3.0, 0.5;
//	waypoints.push_back(waypoint);
//	waypoint << 1.1, 2.0, 0.0;
//	waypoints.push_back(waypoint);
//	waypoint << 1.0, 0.0, 0.0;
//	waypoints.push_back(waypoint);
//	waypoint << 0.0, 1.0, 0.0;
//	waypoints.push_back(waypoint);
//	waypoint << 0.0, 0.0, 1.0;
    
    VectorXd waypoint(6);
    waypoint << -0.603869, -0.21467, 0.361972, -0.81784, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << 0, 0, 0, 0, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.0503237, -0.0178896, 0.0301651, -0.0681551, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.100647, -0.0357792, 0.0603302, -0.13631, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.150971, -0.0536689, 0.0904953, -0.204465, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.201295, -0.0715585, 0.12066, -0.27262, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.251618, -0.0894481, 0.150826, -0.340776, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.301942, -0.107338, 0.180991, -0.408931, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.352266, -0.125227, 0.211156, -0.477086, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.402589, -0.143117, 0.241321, -0.545241, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.452913, -0.161007, 0.271486, -0.613396, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.503237, -0.178896, 0.301651, -0.681551, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.55356, -0.196786, 0.331816, -0.749706, 0, 0;
    waypoints.push_back(waypoint);
    waypoint << -0.603884, -0.214675, 0.361981, -0.817861, 0, 0;
    waypoints.push_back(waypoint);
    
    VectorXd ones(waypoint.size()); ones.setOnes();
    double maxAccel = 0.8;
	VectorXd maxAcceleration(waypoint.size());
    maxAcceleration = maxAccel * ones;
    double maxSpeed = 1.0;
	VectorXd maxVelocity(waypoint.size());
    maxVelocity = maxSpeed * ones;

	Trajectory trajectory(Path(waypoints, 0.01), maxVelocity, maxAcceleration);
	trajectory.outputPhasePlaneTrajectory();
	if(trajectory.isValid()) {
		double duration = trajectory.getDuration();
		cout << "Trajectory duration: " << duration << " s" << endl << endl;
		cout << "Time      Position                  Velocity" << endl;
		for(double t = 0.0; t < duration; t += 0.1) {
			printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", t, trajectory.getPosition(t)[0], trajectory.getPosition(t)[1], trajectory.getPosition(t)[2],
				trajectory.getVelocity(t)[0], trajectory.getVelocity(t)[1], trajectory.getVelocity(t)[2]);
		}
		printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", duration, trajectory.getPosition(duration)[0], trajectory.getPosition(duration)[1], trajectory.getPosition(duration)[2],
			trajectory.getVelocity(duration)[0], trajectory.getVelocity(duration)[1], trajectory.getVelocity(duration)[2]);
	}
	else {
		cout << "Trajectory generation failed." << endl;
	}

    double frequency = 200;
    double dt = 1.0/frequency;
    size_t traj_count = trajectory.getDuration()*frequency;
    std::vector<VectorXd> points;
    for(size_t i=0; i<traj_count; ++i)
    {
        points.push_back(trajectory.getPosition(i*dt));
    }
    
    std::cout << "\n" << std::endl;
    
    bool limits_okay = true;
    for(size_t i=0; i<points.size(); ++i)
    {
        const VectorXd& elem = points[i];
        const VectorXd& last_elem = ( i==0 ) ?
                    points[0] : points[i-1];
        
        const VectorXd& next_elem = ( i == points.size()-1 ) ?
                    points.back() : points[i+1];
        
        for(int j=0; j<elem.size(); ++j)
        {

            if( !(elem[j] == elem[j]) )
            {
                print_limit_violation("NaN detection", j,
                                      0, elem[j], i);
                limits_okay = false;
            }

            if( i == 0 )
            {
                // No sense in checking speed if this is the first element
                continue;
            }
            
            double speed = fabs(elem[j] - last_elem[j])
                           * frequency;
            if( speed > maxSpeed + eps )
            {
                print_limit_violation("max speed", j,
                                      maxSpeed, speed, i);
                limits_okay = false;
            }
            
            if( i == 0 || i == points.size()-1 )
            {
                // No sense in checking acceleration if this is the first or last element
                continue;
            }

            double accel = fabs(next_elem[j] - 2*elem[j] + last_elem[j])
                           * frequency * frequency;
            if( accel > maxAccel + eps )
            {
                print_limit_violation("max acceleration", j,
                                      maxAccel, accel, i);
                limits_okay = false;
            }
        }
    }
    
    if(limits_okay)
        std::cout << "No violations found!" << std::endl;
}


