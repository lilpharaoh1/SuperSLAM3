/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <super_msgs/MatchesStamped.h>
#include <super_msgs/Matches.h>

#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
#include"../../../include/System.h"

using namespace std;

class SuperHandler
{
public: 
    SuperHandler(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabMatches(const super_msgs::MatchesStampedPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Super");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Super path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    SuperHandler shb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/matches", 1, &SuperHandler::GrabMatches,&shb);
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void SuperHandler::GrabMatches(const super_msgs::MatchesStampedPtr& msg)
{
    const double &timestamp = msg->header.stamp.toSec();
    vector<int> kpts0_x, kpts0_y, kpts1_x, kpts1_y;
    vector<float> confidences;
    for (int i = 0; i < msg->matches.keypoints0.features.size(); i++) { 
        // cout << msg->matches.keypoints0.features[i].x << endl;
        kpts0_x.push_back(msg->matches.keypoints0.features[i].x);
        kpts0_y.push_back(msg->matches.keypoints0.features[i].y);
        kpts1_x.push_back(msg->matches.keypoints1.features[i].x);
        kpts1_y.push_back(msg->matches.keypoints1.features[i].y);
        confidences.push_back(msg->matches.confidences[i]);
    }

    mpSLAM->TrackSuper(kpts0_x, kpts0_y, kpts1_x, kpts1_y, confidences, timestamp);
}


