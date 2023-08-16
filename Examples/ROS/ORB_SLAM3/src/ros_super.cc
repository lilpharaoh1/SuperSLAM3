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
    cout << "----------------------------start of callback--------------------------" << endl;
    const double &timestamp = msg->header.stamp.toSec();
    vector<cv::Point2f> kpts;
    vector<int> mpts_prev, mpts_curr;
    vector<float> confidences;

    cv::Point2f pt;
    for (int i = 0; i < msg->matches.keypoints.size(); i++) { 
        pt.x = msg->matches.keypoints[i].x;
        pt.y = msg->matches.keypoints[i].y;
        kpts.push_back(pt);
    }

    for (int i = 0; i < msg->matches.prev.size(); i++) { 
        mpts_prev.push_back(msg->matches.prev[i]);
        mpts_curr.push_back(msg->matches.curr[i]);
        confidences.push_back(msg->matches.confidences[i]);
    }

    mpSLAM->TrackSuper(kpts, mpts_prev, mpts_curr, confidences, timestamp);
    cout << "----------------------------end of callback--------------------------" << endl;
}


