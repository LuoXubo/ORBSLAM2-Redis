/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<string>

#include<opencv2/core/core.hpp>

#include<System.h>

#include <sw/redis++/redis++.h>
#include "/home/zino/lxb/SLAM-codes/ORBSLAM2-Redis/build/poseInfo.pb.h"


using namespace std;
using namespace sw::redis;

typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Quaterniond Qua;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void LoadAbsoluteLocation(vector<Vec3> &map_abs_xyz,
                          vector<Qua> &map_abs_qwxyz,
                          vector<double> &vTimestamps,
                          vector<double> &conf);

void call_back(std::string channel, std::string msg);

// Redis redis("tcp://127.0.0.1:6379");
// Redis redis1("tcp://127.0.0.1:6379");

// Load xy data. 
std::vector<Vec3> map_abs_xyz;
std::vector<Qua> map_abs_qwxyz;
std::vector<double> conf;

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/images.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);
    LoadAbsoluteLocation(map_abs_xyz, map_abs_qwxyz, vTimestamps, conf);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);

            // redis1.publish("l2g", "ok"); // 准备好接收了
            // // cout << "OK\n";
            // auto sub = redis.subscriber();
            // sub.subscribe("g2l");
            // sub.on_message(call_back);
            // sub.consume();
            // while(true){
            //     try
            //     {
            //         sub.consume();
            //     }
            //     catch(const Error &err)
            //     {
            //         // std::cerr << e.what() << '\n';
            //     }
                
            // }
        }
    }
}

void LoadAbsoluteLocation(vector<Vec3> &map_abs_xyz,
                          vector<Qua> &map_abs_qwxyz,
                          vector<double> &vTimestamps,
                          vector<double> &conf){

                            Redis redis("tcp://127.0.0.1:6379");
                            auto sub = redis.subscriber();
                            sub.subscribe("g2l");
                            sub.on_message(call_back);
                            while(conf.size() != 116){
                                try{
                                    sub.consume();
                                }
                                catch (const Error &err) {
                                    // Handle exceptions.
                                }
                            }
                          }


void call_back(std::string channel, std::string msg)
{
    // cout << "Hello\n";
    if(channel == "g2l"){
        posebag::poseInfo pos;
        pos.ParseFromString(msg);
        float timestamp, x, y, z, qw, qx, qy, qz, c;
        x = pos.x();
        y = pos.y();
        z = pos.z();
        qw = pos.qw();
        qx = pos.qx();
        qy = pos.qy();
        qz = pos.qz();
        // timestamp = atof(pos.timestamp());
        c = pos.conf();

        Qua quaternion(qw, qx, qy, qz);
        // vTimestamps.push_back(timestamp);
        map_abs_xyz.push_back(Vec3(x, y, z));
        map_abs_qwxyz.push_back(quaternion);
        conf.push_back(c);

        cout << pos.timestamp() << endl;
    }
}