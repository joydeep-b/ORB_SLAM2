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
#include<iomanip>
#include<string>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;
// Returns truee iff the file specified exists in the file system.
bool FileExists(const std::string& file_name) {
  struct stat st;
  return(stat(file_name.c_str(), &st) == 0);
}

void LoadImages(const string &strSequence,
                vector<string> &files,
                vector<double> &times)
{
  static const float kFrameRate = 1.0 / 29.98;
  static const int kMaxNumFiles = 1000000;
  for (int i = 1; i < kMaxNumFiles; ++i) {
    char filename[1024];
    snprintf(filename,
             sizeof(filename),
             "%s/%06d.png",
             strSequence.c_str(),
             i);
    if (FileExists(filename)) {
      files.push_back(filename);
      times.push_back(static_cast<float>(i - 1) * kFrameRate);
    } else {
      break;
    }
  }
}

int main(int argc, char **argv)
{
    bool use_gui = true;
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_umass path_to_vocabulary path_to_settings path_to_sequence [--no-gui]" << endl;
        return 1;
    } else if (argc >= 5 && strcmp(argv[4], "--no-gui") == 0) {
      use_gui = false;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    const string dataset(argv[3]);
    LoadImages(dataset, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,use_gui);

    vector<float> vTimesTrack(nImages, 0);

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        printf("\rProcessing frame %6d / %d ", ni + 1, nImages);
        fflush(stdout);
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
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
    cout << endl << "-------" << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryKITTI(dataset + "/trajectory.txt");
    return 0;
}
