/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include "cstring"
#include "string"
#include <iomanip>
#include <iostream>

#include "opencv2/opencv.hpp"
#include "opencv2/viz.hpp"
#include "Eigen/Dense"

extern "C" {
#include "apriltag.h" 
#include "apriltag_pose.h"
#include "common/getopt.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
// for sending udp packets
#include "arpa/inet.h"
#include "stdlib.h"
#include "sys/socket.h"
#include "unistd.h"
}

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
  getopt_t *getopt = getopt_create();

  getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
  getopt_add_int(getopt, 'c', "camera", "0", "camera ID");
  getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
  getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
  getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
  getopt_add_string(getopt, 'a', "addr", "192.168.0.68",
                    "IP address for sending pose data to");
  getopt_add_int(getopt, 'p', "port", "9800", "port to send pose data on");
  getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
  getopt_add_double(getopt, 'x', "decimate", "2.0",
                    "Decimate input image by this factor");
  getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
  getopt_add_bool(getopt, '0', "refine-edges", 1,
                  "Spend more time trying to align edges of tags");

  if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
    printf("Usage: %s [options]\n", argv[0]);
    getopt_do_usage(getopt);
    exit(0);
  }

  // setup UDP socket
  const char *peer_ip = getopt_get_string(getopt, "addr");
  int peer_port = getopt_get_int(getopt, "port");

  struct sockaddr_in peer_addr = {.sin_family = AF_INET,
                                  .sin_port = htons(peer_port)};

  if (inet_pton(AF_INET, peer_ip, &(peer_addr.sin_addr)) <= 0) {
    cerr << "Something went wrong with the IP address!" << endl;
  }

  int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket < 0) {
    cerr << "Sorry, couldn't create the socket." << endl;
  }

  cout << "Enabling video capture" << endl;

  TickMeter meter;
  meter.start();

  // Initialize camera
  VideoCapture cap(getopt_get_int(getopt, "camera"), cv::CAP_AVFOUNDATION);
  if (!cap.isOpened()) {
    cerr << "Couldn't open video capture device" << endl;
    return -1;
  }
  if (cap.set(cv::CAP_PROP_FPS, 30)) {
    cout << "framerate set successfully" << endl;
  } else {
    cerr << "failed to set framerate" << endl;
  }

  // Initialize tag detector with options
  apriltag_family_t *tf = NULL;
  const char *famname = getopt_get_string(getopt, "family");
  if (!strcmp(famname, "tag36h11")) {
    tf = tag36h11_create();
  } else if (!strcmp(famname, "tag25h9")) {
    tf = tag25h9_create();
  } else if (!strcmp(famname, "tag16h5")) {
    tf = tag16h5_create();
  } else if (!strcmp(famname, "tagCircle21h7")) {
    tf = tagCircle21h7_create();
  } else if (!strcmp(famname, "tagCircle49h12")) {
    tf = tagCircle49h12_create();
  } else if (!strcmp(famname, "tagStandard41h12")) {
    tf = tagStandard41h12_create();
  } else if (!strcmp(famname, "tagStandard52h13")) {
    tf = tagStandard52h13_create();
  } else if (!strcmp(famname, "tagCustom48h12")) {
    tf = tagCustom48h12_create();
  } else {
    printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
    exit(-1);
  }

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  if (errno == ENOMEM) {
    printf("Unable to add family to detector due to insufficient memory to "
           "allocate the tag-family decoder with the default maximum hamming "
           "value of 2. Try choosing an alternative tag family.\n");
    exit(-1);
  }

  td->quad_decimate = getopt_get_double(getopt, "decimate");
  td->quad_sigma = getopt_get_double(getopt, "blur");
  td->nthreads = getopt_get_int(getopt, "threads");
  td->debug = getopt_get_bool(getopt, "debug");
  td->refine_edges = getopt_get_bool(getopt, "refine-edges");

  meter.stop();
  cout << "Detector " << famname << " initialized in " << std::fixed
       << std::setprecision(3) << meter.getTimeSec() << " seconds" << endl;
#if CV_MAJOR_VERSION > 3
  cout << "  " << cap.get(CAP_PROP_FRAME_WIDTH) << "x"
       << cap.get(CAP_PROP_FRAME_HEIGHT) << " @" << cap.get(CAP_PROP_FPS)
       << "FPS" << endl;
#else
  cout << "  " << cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
       << cap.get(CV_CAP_PROP_FRAME_HEIGHT) << " @" << cap.get(CV_CAP_PROP_FPS)
       << "FPS" << endl;
#endif
  meter.reset();

  Mat frame, gray;
  while (true) {
    errno = 0;
    cap >> frame;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

    zarray_t *detections = apriltag_detector_detect(td, &im);

    if (errno == EAGAIN) {
      printf("Unable to create the %d threads requested.\n", td->nthreads);
      exit(-1);
    }

    // loop throught the tags
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      line(frame, Point(det->p[0][0], det->p[0][1]),
           Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
      line(frame, Point(det->p[0][0], det->p[0][1]),
           Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
      line(frame, Point(det->p[1][0], det->p[1][1]),
           Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
      line(frame, Point(det->p[2][0], det->p[2][1]),
           Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);

      // get pose of tag
      apriltag_detection_info_t info;
      info.det = det;
      info.tagsize = 0.155575;
      info.fx = 1483; // 678;
      info.fy = 1483; // 678;
      info.cx = 1920 / 2;
      info.cy = 1080 / 2;
      apriltag_pose_t pose;

      // send UDP packet
      stringstream idss;
      idss << det->id;
      String text = idss.str();
      int fontface = FONT_HERSHEY_COMPLEX;
      double fontscale = 1.0;
      int baseline;
      Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
      putText(frame, text,
              Point(det->c[0] - textsize.width / 2,
                    det->c[1] + textsize.height / 2),
              fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

      double err = estimate_tag_pose(&info, &pose);
      Eigen::Vector3d position(pose.t->data[0], pose.t->data[1], pose.t->data[2]);
      Eigen::MatrixXd rotationMatrix(3,3);
      rotationMatrix << pose.R->data[0],pose.R->data[1],pose.R->data[2],
                        pose.R->data[3],pose.R->data[4],pose.R->data[5],
                        pose.R->data[6],pose.R->data[7],pose.R->data[8];
      Eigen::Vector3d newPosition = rotationMatrix * position;
      cout << newPosition(0) << ", " << newPosition(1) << ", " << newPosition(2) << endl;
        
      const int message_size = sizeof(int) + sizeof(double) * 13;
      char *message = (char *)malloc(message_size);
      size_t index = 0;
      memcpy(message, &(det->id), sizeof(int));
      index += sizeof(int);
      memcpy(message + index, &err, sizeof(double));
      index += sizeof(double);
      memcpy(message + index, &(pose.t->data[0]), sizeof(double) * 3);
      index += sizeof(double) * 3;
      memcpy(message + index, &(pose.R->data[0]), sizeof(double) * 9);
      if (sendto(udp_socket, message, message_size, 0,
                 (struct sockaddr *)&peer_addr, sizeof(peer_addr)) < 0) {
        cerr << "Failed to send message." << endl;
      }
      free(message);
      // cout << pose.t->data[0] << ", " << pose.t->data[1] << ", " << pose.t->data[2] << endl;
    }

    apriltag_detections_destroy(detections);

    imshow("Tag Detections", frame);
    if (waitKey(30) >= 0)
      break;
  }

  apriltag_detector_destroy(td);

  if (!strcmp(famname, "tag36h11")) {
    tag36h11_destroy(tf);
  } else if (!strcmp(famname, "tag25h9")) {
    tag25h9_destroy(tf);
  } else if (!strcmp(famname, "tag16h5")) {
    tag16h5_destroy(tf);
  } else if (!strcmp(famname, "tagCircle21h7")) {
    tagCircle21h7_destroy(tf);
  } else if (!strcmp(famname, "tagCircle49h12")) {
    tagCircle49h12_destroy(tf);
  } else if (!strcmp(famname, "tagStandard41h12")) {
    tagStandard41h12_destroy(tf);
  } else if (!strcmp(famname, "tagStandard52h13")) {
    tagStandard52h13_destroy(tf);
  } else if (!strcmp(famname, "tagCustom48h12")) {
    tagCustom48h12_destroy(tf);
  }

  getopt_destroy(getopt);
  close(udp_socket);

  return 0;
}
