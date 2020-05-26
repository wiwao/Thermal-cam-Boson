#pragma once
#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOsc.h"
#include <stdio.h>
#include <fcntl.h>               // open, O_RDWR
//#include <opencv2/opencv.hpp>

#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>

// send host (aka ip address)
//#define HOST "192.168.0.8"

/// send port
#define SPORT 8090
#define RPORT 8091
// demonstrates sending OSC messages with an ofxOscSender,
// use in conjunction with the oscReceiveExample
class ofApp : public ofBaseApp{

	public:

		void setup();
		void update();
		void draw();
        //void exit();
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		
		ofxOscSender sender;
		ofxOscReceiver receiver;
		
		
		string arg_x;
		string HOST="192.168.0.12";
		
};
