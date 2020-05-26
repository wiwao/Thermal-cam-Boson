// 2020/3/30by Takeshi Ono in Japan
// 2019/12/15 by Takeshi Ono in Japan
//温度補正ルーチン追加
//BOSON　RAWデータ読み込みはBOSON SDKを使用している。
//OSCでデータを送らなければ50FPSくらいで読み込み可能
//OSCでデータ送信中は25FPS全般まで下がる。受信側は20FPS弱。OSCでデータ損失無く画像送信する限度かも知れない

#include "ofApp.h"

extern "C"{
#include "UART_HalfDuplex.h"
#include "flirCRC.h"
#include "rs232.h"
#include "flirChannels.h"
#include "BOSON/Client_API.h"
}

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
//********************BOSON

#define YUV   0
#define RAW16 1

using namespace cv;

#define v_major 1
#define v_minor 0

// Define COLOR CODES
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

//***************************
int fd;
static uint8_t result[164*240];
static uint16_t *frameBuffer;
ofxOscMessage mm;

ofImage img,img1,myImage1; 
//ofTrueTypeFont font;
ofPixels pixels;

char Pi3_ip[15];

float   min1,max1,tem1,min2,max2,tem2;
int     touchX =159, touchY = 127;
float   sl_256 = 0.0f;
float   value_min = 0, value_max = 0, tempXY  = 0,value_min1 = 0, value_max1 = 0;
int     th_l=999,th_h=999;
float   av3;
int     av5,huri1=0;
float   diff;
bool 	ok;
float 	ttt,tttx,aux;
float 	point_l[9][9];
int 	    past=0;
float 	xreset=0;
//ofImage myImage1;

int     Lepton_3x,read_err=0,test_err=0,F_H=0;

int     b_count=0;
int     ret;
struct  v4l2_capability cap;
long    frame=0;     // First frame number enumeration
char    video[20];   // To store Video Port Device
char    label[50];   // To display the information
char    thermal_sensor_name[20];  // To store the sensor name
char    filename[60];  // PATH/File_count
char    folder_name[30];  // To store the folder name
char    video_frames_str[30];
	// Default Program options
int     video_mode=RAW16;
	
int     video_frames=0;
int     zoom_enable=0;
int     record_enable=0;
float ttt10,ttt11;
double k_a,k_b,k_c;
float temp_b=44.2;
enum sensor_types {
  Boson320, Boson640
};
sensor_types my_thermal=Boson320;

	// To record images
std::vector<int> compression_params;

struct v4l2_requestbuffers bufrequest;
struct v4l2_buffer bufferinfo;
void * buffer_start;
int type;

Size size(640,512);
int width;
int height;
float fpatemp_f;
int luma_height = height+height/2;
int luma_width = width;
int color_space = CV_8UC1;


ofBuffer imgAsBuffer;
//---------AI-----------------------------------------
struct todo_thermal {
    int thermal9x9X;
    int thermal9x9Y;
    int x;
    int y;
};
struct todo_thermal1 {
    float thermal_X;
    int thermal9x9X;
    int thermal9x9Y;
    int x;
    int y;
    
};
todo_thermal xxth;
todo_thermal1 yyth;
std::vector<todo_thermal>thermals;
std::vector<todo_thermal1>thermals1;
int coun_t;
//---------Get my IP---------------------------------
int IP_Adr (char *abc) {
struct ifaddrs * ifAddrStruct=NULL;
struct ifaddrs * ifa=NULL;
void * tmpAddrPtr=NULL;
char addressBuffer[INET_ADDRSTRLEN];

    getifaddrs(&ifAddrStruct);
    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) {
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            printf("%s IP4 Address %s\n", ifa->ifa_name, addressBuffer); 
        }
    }
     for(int ii=0;ii<INET_ADDRSTRLEN;ii++){
           abc[ii]=addressBuffer[ii]; 
	   }
    if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
    return 0;
}
//------------------温度補正用２次方程式係数算出
void keisu(){

    int mimTempR=19365,midTempR=23108,maxTempR=33792;
    float mimTempD=0.0,midTempD=36.0,maxTempD=100.0,nawTempD=44.2;
    
    //設定ファイル読み込み
    vector <string> lines;
    ofBuffer buffer =ofBufferFromFile("/home/pi/boson_temp.txt");
    for (auto line:buffer.getLines()){
	lines.push_back(line);
    } 
    sscanf(lines[0].c_str(), "mimTempD=%f",&mimTempD);
    sscanf(lines[1].c_str(), "mimTempR=%i",&mimTempR);
    sscanf(lines[2].c_str(), "midTempD=%f",&midTempD);
    sscanf(lines[3].c_str(), "midTempR=%i",&midTempR);
    sscanf(lines[4].c_str(), "maxTempD=%f",&maxTempD);
    sscanf(lines[5].c_str(), "maxTempR=%i",&maxTempR);
    sscanf(lines[6].c_str(), "nawTempD=%f",&nawTempD);
    if(mimTempD <0 || mimTempD >=100) mimTempD=0.0f;
    if(midTempD <0 || midTempD >=100) midTempD=36.0f;
    if(maxTempD <0 || maxTempD >=100) maxTempD=100.0f;
    if(nawTempD <0 || nawTempD >=100) nawTempD=44.2f;
    
    if(mimTempR <0 || mimTempR>=65536) mimTempD=0.0f;
    if(midTempR <0 || midTempR>=65536) midTempD=36.0f;
    if(maxTempR <0 || maxTempR>=65536) maxTempD=100.0f;

    
    mimTempD=mimTempD/0.5;midTempD=200.0f-(midTempD/0.5f);maxTempD=maxTempD/0.5f;
    cout << "mimTempD=" <<mimTempD << endl;
    cout << "midTempD=" <<midTempD << endl;
    cout << "maxTempD=" <<maxTempD << endl;
    cout << "mimTempR=" <<mimTempR << endl;
    cout << "midTempR=" <<midTempR << endl;
    cout << "maxTempR=" <<maxTempR << endl;
    cout << "nawTempD=" <<nawTempD<< endl;
    //方程式係数算定
    Mat ll1=(Mat_<double>(3,3) << pow(mimTempD,2.0),mimTempD,1,pow(midTempD,2.0),midTempD,1,pow(maxTempD,2.0),maxTempD,1);
    Mat rr1=(Mat_<double>(3,1) << maxTempR,midTempR,mimTempR);
    Mat ans;
    cv::solve(ll1,rr1,ans);
    k_a=ans.at<double>(0);
    k_b=ans.at<double>(1) ;
    k_c=ans.at<double>(2) ;
    temp_b=nawTempD;
    cout << "k_a=" <<k_a << endl;
    cout << "k_b=" <<k_b << endl;
    cout << "k_c=" <<k_c << endl;
    
}
//--------------------------------------------------------------
void AGC_Basic_Linear(Mat input_16, Mat output_8, int height, int width) {
	int i, j;
    
//void read_lepton(){
	//frameBuffer = (uint16_t *)result;
	//int row, column;
	uint16_t maxValue=0;         // 16 bits
	uint16_t minValue=0xFFFF;    // 16 bits
	uint16_t value1, value2, value3, value4;
	uint16_t ondox[320][256]; //*************hanten you!!
	int tempXY1;
	int col1=touchX,row1=touchY;
    float kei_x=0.95;
	
	for (i=0; i<height; i++) {
		for (j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			if ( value3 <= minValue ) {
				minValue = value3;
			}
			if ( value3 >= maxValue ) {
				maxValue = value3;
			}
		}
	}	
		float rrr = (float)(maxValue - minValue) * (1.0f - abs(sl_256)); //高温側を256等分 iPhoneスライダーに連動 標準は0。最高で0.9	
		diff = rrr / 255.0f;
        ttt11=ofGetElapsedTimef()-ttt10;
        if(ttt11>5){
                printf(GRN "Min=%.1f "  ,(float)minValue);
                printf(RED "Max=%.1f ",(float)maxValue);
                printf(YEL "Temp=%.2f\n"  WHT,fpatemp_f);
                ttt10=ofGetElapsedTimef();
           }
		if(diff < 1.0f) diff = 1.0f;  //256階調で表現できなくなったらノイズを減らすため強制的に256階調演算をさせる
		rrr = (float)minValue +(float)(maxValue - minValue) * sl_256;         //開始低温度側温度
		if (sl_256 > 0) minValue =  rrr; //最低温度表示も開始温度に連動させる
		if (sl_256 < 0) maxValue = rrr; //最高温度表示も開始温度に連動させる minはそのまま
        k_b=abs(k_b);
        value_min1= 100-(k_b-sqrt(abs(pow(-k_b,2.0)-4.0*k_a*(k_c- (float)minValue))))/(2*k_a)*0.5f;
        value_max1= 100-(k_b-sqrt(abs(pow(-k_b,2.0)-4.0*k_a*(k_c- (float)maxValue))))/(2*k_a)*0.5f;
        
        value_min1=value_min1-(temp_b-fpatemp_f)*kei_x;
        value_max1=value_max1-(temp_b-fpatemp_f)*kei_x;
    
        value_min = (float)minValue/(192.752941176f)-80.0f-(40.0f-fpatemp_f);  //65536.0f/340.0f
        value_max = (float)maxValue/(192.752941176f)-80.0f-(40.0f-fpatemp_f);
	
		if(th_l != 999){   //一応エラーチェック　温度範囲指定の時の事前計算
            float 	av1=255.0f/(th_h-th_l);
					av3=av1/(255.0f/(value_max-value_min));
					av5=int((value_min-th_l)*av1);
            }
		else {av3=1; av5=0;}   //iPhoneから送られてくる温度差が0以下なら通常の色設定にする
		float avx=av3/diff;
		float avxx=-minValue*avx+av5;	
		
		for (int i=0; i<height; i++) {
		  for (int j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			value4 = (int)value3*avx+avxx;
            if (value4>255) value4=255;
            else if (value4<0) value4=0;
			//value4 = ( ( 255 * ( value3 - min1) ) ) / (max1-min1)   ;
			
			
			if(F_H==0){
				ondox[j][i]=value3;
				output_8.at<uchar>(i,j)= (uchar)(value4&0xFF);  //? Wakaran!!
			}
			else {
				ondox[319-j][i]=value3;
				output_8.at<uchar>(i,319-j)= (uchar)(value4&0xFF);  //? Wakaran!!

			}			
			
		  }
	    }
	    
	    tempXY= 100-(k_b-sqrt(abs(pow(-k_b,2)-4*k_a*(k_c-(float)ondox[touchX][touchY]))))/(2*k_a)*0.5f;   
	    tempXY=tempXY-(temp_b-fpatemp_f)*kei_x;
    
		int x1=col1-4;	if(x1<0	) 	x1=0;
		int x2=col1+4;	if(x2>319) 	x2=319;
		int y1=row1-4;	if(y1<0	) 	y1=0;
		int y2=row1+4;	if(y2>255) 	y2=255;
    
		for(int i=x1;i<=x2;i++)
		{
			for (int j=y1;j<=y2;j++){
					point_l[i-x1][j-y1]= 100-(k_b-sqrt(abs(pow(-k_b,2)-4*k_a*(k_c-(float)ondox[i][j]))))/(2*k_a)*0.5f;   
					point_l[i-x1][j-y1]=point_l[i-x1][j-y1]-(temp_b-fpatemp_f)*kei_x;
					
					if(j==y1+4 && i==x1+4) tempXY=point_l[i-x1][j-y1];
				
			}
        }
    
    if(thermals.size()>0){
        //printf("BBBB--------Thermals.size()=%i\n",thermals.size());
          thermals1.clear();
            for(int t=0;t<thermals.size();t++){
                int col2=thermals[t].thermal9x9X;
                int row2=thermals[t].thermal9x9Y;
                int x1=col2-16;    if(x1<0    )     x1=0;
                int x2=col2+16;    if(x2>319)     x2=319;
                int y1=row2-32;    if(y1<0    )     y1=0;
                int y2=row2+32;    if(y2>255)     y2=255;
                float max_min=0;
                for(int i=x1;i<=x2;i++)
                {
                    for (int j=y1;j<=y2;j++){
                        float max_1=(float)ondox[i][j];
                        if (max_1 >max_min) max_min=max_1;
                    }
                }
                
                max_min= 100-(k_b-sqrt(abs(pow(-k_b,2)-4*k_a*(k_c-max_min))))/(2*k_a)*0.5f;
                max_min=max_min-(temp_b-fpatemp_f)*kei_x;
                
                yyth.thermal_X=max_min;
                yyth.thermal9x9X=col2;
                yyth.thermal9x9Y=row2;
                yyth.x=thermals[t].x;
                yyth.y=thermals[t].y;
                thermals1.emplace_back(yyth);
           //     printf("D:E:F~=%f ,%i ,%i\n",yyth.thermal_X,yyth.thermal9x9X,yyth.thermal9x9Y);
                
            }
        thermals.clear();
        }
}
//--------------------------------------------------------------

class XxBoson:public ofThread{
public:

  void threadedFunction(){
	
      while(1){
		Mat thermal16(height, width, CV_16U, buffer_start);   // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode
		Mat thermal16_linear(height,width, CV_8U, 1);

		if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0){
			perror(RED "VIDIOC_QBUF" WHT);
            abort();
		}
		if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
			perror(RED "VIDIOC_QBUF" WHT);
            abort();
		}
			AGC_Basic_Linear(thermal16, thermal16_linear, height, width);
			
            lock();
                Mat matx;
                thermal16_linear.convertTo(matx,CV_8UC1);
                img.setFromPixels(matx.ptr(), matx.cols, matx.rows,OF_IMAGE_GRAYSCALE,true);  //SAIGO ga true!!  false zyanai
//                img.update();
            unlock();
		}
  }
};
XxBoson bosonImage;
//--------------------------------------------------------------
void ofApp::setup(){
    keisu();

    
    //font.load("verdana.ttf", 20, true, true, true);
    //myImage1.load("WTlogoH.png");

	ofSetVerticalSync(true);
	HOST=arg_x;
	sender.setup(HOST, SPORT);
    receiver.setup( RPORT );
    
	img.allocate(320,256,OF_IMAGE_GRAYSCALE);
    img1.allocate(320,256/2,OF_IMAGE_GRAYSCALE);

	IP_Adr(Pi3_ip);printf("Pi　Zero IP= %s\n",Pi3_ip);
	int ans=system("sudo chmod a+rwx /dev/ttyACM0");
   
    //font.load("verdana.ttf", 20, true, true, true);
    //myImage1.load("kanden.png");
    //myImage1.load("WTlogoH.png");
	ofSetFrameRate(23); // run at 60 fps
	ofSetVerticalSync(true);
	HOST=arg_x;
	sender.setup(HOST, SPORT);
    receiver.setup( RPORT );

    //---以下　BOSON設定　BOSON設定はset upで行わないと　Bosonが起動しない（time outになりENTERキー入力が必要になる）
    FLR_RESULT result;
    result = Initialize(24, 921600); // /dev/ttyACM0, 921600 baud
    uint32_t camera_sn;
    result = bosonGetCameraSN(&camera_sn);
    result = bosonSetGainMode(FLR_BOSON_HIGH_GAIN);  //Syoki=HIGH
    open_port(2300, 115200);
    
    compression_params.push_back(IMWRITE_PXM_BINARY);
    
    sprintf(video, "/dev/video0");
    sprintf(thermal_sensor_name, "Boson_320");

    video_mode=RAW16;
    //video_mode=YUV;
    
    // Printf Sensor defined
    printf(WHT ">>> " YEL "%s" WHT " selected\n", thermal_sensor_name);
    
    // We open the Video Device
    printf(WHT ">>> " YEL "%s" WHT " selected\n", video);
    if((fd = open(video, O_RDWR)) < 0){
        perror(RED "Error : OPEN. Invalid Video Device" WHT "\n");
        abort();
    }
    
    // Check VideoCapture mode is available
    if(ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0){
        perror(RED "ERROR : VIDIOC_QUERYCAP. Video Capture is not available" WHT "\n");
        abort();
    }
    
    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
        fprintf(stderr, RED "The device does not handle single-planar video capture." WHT "\n");
        abort();
    }
    
    struct v4l2_format format;
    
    // Two different FORMAT modes, 8 bits vs RAW16
    if (video_mode==RAW16) {
        printf(WHT ">>> " YEL "16 bits " WHT "capture selected\n");
        
        // I am requiring thermal 16 bits mode
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
        
        // Select the frame SIZE (will depend on the type of sensor)
        switch (my_thermal) {
            case Boson320:  // Boson320
                width=320;
                height=256;
                break;
            case Boson640:  // Boson640
                width=640;
                height=512;
                break;
            default:  // Boson320
                width=320;
                height=256;
                break;
        }
        
    } else { // 8- bits is always 640x512 (even for a Boson 320)
        printf(WHT ">>> " YEL "8 bits " WHT "YUV selected\n");
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420; // thermal, works   LUMA, full Cr, full Cb
        width = 640;
        height = 512;
    }
    // Common varibles
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    
    // request desired FORMAT
    if(ioctl(fd, VIDIOC_S_FMT, &format) < 0){
        perror(RED "VIDIOC_S_FMT" WHT);
        abort();
    }
    
    struct v4l2_requestbuffers bufrequest;
    bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufrequest.memory = V4L2_MEMORY_MMAP;
    bufrequest.count = 1;   // we are asking for one buffer
    
    if(ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0){
        perror(RED "VIDIOC_REQBUFS" WHT);
        abort();
    }
    memset(&bufferinfo, 0, sizeof(bufferinfo));
    
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = 0;
    
    if(ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0){
        perror(RED "VIDIOC_QUERYBUF" WHT);
        abort();
    }
    printf(WHT ">>> Image width  =" YEL "%i" WHT "\n", width);
    printf(WHT ">>> Image height =" YEL "%i" WHT "\n", height);
    printf(WHT ">>> Buffer lenght=" YEL "%i" WHT "\n", bufferinfo.length);
    
    buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,MAP_SHARED, fd, bufferinfo.m.offset);
    
    if(buffer_start == MAP_FAILED){
        perror(RED "mmap" WHT);
        abort();
    }
    memset(buffer_start, 0, bufferinfo.length);
    type = bufferinfo.type;
    if(ioctl(fd, VIDIOC_STREAMON, &type) < 0){
        perror(RED "VIDIOC_STREAMON" WHT);
        abort();
    }
    
    //int s=system("ls");
    usleep(1000);
	
	luma_height = height+height/2;
	luma_width = width;
	color_space = CV_8UC1;
	bosonImage.startThread();
    
    ttt10=ofGetElapsedTimef();
}
//--------------------------------------------------------------
void ofApp::update(){
//	cout << "FPS= " << ofToString(ofGetFrameRate(),2) << "\n";
    coun_t=0;
	FLR_RESULT result;
	int16_t temp_c_x10;
    //uint16_t temp_k_x10;
    b_count=0;  //yolo count reset!!
	result =bosonlookupFPATempDegCx10(&temp_c_x10);
	aux=(float)temp_c_x10;
	fpatemp_f=aux/10.0f;
    //result=bosonlookupFPATempDegKx10(&temp_k_x10);
    //fpatemp_k=(float)temp_k_x10;
    // send as a binary blob
    // OSCblobの制限により320x256画像を上下２分割して送る。OSCアドレスは別とする
    //ofBufferは別宣言しないとblobを送れない？念のためofxOscMessageも別変数名とした
    ofxOscMessage m;
    bosonImage.lock();
    unsigned char *data1;
    unsigned char *data2;
    uint8_t dataS[40960];
    int xxx;
    data1=img.getPixels().getData();
    data2=dataS;
        m.setAddress("/i");
        for (int i=0;i<320;++i){
            for(int j=0;j<128;++j){
                xxx=j*320+i;
               // if(data1[xxx]<=0)data1[xxx]=1;
               // if(data1[xxx]>=255)data1[xxx]=255;
                data2[xxx]=data1[xxx];
            }
        }
        ofBuffer buffer((char *)data2, 40960);
        m.addBlobArg(buffer);
        sender.sendMessage(m);
    ofxOscMessage m1;
    m1.setAddress("/j");
    for (int i=0;i<320;++i){
        for(int j=0;j<128;++j){
            xxx=j*320+i;
            //if(data1[xxx+40960]<=0)data1[xxx+40960]=1;
            //if(data1[xxx+40960]>=255)data1[xxx+40960]=255;
            data2[xxx]=data1[xxx+40960];
        }
    }
    ofBuffer buffer1((char *)data2, 40960);
    m1.addBlobArg(buffer1);
    sender.sendMessage(m1);
    bosonImage.unlock();
	
	while( receiver.hasWaitingMessages() ){
        ofxOscMessage m;
        receiver.getNextMessage(m);
        string GM=m.getAddress() ;
        //*********Resive Point & Config
        if(GM == "/t" ){   //３種類の温度受信処理
			touchX  = m.getArgAsInt(0);
			touchY  = m.getArgAsInt(1);
			sl_256  = m.getArgAsFloat(2);  //Dynamic range
			F_H		= m.getArgAsInt(3);    //Sayu Hanten 
			th_l    = m.getArgAsInt(4);    //Color renge
			th_h    = m.getArgAsInt(5);
			//printf("X=%i  Y=%i\n",touchX,touchY);
         }
        else if(GM == "/u" ){   //AIで認識した複数ボックスの芯
           thermals.clear();
           coun_t  = m.getArgAsInt(0);
            
          //  printf(" baaaaka!!!!!!=================%i\n",coun_t);
           for(int t=0;t<coun_t;t++){
               xxth.thermal9x9X=    m.getArgAsInt(t*4+1);
               xxth.thermal9x9Y=    m.getArgAsInt(t*4+2);
               xxth.x=              m.getArgAsInt(t*4+3);
               xxth.y=              m.getArgAsInt(t*4+4);
               
               thermals.emplace_back(xxth);
               //printf("A:B:C~=%i ,%i ,%i ,%i ,%i\n",coun_t,xxth.thermal9x9X,xxth.thermal9x9Y,xxth.x,xxth.y);
           }
        //printf("AAAA--------Thermals.size()=%i\n",thermals.size());
        }
			
        }		
        //*******Send  Ondo
		ofxOscMessage mm;
			mm.setAddress("/m");
            mm.addFloatArg(value_min1);
            mm.addFloatArg(value_max1);
			mm.addFloatArg(value_min);
			mm.addFloatArg(value_max);
			mm.addFloatArg(tempXY);
			mm.addStringArg(Pi3_ip);
			sender.sendMessage(mm);
		//********Send 9X9 Ondo
		ofxOscMessage mx;
			mx.setAddress("/x");
			for(int i=0;i<9;i++)
			{
				for (int j=0;j<9;j++) mx.addFloatArg(point_l[i][j]);
			}
			sender.sendMessage(mx);
    //printf("CCC--------Thermals1.size()=%i\n",thermals1.size());
    if(thermals1.size()>0){
        ofxOscMessage my;
        my.setAddress("/v");
        my.addIntArg(thermals1.size());
        for(int t=0;t<thermals1.size();t++){
            my.addFloatArg(thermals1[t].thermal_X);
            my.addIntArg(thermals1[t].thermal9x9X);
            my.addIntArg(thermals1[t].thermal9x9Y);
            my.addIntArg(thermals1[t].x);
            my.addIntArg(thermals1[t].y);
           //*******printf("G:H:I~=%f ,%i ,%i\n",thermals1[t].thermal_X,thermals1[t].thermal9x9X,thermals1[t].thermal9x9Y);
           
        }
        sender.sendMessage(my);
        //printf("AAAA--------Thermals1.size()=%i\n",thermals1.size());
        thermals1.clear();
    }
    
          
}
//--------------------------------------------------------------
void ofApp::draw(){
	ofBackgroundGradient(0, 0);
	//確認用なので最低表示とする
    img.update();
	img.draw(25,25);
	ofSetColor( 255,255,255);
}
//--------------------------------------------------------------
void ofApp::keyPressed(int key){

	
}
//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
