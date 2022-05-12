#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
 
#include <stdlib.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include<iostream>
extern "C" {
#include "dynamixel_sdk.h"
 
    unsigned int vel_convert(int speed);
    int syncwrite(int port_num, int group_num, int goal_velocity1, int goal_velocity2);
}
 
using namespace std;
using namespace cv;
 
// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING_SPEED            32
 
// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_MOVING_SPEED             2
 
// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
 
// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            
 
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      300                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      600              
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
 
#define ESC_ASCII_VALUE                 0x1b
 
 
int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}
 
int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;
 
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
    ch = getchar();
 
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
 
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
 
    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}
 
 
string codec1 = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), \ 
     width=(int)640, height=(int)360, format=(string)NV12, framerate=(fraction)10/1 ! \
     nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, \
     format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
 
 
int main()
{
    //VIdeo on
    VideoCapture cap(codec1, CAP_GSTREAMER);
    if (!cap.isOpened()) { cout << "Video error" << endl; }
 
    int port_num = portHandler(DEVICENAME);
 
    packetHandler();
 
    int group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, 
            ADDR_MX_MOVING_SPEED, LEN_MX_GOAL_POSITION);
 
    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_addparam_result = false;            // AddParam result
    int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE,
         DXL_MAXIMUM_POSITION_VALUE };  // Goal position
 
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl1_present_position = 0, dxl2_present_position = 0;    
 
    // Open port
    if (openPort(port_num))
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }
 
    // Set port baudrate
    if (setBaudRate(port_num, BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }
 
    // Enable Dynamixel#1 Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, 
        ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }
 
    // Enable Dynamixel#2 Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, 
        ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    }
 
    Mat frame;
    double myproms;
    float angle;
    int x, y;
    Point center;
    while (1)
    {
      int64 t1 = getTickCount();
 
      cap >> frame;
      if (frame.empty())  break;
 
      //전처리
      Mat ROI, HSV;
      ROI = frame(Rect(0, 150, frame.cols, frame.rows - 150));
      cvtColor(ROI, HSV, COLOR_BGR2HSV);
      Scalar lower_white = Scalar(0, 0, 200);
      Scalar upper_white = Scalar(180, 255, 255);
      inRange(HSV, lower_white, upper_white, HSV);
      
      //필터, 에지검출
      Mat filter, canny;
      bilateralFilter(HSV, filter, 5, 100, 100);
      Canny(filter, canny, 60, 120);
 
      Mat lineResult;
      cvtColor(canny, lineResult, COLOR_GRAY2BGR);
      
      //직선검출
      vector<Vec4i> lines;
      HoughLinesP(canny, lines, 1, CV_PI/180, 50, 50, 10);
      for(size_t i = 0; i < lines.size(); i++)
      {
          Vec4i l = lines[i];
          line(lineResult, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(0,0,255), 2, LINE_AA);
      }
 
      static int error = 0;
      center = Point(canny.cols/2, canny.rows);
      if(lines.size() > 1)
      {
        x = lines[0][0] - center.x;
        y = lines[0][1] - center.y;
        angle = (atan(y / (float)x)) * 180 / CV_PI;
        if(angle < -75 || (angle >= 0 && angle < 15)) //직진
        {
            error = 0;
        }
        else if(angle >= 15 && angle < 70) //좌회전
        {
            error = (180 - angle) / 5; 
        }
        else if(angle >= 70) //좌로 살짝
        {
            error = (180 - angle) / 12;
        }
        else if(angle > -50 && angle < 0) //우회전
        {
            error = -((180 + angle) / 5);
        }
        else if(angle < -50) //우로 살짝
        {
            error = -((180 + angle) / 12);
        }
 
        cout << "angle : " << angle << endl;
        cout << "error : " << error << endl;
      }
 
      int L_speed = 80 - error;
      int R_speed = 80 + error;
 
      syncwrite(port_num, group_num, L_speed, -R_speed);
      printf("speed1 : %d, speed2 : %d\n", L_speed, -R_speed);
      usleep(10000);
      
      imshow("line", canny);
 
      int64 t2 = getTickCount();
      myproms = (t2 - t1) * 1000 / getTickFrequency();
      cout << "time: " << myproms << endl;
      if (waitKey(33) == 27) break;
    }
 
    // Disable Dynamixel#1 Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, 
        ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
 
    // Disable Dynamixel#2 Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
 
    // Close port
    closePort(port_num);
 
    return 0;
}
 
//-1023~1023 -> +(CCW) 0~1023, -(CW) 1024~2047
unsigned int vel_convert(int speed)
{
    unsigned int temp;
    if (speed > 1023) speed = 1023;
    else if (speed < -1023) speed = -1023;
 
    if (speed >= 0) temp = (unsigned int)speed;
    else temp = (unsigned int)(-speed + 1023);
 
    return temp;
}
 
int syncwrite(int port_num, int group_num, int goal_velocity1, int goal_velocity2)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_addparam_result = false;            // AddParam result
     // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, DXL1_ID, 
        vel_convert(goal_velocity1), LEN_MX_GOAL_POSITION);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
        return 0;
    }
 
    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, DXL2_ID, 
        vel_convert(goal_velocity2), LEN_MX_GOAL_POSITION);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
        return 0;
    }
 
    // Syncwrite goal position
    groupSyncWriteTxPacket(group_num);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
 
    // Clear syncwrite parameter storage
    groupSyncWriteClearParam(group_num);
    return 0;
 
}
