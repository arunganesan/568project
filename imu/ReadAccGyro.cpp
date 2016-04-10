/******************************************************************************************
* Test Program: Mac OSX / Unix / Linux C++ Interface for Razor AHRS v1.4.2
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
******************************************************************************************/

#include <iostream>   // cout()
#include <iomanip>    // setprecision() etc.
#include <stdexcept>  // runtime_error
#include <cstdio>     // getchar()
#include "RazorAHRS.h"
#include <time.h>

using namespace std;


// Set your serial port here!
//const string serial_port_name = "/dev/tty.FireFly-6162-SPP"; 
//const string serial_port_name = "/dev/tty.usbserial-A700eEhN";
const string serial_port_name = "/dev/ttyUSB0"; // a good guess on linux


// Razor error callback handler
// Will be called from (and in) Razor background thread!
void on_error(const string &msg)
{
  cout << "  " << "ERROR: " << msg << endl;
  
  // NOTE: make a copy of the message if you want to save it or send it to another thread. Do not
  // save or pass the reference itself, it will not be valid after this function returns! 
}

// Razor data callback handler
// Will be called from (and in) Razor background thread!
// 'data' depends on mode that was set when creating the RazorAHRS object. In this case 'data'
// holds 3 float values: yaw, pitch and roll.
//
clock_t t1, t2; 
double dt=0.0;
double  yaw=0, pitch=0, roll=0;
bool firstReading;
double scalar = 14.375;
int count = 0;
double accX=0,accY=0,accZ=0, gyroYaw=0, gyroPitch=0, gyroRoll=0;
double offset_accX=0,offset_accY=0,offset_accZ=0, offset_gyroYaw=0, offset_gyroPitch=0, offset_gyroRoll=0;
double velocity_y = 0;
int numAvg = 100;
void on_data(const float data[])
{
   
    // store data
    accX = data[1];
    accY = data[0];
    accZ = data[2];

    //offset Gyro
    gyroYaw = data[6];
    gyroPitch = data[7];
    gyroRoll = data[8];

   
    count++;
    firstReading = true;
    if (count <= numAvg){
        t1 = clock();
        firstReading = false;

        //offset Acc
        offset_accX += accX;
        offset_accY += accY;
        offset_accZ += accZ;

        //offset Gyro
        offset_gyroYaw += gyroYaw;
        offset_gyroPitch += gyroPitch;
        offset_gyroRoll += gyroRoll;
    }
    else if (count == numAvg+1){
        offset_accX /= numAvg;
        offset_accY /= numAvg;
        offset_accZ /= numAvg;

        //offset Gyro
        offset_gyroYaw /= numAvg;
        offset_gyroPitch /= numAvg;
        offset_gyroRoll /= numAvg;

    }
//  cout << "  " << fixed << setprecision(1) 
//  << "Yaw = " << setw(6) << data[0] << "      Pitch = " << setw(6) << data[1] << "      Roll = " << setw(6) << data[2] << endl;

  // NOTE: make a copy of the yaw/pitch/roll data if you want to save it or send it to another
  // thread. Do not save or pass the pointer itself, it will not be valid after this function
  // returns!
  
  // If you created the Razor object using RazorAHRS::ACC_MAG_GYR_RAW or RazorAHRS::ACC_MAG_GYR_CALIBRATED
  // instead of RazorAHRS::YAW_PITCH_ROLL, 'data' would contain 9 values that could be printed like this:
  
  // cout << "  " << fixed << setprecision(3)
  // << "ACC = " << setw(6) << (data[0]/256.0)*9.81 << ", " << setw(6) << (data[1]/256.0)*9.81 << ", " << setw(6) << (data[2]/256.0*9.81)
  // << "        MAG = " << setw(7) << data[3] << ", " << setw(7) << data[4] << ", " << setw(7) << data[5]
  // << "        GYR = " << setw(7) << data[6] << ", " << setw(7) << data[7] << ", " << setw(7) << data[8] << endl;
  
  // cout << "  " << fixed << setprecision(10)<< "TIMESTEP: "<<dt<<endl;
    t2 = clock();
    dt = double(t2-t1)/CLOCKS_PER_SEC;
    t1 = t2;

    int tmp1 = 0, tmp2 = 1;

    if (firstReading){
       
        // Subtract Acceleration Readings 
        accX -= offset_accX*tmp1;
        accY -= offset_accY*tmp1;
        accZ -= offset_accZ*tmp1;
        

        // Subtract Gyro Readings
        gyroYaw -= offset_gyroYaw*tmp2;
        gyroPitch -= offset_gyroPitch*tmp2;
        gyroRoll -= offset_gyroRoll*tmp2;

        // Introduce Constants (calibrate this better)
        accX = accX/256.0*9.81;
        accY = accY/256.0*9.81;
        accZ = accZ/256.0*9.81;

        velocity_y += accY*dt;
        
        // Integrate angle
        yaw += 30*gyroYaw/scalar*dt;
        pitch += 30*gyroPitch/scalar*dt;
        roll += 30*gyroRoll/scalar*dt;
        
        //cout << accX << " " << accY << " " << accZ << " " << yaw << " " << pitch << " " << roll << " " <<dt << endl;
        
        cout << accY << " "  << gyroRoll/scalar << " " << dt << endl;
        
        //cout << fixed << setprecision(3) << setw(10) << yaw << setw(10) << pitch << setw(10) <<  roll << endl;
        //cout << accX << " " << accY << " " << accZ << " " << gyroYaw << " " << gyroPitch << " " << gyroRoll << " " <<dt << endl;
        
    }
}

RazorAHRS *razor;
int main()
{
  cout << endl;
  cout << "  " << "Razor AHRS C++ test" << endl;
  cout << "  " << "Connecting..." << endl << endl;
  
  try
  {
    // Create Razor AHRS object. Serial I/O will run in background thread and report
    // errors and data updates using the callbacks on_data() and on_error().
    // We want to receive yaw/pitch/roll data. If we wanted the unprocessed raw or calibrated sensor
    // data, we would pass RazorAHRS::ACC_MAG_GYR_RAW or RazorAHRS::ACC_MAG_GYR_CALIBRATED
    // instead of RazorAHRS::YAW_PITCH_ROLL.
    razor = new RazorAHRS(serial_port_name, on_data, on_error, RazorAHRS::ACC_MAG_GYR_CALIBRATED);
    
    // NOTE: If these callback functions were members of a class and not global
    // functions, you would have to bind them before passing. Like this:
    
    // class Callback
    // {
    //   public:
    //     void on_data(const float ypr[]) { }
    //     void on_error(const string &msg) { }
    // };
    
    // Callback c;
    
    // razor = new RazorAHRS(serial_port_name,
    //    bind(&Callback::on_data, &c, placeholders::_1),
    //    bind(&Callback::on_error, &c, placeholders::_1),
    //    RazorAHRS::YAW_PITCH_ROLL);
    
    // If you're calling from inside of "c" you would of course use "this" instead of "&c".
  }
  catch(runtime_error &e)
  {
    cout << "  " << (string("Could not create tracker: ") + string(e.what())) << endl;
    cout << "  " << "Did you set your serial port in Example.cpp?" << endl;
    return 0;
  }
  
  getchar();  // wait for RETURN key
  return 0;
}

