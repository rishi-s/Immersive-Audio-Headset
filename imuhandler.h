#ifndef IMU_HANDLER_H_
#define IMU_HANDLER_H_

#include <Bela.h>


/*----------*/
/*----------*/
/* IMU #includes*/
#include <rtdk.h>
#include "Bela_BNO055.h"
/*----------*/
/*----------*/

/*----------*/
/*----------*/
/*IMU #variables*/

// Change this to change how often the BNO055 IMU is read (in Hz)
int readInterval = 100;

I2C_BNO055 bno; // IMU sensor object
int buttonPin = 1; // calibration button pin
int lastButtonValue = 0; // using a pulldown resistor

// Quaternions and Vectors
imu::Quaternion gCal, gCalLeft, gCalRight, gIdleConj = {1, 0, 0, 0};
imu::Quaternion qGravIdle, qGravCal, quat, steering, qRaw;

imu::Vector<3> gRaw;
imu::Vector<3> gGravIdle, gGravCal;
imu::Vector<3> ypr; //yaw pitch and roll angles


int calibrationState = 0; // state machine variable for calibration
int setForward = 0; // flag for setting forward orientation

// variables handling threading
AuxiliaryTask i2cTask;		// Auxiliary task to read I2C
AuxiliaryTask gravityNeutralTask;		// Auxiliary task to read gravity from I2C
AuxiliaryTask gravityDownTask;		// Auxiliary task to read gravity from I2C

int readCount = 0;			// How long until we read again...
int readIntervalSamples = 0; // How many samples between reads

int printThrottle = 0; // used to limit printing frequency

// function declarations
void readIMU(void*);
void getNeutralGravity(void*);
void getDownGravity(void*);
void calibrate();
void resetOrientation();
/*----------*/
/*----------*/


/*----------*/
/*----------*/
/*IMU #setup routine*/
bool setupIMU(int sampleRate) {
  {
    if(!bno.begin(2)) {
      rt_printf("Error initialising BNO055\n");
      return false;
    }

    rt_printf("Initialised BNO055\n");

    // use external crystal for better accuracy
      bno.setExtCrystalUse(true);

    // get the system status of the sensor to make sure everything is ok
    uint8_t sysStatus, selfTest, sysError;
      bno.getSystemStatus(&sysStatus, &selfTest, &sysError);
    rt_printf("System Status: %d (0 is Idle)   Self Test: %d (15 is all good)   System Error: %d (0 is no error)\n", sysStatus, selfTest, sysError);

    // set sensor reading in a separate thread
    // so it doesn't interfere with the audio processing
    i2cTask = Bela_createAuxiliaryTask(&readIMU, 5, "bela-bno");
    readIntervalSamples = sampleRate / readInterval;

    gravityNeutralTask = Bela_createAuxiliaryTask(&getNeutralGravity, 5, "bela-neu-gravity");
    gravityDownTask = Bela_createAuxiliaryTask(&getDownGravity, 5, "bela-down-gravity");


  }
  return true;
}

/*----------*/
/*----------*/

/*----------*/
/*----------*/
/*IMU scheduler*/

bool scheduleIMU(){
  {
    // this schedules the imu sensor readings
    if(++readCount >= readIntervalSamples) {
      readCount = 0;
      Bela_scheduleAuxiliaryTask(i2cTask);
      checkOSC();
    }

    // print IMU values, but not every sample
    printThrottle++;
    if(printThrottle >= 4100){
      //rt_printf("Tracker Value: %d %d %d \n",gVBAPTracking[0],gVBAPTracking[1],gVBAPTracking[2]); //print horizontal head-track value
      //rt_printf("%f %f %f\n", ypr[0], ypr[1], ypr[2]);
      //rt_printf("Positions Update: %d %d\n",gVBAPUpdatePositions[0],gVBAPUpdatePositions[9]); //print horizontal head-track value
      imu::Vector<3> qForward = gIdleConj.toEuler();
      printThrottle = 0;
    }

    //read the value of the button
    int buttonValue = gCalibrate;

    // if button wasn't pressed before and is pressed now
    if( buttonValue != lastButtonValue && buttonValue == 1 ){
      // then run calibration to set looking forward (gGravIdle)
      // and looking down (gGravCal)
      switch(calibrationState) {
      case 0: // first time button was pressed
        setForward = 1;
        // run task to get gravity values when sensor in neutral position
        Bela_scheduleAuxiliaryTask(gravityNeutralTask);
        calibrationState = 1;	// progress calibration state
        break;
      case 1: // second time button was pressed
        // run task to get gravity values when sensor 'looking down' (for head-tracking)
        Bela_scheduleAuxiliaryTask(gravityDownTask);
        calibrationState = 0; // reset calibration state for next time
        break;
      }
    }
    lastButtonValue = buttonValue;
  }
return true;
}
/*----------*/
/*----------*/


/*----------*/
/*----------*/
/* Auxiliary task to read from the I2C board*/
void readIMU(void*)
{
	// get calibration status
	uint8_t sys, gyro, accel, mag;
	bno.getCalibration(&sys, &gyro, &accel, &mag);
	// status of 3 means fully calibrated
	//rt_printf("CALIBRATION STATUSES\n");
	//rt_printf("System: %d   Gyro: %d Accel: %d  Mag: %d\n", sys, gyro, accel, mag);

	// quaternion data routine from MrHeadTracker
  	imu::Quaternion qRaw = bno.getQuat(); //get sensor raw quaternion data

  	if( setForward ) {
  		gIdleConj = qRaw.conjugate(); // sets what is looking forward
  		setForward = 0; // reset flag so only happens once
  	}

  	steering = gIdleConj * qRaw; // calculate relative rotation data
  	quat = gCalLeft * steering; // transform it to calibrated coordinate system
  	quat = quat * gCalRight;

  	ypr = quat.toEuler(); // transform from quaternion to Euler
}

// Auxiliary task to read from the I2C board
void getNeutralGravity(void*) {
	// read in gravity value
  	imu::Vector<3> gravity = bno.getVector(I2C_BNO055::VECTOR_GRAVITY);
  	gravity = gravity.scale(-1);
  	gravity.normalize();
  	gGravIdle = gravity;
}

// Auxiliary task to read from the I2C board
void getDownGravity(void*) {
	// read in gravity value
  	imu::Vector<3> gravity = bno.getVector(I2C_BNO055::VECTOR_GRAVITY);
  	gravity = gravity.scale(-1);
  	gravity.normalize();
  	gGravCal = gravity;
  	// run calibration routine as we should have both gravity values
  	calibrate();
}

// calibration of coordinate system from MrHeadTracker
// see http://www.aes.org/e-lib/browse.cfm?elib=18567 for full paper
// describing algorithm
void calibrate() {
  	imu::Vector<3> g, gravCalTemp, x, y, z;
  	g = gGravIdle; // looking forward in neutral position

  	z = g.scale(-1);
  	z.normalize();

  	gravCalTemp = gGravCal; // looking down
  	y = gravCalTemp.cross(g);
  	y.normalize();

  	x = y.cross(z);
  	x.normalize();

  	imu::Matrix<3> rot;
  	rot.cell(0, 0) = x.x();
  	rot.cell(1, 0) = x.y();
  	rot.cell(2, 0) = x.z();
  	rot.cell(0, 1) = y.x();
  	rot.cell(1, 1) = y.y();
  	rot.cell(2, 1) = y.z();
  	rot.cell(0, 2) = z.x();
  	rot.cell(1, 2) = z.y();
  	rot.cell(2, 2) = z.z();

  	gCal.fromMatrix(rot);

  	resetOrientation();
}

// from MrHeadTracker
// resets values used for looking forward
void resetOrientation() {
  	gCalLeft = gCal.conjugate();
  	gCalRight = gCal;
}
/*----------*/
/*----------*/

#endif /* IMU_HANDLER_H_ */
