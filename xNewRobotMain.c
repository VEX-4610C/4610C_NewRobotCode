#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    chainBarPot,    sensorPotentiometer)
#pragma config(Sensor, in2,    mobilePot,      sensorPotentiometer)
#pragma config(Sensor, in3,    gyro,           sensorGyro)
#pragma config(Sensor, in4,    peStatus,       sensorAnalog)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           mobileGoal,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           backLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           doubleLeft,    tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_3)
#pragma config(Motor,  port5,           chainbar,      tmotorVex393_MC29, openLoop, encoderPort, I2C_4)
#pragma config(Motor,  port6,           doubleRight,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           claw,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           frontLeft,     tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port9,           backRight,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          frontRight,    tmotorVex393_HBridge, openLoop, encoderPort, I2C_2)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)
// Sign is a ternary statment
#define sign(x) ((x) < 0 ? -1 : 1)
// toggle -- 1 -> (1-1)=0;  0 -> (1-0)=1
#define toggle(x) (1 - (x))
#define toggleNeg(x) ((x) == 1 ? -1 : 1)
#define min(x,y) ((x) > (y) ? (y) : (x))
#define max(x,y) ((x) > (y) ? (x) : (y))
#define resetButton (vexRT[Btn7R])
#include "Vex_Competition_Includes.c"
/* include "zSmartMotorLib.c" */
#define SetMotor(x, y) (motor[(x)] = (y))
#define SmartMotorRun() ;
#include "zBCIPIDLib.c"
#include "zAutonomousFunctions.c"
#include "zAutonomousRoutines.c"

int RUNTEST = 1, TEST = 3; // Manual Autonomous Test Controls
void pre_auton()
{
	startTask(batLevel, 0);
	nMotorEncoder[frontLeft] = 0;
	nMotorEncoder[frontRight] = 0;
	nMotorEncoder[doubleLeft] = 0;
	nMotorEncoder[chainbar] = 0;
	bStopTasksBetweenModes = true;
	if(!RUNTEST)
		LcdAutonomousSelection();
	motor[claw] = clawClosed;
}
task autonomous()
{
	// SmartMotorRun();
	startTask(batLevel, 0);
	if(RUNTEST == 1 && TEST == 0)
		testDegmove();
	if(RUNTEST == 1 && TEST == 1)
		testSmallGyroturn();
	if(RUNTEST == 1 && TEST == 2)
		testLargeGyroturn();
	if(MyAutonomous == 0 || (RUNTEST == 1 && TEST == 3))
		mobileGoalTenAuto();
	if(MyAutonomous == 1 || (RUNTEST == 1 && TEST == 4))
		mobileGoalTwenAuto(LEFT);
	if(MyAutonomous == 2 || (RUNTEST == 1 && TEST == 5))
		mobileGoalTwenAuto(RIGHT);
	if(MyAutonomous == 3 || (RUNTEST == 1 && TEST == 6))
		programmingSkills();
}
int EMERGENCY_MODE = 0;
int left = 0, right = 0;
int lastManualMobileGoal = 0;
int lastManualLift = 0;
int lastManualChainBar = 0;
task usercontrol()
{
	//SmartMotorRun();
	startTask(WATCHDOG);
	startTask(batLevel, 0);
	while (true)
	{
		// Drive Code
		left   = abs(vexRT[Ch3]);
		right  = abs(vexRT[Ch2]);
		left   = left < 15 ? 0 : left;
		right  = right < 15 ? 0 : right;
		left   = (5*left*left   + 305*left  + 7624)/1000;
		right  = (5*right*right + 305*right + 7624)/1000;
		motor[frontLeft] = motor[backLeft]   = ((int) left) * sign(vexRT[Ch3]);
		motor[frontRight] = motor[backRight] = ((int) right) * sign(vexRT[Ch2]);

		// Mobile Goal
		if(EMERGENCY_MODE && vexRT[Btn7D])
		{
			mobilePIDActive = 0;
			lastManualMobileGoal = 1;
			SetMotor(mobileGoal, 127);
		}
		else if(EMERGENCY_MODE && vexRT[Btn8D])
		{
			mobilePIDActive = 0;
			lastManualMobileGoal = 1;
			SetMotor(mobileGoal, -127);
		}
		else if(mobilePIDActive == 0 && lastManualMobileGoal == 1)
		{
			mobilePIDActive = 1;
			mobileGoalSetpoint = nMotorEncoder[mobileGoal];
		}
		else if(vexRT[Btn7D])
		{
			mobilePIDActive = 1;
			mobileGoalSetpoint = mobileGoalDown;
		}
		else if(vexRT[Btn8D])
		{
			mobilePIDActive = 1;
			mobileGoalSetpoint = mobileGoalUp;
		}

		// Lift (Autostack and Manual)
		if(vexRT[Btn8L])
		{
			while(vexRT[Btn8L]) { wait1Msec(20); }
			doubleStackLoader = toggle(doubleStackLoader); // 0 -> 1-0 = 1 ; 1 -> 1-1 = 0 -- Toggle Shorthand
		}
		if(vexRT[Btn5U])
		{
			while(vexRT[Btn5U]) { wait1Msec(20); }
			currentStacked--;
		}

		if(vexRT[Btn7L])
		{
			doublePIDActive = 0;
			SetMotor(doubleLeft, -127);
			SetMotor(doubleRight, -127);
			lastManualLift = 1;
			activateAutoStacker = 0;
		}
		else if(vexRT[Btn8R])
		{
			doublePIDActive = 0;
			SetMotor(doubleLeft, 127);
			SetMotor(doubleRight, 127);
			lastManualLift = 1;
			activateAutoStacker = 0;
		}
		else if(vexRT[Btn6U])
		{
			doublePIDActive = 1;
			activateAutoStacker = 1;
		}
		else if(doublePIDActive == 0 && lastManualLift == 1)
		{
			doublePIDActive = 1;
			doubleSetpoint = nMotorEncoder[doubleLeft];
		}
		else
		{
			doublePIDActive = 1;
		}

		if(vexRT[Btn7U])
		{
			chainBarPIDActive = 0;
			motor[chainbar] = -127;
			lastManualChainBar = 1;
		}
		else if(vexRT[Btn8U])
		{
			chainBarPIDActive = 0;
			motor[chainbar] = 127;
			lastManualChainBar = 1;
		}
		else
		{
			if(lastManualChainBar)
				chainBarSetpoint = nMotorEncoder[chainbar];
			lastManualChainBar = 0;
			chainBarPIDActive = 1;
			motor[chainbar] = 0;
		}

		// Claw
		if(vexRT[Btn6D])
		{
			clawSetpoint = clawOpen;
			motor[claw] = clawOpen;
		}
		else if(vexRT[Btn5D])
		{
			clawSetpoint = clawClosed;
			motor[claw] = clawClosed;
		}
		// RESET
		if(resetButton)
		{
			doublePIDActive = 1;
			chainBarPIDActive = 1;
			mobilePIDActive = 1;
			doubleSetpoint = doubleDown;
			chainBarSetpoint = chainBarDown;
			mobileGoalSetpoint = mobileGoalDown;
			activateAutoStacker = 0;
		}
		if(vexRT[Ch1] > 100 && vexRT[Ch4] < -100)
		{
			doubleSetpoint = doubleFixedGoal;
			chainBarSetpoint = chainBarPassPos;
		}
	}

}
