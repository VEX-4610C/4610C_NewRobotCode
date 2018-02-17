#define RED 0
#define BLUE 1
#define LEFT 0
#define RIGHT 1
void testDegmove()
{
	degmove(24);
	wait1Msec(250);
	degmove(-48);
	wait1Msec(250);
	degmove(24);
}
void testSmallGyroturn()
{
	gyroturn(900, 0);
	wait1Msec(250);
	gyroturn(-900, 0);
	wait1Msec(250);
	gyroturn(900, 0);
	wait1Msec(250);
	gyroturn(-900, 0);
}
void testLargeGyroturn()
{
	for(int i = 0; i < 4; i++)
	{
		gyroturn(1800, 0);
	}
}
/* Routines to Write
-	7 Point Match Auto
*/
#define mobileUp() mobileDone = 0; \
mobileGoalSetpoint = mobileGoalUp; \
wait1Msec(500); \
while(!mobileDone) { wait1Msec(20); }

#define mobileDown() mobileDone = 0; \
mobileGoalSetpoint = mobileGoalDown; \
wait1Msec(500); \
while(!mobileDone) { wait1Msec(20); }
#define intake() 	chainBarSetpoint = chainBarIntake;    \
rollerSetpoint = rollerIn;         \
wait1Msec(250);									    \
rollerSetpoint = rollerHold;        \
chainBarSetpoint = chainBarDown \

#define outtake() rollerSetpoint = rollerOut;         \
wait1Msec(200);									    \
rollerSetpoint = rollerStop

#define setAllMotors(x) motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = (x)									\

void mobileGoalTenAuto()
{
	SensorValue[gyro] = 0;
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	startTask(autoStacker);
	chainBarSetpoint = 2000;
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(500);
	rollerSetpoint = 25;
	degmove(48);
	gyroturn(SensorValue[gyro], 1);
	mobileUp();
	activateAutoStacker = 1;
	while(activateAutoStacker) { wait1Msec(20); }
	for(int i =0; i<3; i++)
	{
		rollerSetpoint = rollerIn;
		chainBarSetpoint = chainBarIntake;
		wait1Msec(250);
		activateAutoStacker = 1;
		degmove(3);
		while(activateAutoStacker) { wait1Msec(20); }
	}
	degmove(-40);
	gyroturn(1800, 1);
	mobileDown();
	degmove(-30);
}

void mobileGoalTwenAuto(int wall)
{
	doubleSetpoint = 150;
	SensorValue[gyro] = 0;
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	startTask(autoStacker);
	chainBarSetpoint = 2200;
	int turnMult = wall == LEFT ? 1 : -1;
	mobileDown();
	doubleSetpoint = 0;
	rollerSetpoint = 25;
	SensorValue[gyro] = 0;
	degmove(47);
	rollerSetpoint = rollerOut;
	mobileUp();
	rollerSetpoint = rollerHold;
	degmove(-40);
	chainBarSetpoint = chainBarStack;
	gyroturn(-450 * turnMult + SensorValue[gyro], 1);
	degmove(-24);
	rollerSetpoint = rollerStop;
	gyroturn(-1000 * turnMult, 1);
	setAllMotors(127);
	wait1Msec(600);
	setAllMotors(0);
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(300);
	setAllMotors(127);
	wait1Msec(300);
	setAllMotors(0);
	mobileGoalSetpoint = mobileGoalUp;
	degmove(-30);
}


void programmingSkills()
{
	SensorValue[gyro] = 0;
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	doubleSetpoint = 250;
	chainBarSetpoint = 2200;
	startTask(WATCHDOG);
	mobileDown();
	degmove(40);
	outtake();
	mobileUp();
	chainBarSetpoint = 1200;
	while(!chainBarDone) { wait1Msec(20); }
	degmove(-22);
	gyroturn(-900, 1);
	degmove(15);
	gyroturn(-800, 1);
	degmove(25);
	mobileDown();
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 127;
	wait1Msec(1000);
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 0;
	degmove(-15);
	mobileUp();
	degmove(-14);
	gyroturn(625, 0);
	degmove(-60);
}

void disruptAuto()
{
	degmove(-200);
}
