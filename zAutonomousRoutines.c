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
#define intake() 	chainBarSetpoint = chainBarDown;    \
									rollerSetpoint = rollerIn;         \
									wait1Msec(500);									    \
									rollerSetpoint = rollerStop;        \
									chainBarSetpoint = chainBarPassPos \

#define outtake() rollerSetpoint = rollerOut;         \
									wait1Msec(500);									    \
									rollerSetpoint = rollerStop         \

void mobileGoalTenAuto()
{
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	chainBarSetpoint = 2000;
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(500);
	rollerSetpoint = 25;
	degmove(48);
	rollerSetpoint = rollerOut;
	gyroturn(SensorValue[gyro], 1);
	mobileUp();
	chainBarSetpoint = 1350;
	degmove(-40);
	gyroturn(1700, 1);
	degmove(10);
	mobileDown();
	degmove(-30);
}

void mobileGoalTwenAuto(int wall)
{
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	chainBarSetpoint = 2000;
	int turnMult = wall == LEFT ? 1 : -1;
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(500);
	rollerSetpoint = 25;
	degmove(48);
	rollerSetpoint = rollerOut;
	gyroturn(SensorValue[gyro], 1);
	mobileUp();
	chainBarSetpoint = 1350;
	degmove(-40);
	gyroturn(-450 * turnMult, 1);
	chainBarSetpoint = 1600;
	degmove(-20);
	rollerSetpoint = rollerStop;
	gyroturn(-780 * turnMult, 1);
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 127;
	wait1Msec(1000);
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 0;
	mobileDown();
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 127;
	wait1Msec(200);
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 0;
	mobileGoalSetpoint = mobileGoalUp;
	degmove(-30);
}


void programmingSkills()
{
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
	while(!chainbarDone) { wait1Msec(20); }
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
