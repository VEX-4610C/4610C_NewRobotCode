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

#define setAllMotors(x) motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = (x)

void mobileandcone(int wall, int sh)
{
	doubleSetpoint = 150;
	SensorValue[gyro] = 0;
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	startTask(autoStacker);

	chainBarSetpoint = 1750;
	mobileDown();
	doubleSetpoint = 0;
	rollerSetpoint = 25;
	SensorValue[gyro] = 0;
	degmove(48 + sh*2);
	chainBarSetpoint = 2100;
	wait1Msec(100);
	rollerSetpoint = rollerOut;
	mobileUp();
	gyroturn(SensorValue[gyro], 1);
	SensorValue[gyro] = 0;
	wait1Msec(75);
	rollerSetpoint = rollerIn;
	chainBarSetpoint = chainBarDown;
	chainBarSetpoint = chainBarIntake;
	wait1Msec(50);
	setAllMotors(127);
	wait1Msec(350);
		chainBarSetpoint = chainBarIntake+100;
	setAllMotors(0);
	wait1Msec(150);
	gyroturn(SensorValue[gyro], 1);
	SensorValue[gyro] = 0;
}
void mobileGoalTenAuto(int wall, int sh)
{
		//mobileandcone(wall, 2);
doubleSetpoint = 150;
	SensorValue[gyro] = 0;
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	startTask(autoStacker);

	chainBarSetpoint = 1750;
	mobileDown();
	doubleSetpoint = 0;
	rollerSetpoint = 25;
	SensorValue[gyro] = 0;
	degmove(52);
	chainBarSetpoint = 2100;
	wait1Msec(100);
	rollerSetpoint = rollerOut;
	mobileUp();
	gyroturn(SensorValue[gyro], 1);
	SensorValue[gyro] = 0;
	wait1Msec(75);
	int turnMult = (wall == LEFT ? 1 : -1);
	degmove(-35);
	gyroturn(1800, 1);
	setAllMotors(0);
	mobileDown();
	degmove(-15);

}

void mobileGoalTwenAuto(int wall)
{
	//mobileandcone(wall, 2);
doubleSetpoint = 150;
	SensorValue[gyro] = 0;
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	startTask(autoStacker);

	chainBarSetpoint = 1750;
	mobileDown();
	doubleSetpoint = 0;
	rollerSetpoint = 25;
	SensorValue[gyro] = 0;
	degmove(52);
	chainBarSetpoint = 2400;
	wait1Msec(150);
	rollerSetpoint = rollerOut;
	mobileUp();
	gyroturn(SensorValue[gyro], 1);
	SensorValue[gyro] = 0;
	wait1Msec(75);
	int turnMult = (wall == LEFT ? 1 : -1);
	degmove(-45);
	chainBarSetpoint = chainBarStack;
	gyroturn(420, 1);
	//gyroturn((420 * turnMult), 1); // -420 = right
	degmove(-18);
	rollerSetpoint = rollerOut;
	gyroturn(935, 1); // -935 = right
	chainBarSetpoint = 1350;
	rollerSetpoint = rollerStop;
	setAllMotors(127);
	wait1Msec(1000);
	setAllMotors(-12);
	wait1Msec(25);
	setAllMotors(0);
	mobileDown();
	wait1Msec(50);
	setAllMotors(127);
	wait1Msec(225);
	setAllMotors(0);
	wait1Msec(150);
	setAllMotors(-127);
	wait1Msec(150);

	mobileGoalSetpoint = mobileGoalUp;
	wait1Msec(500);
	setAllMotors(0);
}


void programmingSkills()
{
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	doubleSetpoint = 250;
	chainBarSetpoint = 1850;
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
void stationaryAuto()
{
	doubleSetpoint = 150;
	SensorValue[gyro] = 0;
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	doubleSetpoint = 600;
	chainBarSetpoint = 1800;
	wait1Msec(200);
	while(!doubleDone) { wait1Msec(20); }
	degmove(27);
	doubleSetpoint = 300;
	wait1Msec(300);
	rollerSetpoint = rollerOut;
	wait1Msec(500);
	doubleSetpoint = 600;
	wait1Msec(250);
	degmove(-10);
	doubleSetpoint = 0;
	rollerSetpoint = rollerStop;
}
