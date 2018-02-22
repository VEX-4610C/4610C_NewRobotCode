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
	wait1Msec(150);
	rollerSetpoint = rollerOut;
	mobileUp();
	gyroturn(SensorValue[gyro], 1);
	SensorValue[gyro] = 0;
	rollerSetpoint = rollerIn;
	chainBarSetpoint = chainBarDown;
	chainBarSetpoint = chainBarIntake+50;
	wait1Msec(100);
	setAllMotors(127);
	wait1Msec(200);
	setAllMotors(0);
	wait1Msec(350);
	gyroturn(SensorValue[gyro], 1);
	SensorValue[gyro] = 0;
}
void mobileGoalTenAuto(int wall, int sh)
{
	mobileandcone(wall, sh*2);
	int turnMult = (wall == LEFT ? 1 : -1);
	activateAutoStacker = 1;
	if(sh)
	{
		degmove(-14);
		gyroturn(450 * turnMult, 1);
		chainBarSetpoint = chainBarIntake+150;
		rollerSetpoint = rollerIn;
		wait1Msec(200);
		setAllMotors(127);
		wait1Msec(150);
		setAllMotors(0);
		wait1Msec(1000);
		finishStack = 1;
		activateAutoStacker = 1;
		SensorValue[gyro] = 0;
		gyroturn(-1700 * turnMult, 1);
		degmove(17);
		while(activateAutoStacker) { wait1Msec(20); }
		rollerSetpoint = rollerOut;
		wait1Msec(150);
		mobileDown();
		degmove(-12);
		doubleSetpoint = 0;
		chainBarSetpoint = 1400;
	}
	else
	{
		gyroturn(SensorValue[gyro], 1);
		degmove(-40);
		doubleSetpoint = 0;
		chainBarSetpoint = 1400;
		gyroturn(1700 * turnMult, 1);

		mobileDown();
		degmove(-10);
	}

}

void mobileGoalTwenAuto(int wall)
{
	//mobileandcone(wall, 0);
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
	degmove(48);
	chainBarSetpoint = 2100;
	wait1Msec(150);
	rollerSetpoint = rollerOut;
	mobileUp();
	gyroturn(SensorValue[gyro], 1);

	int turnMult = (wall == LEFT ? 1 : -1);
	degmove(-46);
	chainBarSetpoint = chainBarStack;
	gyroturn(-420 * turnMult, 1);
	degmove(-20);
	rollerSetpoint = rollerOut;
	gyroturn(-855 * turnMult, 1);
	chainBarSetpoint = 1350;
	rollerSetpoint = rollerStop;
	setAllMotors(127);
	wait1Msec(750);
	setAllMotors(-12);
	wait1Msec(25);
	setAllMotors(0);
	mobileDown();
	wait1Msec(100);
	setAllMotors(127);
	wait1Msec(75);
	setAllMotors(0);
	setAllMotors(-127);
	wait1Msec(150);
	mobileGoalSetpoint = mobileGoalUp;
	wait1Msec(500);
	setAllMotors(0);
}


void programmingSkills()
{
	int HalfTurn = 1725;
	doubleSetpoint = 150;
	SensorValue[gyro] = 0;
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	startTask(autoStacker);
	chainBarSetpoint = 2000;
	mobileDown();
	doubleSetpoint = 0;
	rollerSetpoint = 25;
	SensorValue[gyro] = 0;
	degmove(54);
	rollerSetpoint = rollerOut;
	mobileUp();
	rollerSetpoint = rollerStop;
	chainBarSetpoint = 1400;
	gyroturn(HalfTurn, 1);
	degmove(45);
	mobileDown();

	degmove(-45);
	mobileUp();
	gyroturn(HalfTurn/2, 0);
	mobileDown();
	degmove(35);
	mobileUp();
	gyroturn(-HalfTurn/2, 1);
	degmove(35);
	gyroturn(-HalfTurn/2, 1);
	degmove(22);
	gyroturn(HalfTurn/2, 1);
	setAllMotors(127);
	wait1Msec(700);
	setAllMotors(-12);
	wait1Msec(25);
	setAllMotors(0);
	mobileDown();
	wait1Msec(100);
	setAllMotors(127);
	wait1Msec(75);
	setAllMotors(0);
	mobileUp();
	degmove(-30);

	gyroturn(-HalfTurn/2, 0);
	degmove(-35);
	gyroturn(-HalfTurn/4, 0);
	setAllMotors(-63);
	wait1Msec(250);
	setAllMotors(0);
	wait1Msec(500);
	degmove(40);
	gyroturn(-HalfTurn/4, 0);
	mobileDown();
	degmove(40);
	mobileUp();
	degmove(35);
	gyroturn(HalfTurn/2, 1);
	degmove(22);
	gyroturn(HalfTurn/2, 1);
	setAllMotors(127);
	wait1Msec(700);
	setAllMotors(-12);
	wait1Msec(25);
	setAllMotors(0);
	mobileDown();
	wait1Msec(100);
	setAllMotors(127);
	wait1Msec(75);
	setAllMotors(0);
	mobileUp();
	degmove(-30);
	gyroturn(-3*HalfTurn/4, 0);
	degmove(120);

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
