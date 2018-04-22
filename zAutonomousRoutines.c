#define RED 0
#define BLUE 1
#define LEFT 0
#define RIGHT 1
void testDegmove()
{
	degmove(36);
	wait1Msec(100);
	straighten();
	wait1Msec(750);
	degmove(-24);
	wait1Msec(100);
	straighten();
	wait1Msec(750);
	degmove(24);
}
void testSmallGyroturn()
{
	gyroturn(900);
	wait1Msec(250);
	gyroturn(-900);
	wait1Msec(250);
	gyroturn(900);
	wait1Msec(250);
	gyroturn(-900);
}
void testLargeGyroturn()
{
	for(int i = 0; i < 4; i++)
	{
		gyroturn(1800);
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

// Autons to Write for Worlds
/*
1 - 7/9/11 Pt
2 - 22/24 Pt
3 - mobilegoal, cone on mobile, pick up second cone, place mg in 5, place cone on stationary
4 - Stationary then Defense
all started from facing the

FUNCTIONS:
2 - Get Mobile Goal + 1/2 Cones
*/
void stationaryAuto()
{
	doubleSetpoint = 150;
	SensorValue[gyro] = 0;
	motor[rollerMotor] = 127;
	rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	doubleSetpoint = 600;
	chainBarSetpoint = 1500;
	wait1Msec(200);
	while(!doubleDone) { wait1Msec(20); }
	degmove(40);
	doubleSetpoint = 500;
	wait1Msec(300);
	rollerSetpoint = rollerOut;
	wait1Msec(500);
	doubleSetpoint = 600;
	wait1Msec(250);
	doubleSetpoint = 0;
	degmove(-16);
}
void mobileMain()
{
		rollerSetpoint = rollerIn;
	startTask(WATCHDOG);
	doubleSetpoint = 0;
	chainBarSetpoint = chainBarDown - 500;
	degmove(15);
		rollerSetpoint = rollerHold;
	setAllMotors(0);
	doubleSetpoint = 150;

	wait1Msec(500)

	mobileDown();
	//gyroturn(SensorValue[gyro]);
	degmove(20);
	mobileGoalSetpoint = mobileGoalUp;
	wait1Msec(350);
	rollerSetpoint = rollerOut;
	wait1Msec(350);
	chainBarSetpoint = chainBarDown - 500;
	while(!mobileDone) { wait1Msec(20); }
	gyroturn(SensorValue[gyro]);
}
void cone()
{
	doubleSetpoint = 0;
	setAllMotors(127);
	wait1Msec(275);
	setAllMotors(0);
	rollerSetpoint = rollerIn;
	chainBarSetpoint = 4400;
	wait1Msec(1000);
	chainBarSetpoint = 1000;
	wait1Msec(350);
	rollerSetpoint = rollerOut;
	wait1Msec(150);
	chainBarSetpoint = 1500;
}
void getBack(int extra=0)
{
	degmove(-45-extra);
}
void fiveZone(int wall)
{
	int turnMult = (wall == LEFT ? 1 : -1);
	gyroturn(1800 * turnMult);
	mobileDown();
	degmove(-12);
}
void twentyZone(int wall)
{
	int turnMult = (wall == LEFT ? 1 : -1);
	gyroturn((420 * turnMult), 1); // -420 = right
	degmove(-18);
	rollerSetpoint = rollerOut;
	gyroturn(935 * turnMult, 1); // -935 = right
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
}
void seven(int wall)
{
	mobileMain();
	getBack();
	fiveZone(wall);
}
void nine(int wall)
{
	mobileMain();
	cone();
	getBack(5);
	fiveZone(wall);
}
void twentytwo(int wall)
{
	mobileMain();
	getBack();
	twentyZone(wall);
}
void twentyfour(int wall)
{
	mobileMain();
	cone();
	getBack(5);
	twentyZone(wall);
}

void stationaryPlusFive(int wall)
{
	int turnMult = (wall == LEFT ? 1 : -1);
	stationaryAuto();
	gyroturn(-900 * turnMult);
	degmove(16);
	rollerSetpoint = rollerIn;
	chainBarSetpoint = 4000;
	wait1Msec(500);
	chainBarSetpoint = 2000;
	mobileGoalSetpoint = mobileGoalDown;
	gyroturn(-450 * turnMult);
	rollerSetpoint = rollerHold;
	degmove(14);
	gyroturn(450 * turnMult);
	degmove(14);
	gyroturn(300 * turnMult);
	gyroturn(-300 * turnMult);
	degmove(30);
	mobileGoalSetpoint = mobileGoalUp;
	degmove(-40);
	gyroturn(1800 * turnMult);
	mobileDown();
	degmove(-12);
}

void disruptAuto()
{
	setAllMotors(-127);
	wait1Msec(3500);
	setAllMotors(0);
}
