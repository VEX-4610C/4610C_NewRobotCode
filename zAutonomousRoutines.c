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
	gyroturn(450, 0);
	wait1Msec(250);
	gyroturn(-900, 0);
	wait1Msec(250);
	gyroturn(900, 0);
	wait1Msec(250);
	gyroturn(-450, 0);
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
void mobileGoalTenAuto()
{
	startTask(WATCHDOG);
	startTask(setUpChainBar, 6);
	mobileDown();
	degmove(55);
	mobileUp();
	degmove(-40);
	clawSetpoint = clawOpen;

	gyroturn(1700,1);
	degmove(20);
	mobileDown();
	degmove(-30);
}

void mobileGoalTwenAuto(int wall)
{
	startTask(WATCHDOG);
	startTask(setUpChainBar, 6);
	int turnMult = wall == LEFT ? 1 : -1;
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(500);
	degmove(55);
	mobileUp();
	degmove(-45);
	clawSetpoint = clawOpen;
	gyroturn(-450 * turnMult, 1);
	degmove(-20);
	gyroturn(-750 * turnMult, 1);
	degmove(20);
	mobileDown();
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 127;
	wait1Msec(500);
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 0;
	mobileGoalSetpoint = mobileGoalUp;
	degmove(-30);
}


void programmingSkills()
{
	doubleSetpoint = 250;
	startTask(WATCHDOG);
	mobileDown();
	degmove(40);
	mobileUp();
	clawSetpoint = clawOpen;
	degmove(-22);
	gyroturn(-900, 1);
	degmove(15);
	gyroturn(-800, 1);
	degmove(25);
	mobileDown();
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 127;
	wait1Msec(1000);
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 0;
	degmove(-30);
	mobileUp();
	degmove(-30)
	gyroturn(625, 0);
	degmove(-60);
}
