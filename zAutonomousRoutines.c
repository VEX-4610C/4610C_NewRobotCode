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
	gyroturn(450);
	wait1Msec(250);
	gyroturn(-900);
	wait1Msec(250);
	gyroturn(900);
	wait1Msec(250);
	gyroturn(-450);
}
void testLargeGyroturn()
{
	gyroturn(900);
	wait1Msec(250);
	gyroturn(-1800);
	wait1Msec(250);
	gyroturn(1800);
	wait1Msec(250);
	gyroturn(-900);
}
/* Routines to Write
-	7 Point Match Auto
*/
void mobileGoalAuto(int wall)
{
	startTask(WATCHDOG);
	startTask(autoStacker);

	mobileGoalSetpoint = mobileGoalDown;
	mobileDone = 0;
	wait1Msec(250);
	while(!mobileDone) { wait1Msec(20); }

	degmove(72);

	mobileGoalSetpoint = mobileGoalUp;
	mobileDone = 0;
	wait1Msec(250);
	while(!mobileDone) { wait1Msec(20); }

	activateAutoStacker = 1;
	degmove(72);
	while(activateAutoStacker) { wait1Msec(20); };

	if(wall == LEFT)
	{
		gyroturn(105);
	}
	if(wall == RIGHT)
	{
		gyroturn(-105);
	}

	mobileGoalSetpoint = mobileGoalDown;
	mobileDone = 0;
	wait1Msec(250);
	while(!mobileDone) { wait1Msec(20); }

	degmove(-14);
}
