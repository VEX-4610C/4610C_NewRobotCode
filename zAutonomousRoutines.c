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
		gyroturn(105, 0);
	}
	if(wall == RIGHT)
	{
		gyroturn(-105, 0);
	}

	mobileGoalSetpoint = mobileGoalDown;
	mobileDone = 0;
	wait1Msec(250);
	while(!mobileDone) { wait1Msec(20); }

	degmove(-14);
}
void programmingSkills()
{
	doubleSetpoint = 250;
	startTask(WATCHDOG);
	mobileDone = 0;
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }

	degmove(50);

	mobileDone = 0;
	mobileGoalSetpoint = mobileGoalUp;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }
	degmove(-20);
	gyroturn(1800, 1);
	chainBarPIDActive = 0;

	degmove(12);

	mobileDone = 0;
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }


	/*
	degmove(-20);
	mobileDone = 0;
	mobileGoalSetpoint = mobileGoalUp;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }
	degmove(10);
	gyroturn(900, 1);
	degmove(40);
	wait1Msec(300);
	gyroturn(900, 0);

	mobileDone = 0;
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }

	degmove(45);
		mobileDone = 0;
	mobileGoalSetpoint = mobileGoalUp;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }

	degmove(-45);
	gyroturn(1800, 1);
	degmove(30);
		mobileDone = 0;
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 127;
	wait1Msec(1000);
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 0;
	degmove(-60);
	*/

	degmove(-20);
	mobileDone = 0;
	mobileGoalSetpoint = mobileGoalUp;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }
	degmove(-28);
	gyroturn(850, 0);

		mobileDone = 0;
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }
	degmove(40);
		mobileDone = 0;
	mobileGoalSetpoint = mobileGoalUp;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }
	degmove(20);
	gyroturn(-1000, 1);
	degmove(75);

			mobileDone = 0;
	mobileGoalSetpoint = mobileGoalDown;
	wait1Msec(500);
	while(!mobileDone) { wait1Msec(20); }
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 127;
	wait1Msec(1000);
	motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = 0;
	degmove(-40);





}
