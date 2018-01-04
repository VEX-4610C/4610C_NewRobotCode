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
void mobileGoalAuto(int alliance)
{
}
