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

