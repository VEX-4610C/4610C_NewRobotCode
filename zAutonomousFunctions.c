//CONFIG PARAMETERS
#define doubleDown 0
#define doubleIntake 100
#define doublePreload 250
#define doublePreloadIntake 300
#define doubleMobileGoal 150
#define doubleFixedGoal 400
#define doubleKP 0.9
#define doubleKI 0
#define doubleKD 0.05
#define doubleA 1
#define doubleB 1.1
#define doubleSensor doubleLeft
#define noLiftAfterDropNum 2
const int doubleStackUp[15] = {125, 94, 160, 245, 281, 375, 400, 460, 519, 580, 625, 719, 750, 781, 850};
const int doubleStationary[8] = {50, 50, 80, 120, 160, 200, 240, 280};
int doubleSetpoint = doubleDown;
int doubleError = 0;
int doubleDone = 0;
int doubleStackLoader = 0;
int doublePIDActive = 1;
int finishStack = 0;

#define mobileGoalDown 4200
#define mobileGoalUp 1815
#define mobileKP 0.2
#define mobileKI 0
#define mobileKD 0
int mobileGoalSetpoint = mobileGoalUp;
int mobileDone = 0;
int mobilePIDActive = 1;

#define chainBarUp 1900
#define chainBarDown 3350
#define chainBarIntake 4300
#define chainBarPreload 3000
#define chainBarReverse 3000
#define chainBarStack 1525
#define chainBarPassPos 2800
#define chainBarKP 0.03
#define chainBarKI 0
#define chainBarKD 0.02
#define chainBarB 1
#define chainBarC 1
#define chainBarSensor chainBarPot
int chainBarSetpoint = chainBarUp;
int chainBarError = 0;
int chainBarDone = 0;
int chainBarPIDActive = 1;
bool chainBarSetupDone = false;

#define rollerIn 127
#define rollerOut -127
#define rollerStop 0
#define rollerHold 12
#define rollerDone 1
int rollerSetpoint = 0;

int activateAutoStacker = 0;
int activateStationaryMobile = 0;
int currentStacked = 0;
int currentStationary = 0;
int minusOnes = 0;
int pidActive = 1;

#define HOLDOUT 400
#define setAllMotors(x) motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backRight] = (x)

task WATCHDOG
{
	int doubleStartTimer, doubleEndTime;
	int mobileStartTimer, mobileEndTime;
	int chainBarStartTimer, chainBarEndTime;
	int lastChainBar = SensorValue[chainBarSensor];
	// DR4B PID Controller
	pos_PID doublePID;
	pos_PID_InitController(&doublePID, doubleSensor, doubleKP, doubleKI, doubleKD);
	pos_PID_Activate2DOF(&doublePID, doubleA, doubleB);
	// Chain Bar PID Controller
	pos_PID chainbarPID;
	pos_PID_InitController(&chainbarPID, chainBarSensor, chainBarKP, chainBarKI, chainBarKD);
	pos_PID_Activate2DOF(&chainbarPID, chainBarB, chainBarC);
	// Mobile Goal PID Controller
	pos_PID mobilePID;
	pos_PID_InitController(&mobilePID, mobilePot, mobileKP, mobileKI, mobileKD);
	// Claw PID Controller
	int doublePower, mobilePower, chainbarPower;
	while(1)
	{
		if(pidActive)
		{

			pos_PID_SetTargetPosition(&doublePID, doubleSetpoint);
			pos_PID_SetTargetPosition(&chainbarPID, chainBarSetpoint);
			pos_PID_SetTargetPosition(&mobilePID, mobileGoalSetpoint);
			doubleError = pos_PID_GetError(&doublePID);
			chainBarError = pos_PID_GetError(&chainbarPID);
			if(chainBarSetpoint == chainBarStack)
			{
				chainbarPID.kP = 0.12;
			}
			else
			{
				chainbarPID.kP = 0.08;
			}
			if(abs(pos_PID_GetError(&mobilePID)) > 150)
			{
				pos_PID_SetTargetPosition(&doublePID, max(doubleSetpoint, doubleMobileGoal));
			}
			if(doublePIDActive)
			{
				doublePower = pos_PID_StepController(&doublePID);
				doublePower = abs(doublePower) < 15 ? 0 : doublePower;
				SetMotor(doubleLeft, doublePower);
				SetMotor(doubleRight, doublePower);
			}
			if(mobilePIDActive)
			{
				mobilePower = pos_PID_StepController(&mobilePID);
				mobilePower = abs(mobilePower) < 15 ? 0 : mobilePower;
				if(mobileDone && abs(mobilePID.error) < 150)
					mobilePower = 0;
				SetMotor(mobileGoal,  mobilePower);
			}

			if(chainBarPIDActive)
			{
				chainbarPower = pos_PID_StepController(&chainbarPID);
				motor[chainbar] = chainbarPower;
			}
			motor[rollerMotor] = rollerSetpoint;
			// Dones
			if(abs(pos_PID_GetError(&doublePID)) < 50 || abs(doublePower) < 25)
			{
				if(doubleStartTimer == 0)
					doubleEndTime = time1[T1] + HOLDOUT;
				doubleStartTimer = 1;
			}
			else
			{
				doubleStartTimer = 0;
			}
			if(doubleStartTimer && time1[T1] > doubleEndTime)
			{
				doubleDone = 1;
			}
			else
			{
				doubleDone = 0;
			}
			if(abs(pos_PID_GetError(&mobilePID)) < 150 || abs(mobilePower) < 50 || fabs(mobilePID.derivative) < 100)
			{
				if(mobileStartTimer == 0)
					mobileEndTime = time1[T1] + HOLDOUT;
				mobileStartTimer = 1;
			}
			else
			{
				mobileStartTimer = 0;
			}
			if(mobileStartTimer && time1[T1] > mobileEndTime)
			{
				mobileDone = 1;
			}
			else
			{
				mobileDone = 0;
			}

			if(abs(pos_PID_GetError(&chainbarPID)) < 150 || (abs(pos_PID_GetError(&chainbarPID)) < 350 && (chainBarSetpoint < 1000 || chainBarSetpoint > 4000)))
			{
				if(chainBarStartTimer == 0)
					chainBarEndTime = time1[T1] + HOLDOUT;
				chainBarStartTimer = 1;
			}
			else
			{
				chainBarStartTimer = 0;
			}
			if(chainBarStartTimer && time1[T1] > chainBarEndTime)
			{
				chainBarDone = 1;
			}
			else
			{
				chainBarDone = 0;
			}
			wait1Msec(25);
			lastChainBar = pos_PID_GetError(&chainbarPID);
		}
	}
}
int innerState = 0;
int lastAutostacker = 0;
task autoStacker
{
	/* ORDER OF ACTIONS:
	1. Close Claw
	2. Lift DR4B
	3. Lift Chainbar
	4. Open Claw
	5. Lower Chainbar
	6. Lower DR4B
	*/
	int limitNum = sizeof(doubleStackUp) / sizeof(int);
	while(1)
	{
		if((activateAutoStacker || activateStationaryMobile) && !lastAutostacker)
		{
			innerState = 0;
		}
		if(!activateAutoStacker && !activateStationaryMobile)
		{
			lastAutostacker = 0;
		}
		else if(activateAutoStacker && currentStacked < limitNum)
		{
			if(innerState == 0)
			{
				if(doubleStackLoader)
				{
					doubleSetpoint = doublePreload + 50;
					chainBarSetpoint = chainBarPreload;
					rollerSetpoint = rollerIn;
					chainBarSetpoint += 1000;
					doubleSetpoint -= 200;
					//while(SensorValue[chainBarPot] > 1200) { wait1Msec(20); }
					wait1Msec(600);

				}
				rollerSetpoint = rollerHold;
				if(!doubleStackLoader)
					chainBarSetpoint = chainBarPassPos;
				else
					chainBarSetpoint = chainBarPreload;
				innerState++;
			}
			else if(innerState == 1)
			{
				if(1)
				{
					doubleDone = 0;
					wait1Msec(100);
					doubleSetpoint = doubleStackUp[currentStacked];
					innerState++;
				}
			}
			else if(innerState == 2)
			{
				if(nMotorEncoder[doubleLeft] > (doubleStackUp[currentStacked] - 100))
				{
					chainBarSetpoint = chainBarStack;
					innerState++;
				}
			}
			else if(innerState == 3)
			{
				if((SensorValue[chainBarPot] < 1950 || chainBarDone) && abs(doubleError) < 150)
				{
					if((currentStacked == (11-minusOnes) && doubleStackLoader) || finishStack)
					{
						activateAutoStacker = 0;
						finishStack = 0;
					}
					else
					{
						wait1Msec(50);
						doubleSetpoint -= 100;
						rollerSetpoint = rollerOut;
						if(currentStacked == 0)
							wait1Msec(75);
						wait1Msec(300);
						rollerSetpoint = rollerStop;
						doubleSetpoint += 140;
						wait1Msec(100);
						innerState++;
					}
				}
			}
			else if(innerState == 4)
			{
				if(1)
				{
					if(doubleStackLoader)
					{
						if(currentStacked < 4)
						{
							doubleSetpoint = doublePreload;
						}
						else
						{
							chainBarSetpoint = chainBarPreload;
						}
					}
					else
					{
						chainBarSetpoint = chainBarDown;
					}
					innerState++;
				}
			}
			else if(innerState == 5)
			{
				if(SensorValue[chainBarPot] > 2300 || (doubleStackLoader && currentStacked < 5 && abs(doubleError) < 400)) // encoder > 350
				{
					if(doubleStackLoader)
					{
						doubleSetpoint = doublePreload;
						chainBarSetpoint = chainBarPreload;
						if(vexRT[Btn6U])
						{
							rollerSetpoint = rollerIn;
						}
					}
					else
					{
						doubleSetpoint = doubleDown;
					}
					activateAutoStacker = 0;
					activateStationaryMobile = 0;
					currentStacked++;
					innerState = 0;
				}
			}
			lastAutostacker = 1;
		}
		// Mobile Above
		// Stationary BELOW
		else if(activateStationaryMobile && currentStacked > 3 && currentStationary < 7)
		{
			if(innerState == 0)
			{
				if(doubleSetpoint > doubleStackUp[currentStacked]) // currently higher
				{
					chainBarSetpoint = chainBarUp;
					doubleSetpoint += 200;
					wait1Msec(200);
					doubleSetpoint = doubleStackUp[currentStacked];
				}
				else // currently lower
				{
					doubleSetpoint = doubleStackUp[currentStacked];
					while(abs(doubleError) > 100)
						wait1Msec(20);
					chainBarSetpoint = chainBarUp;
				}
				innerState++;
			}
			else if(innerState == 1)
			{
				if(abs(doubleError) < 75)
				{
					chainBarSetpoint = chainBarStack;
					innerState++;
				}
			}
			else if(innerState == 2)
			{
				if(SensorValue[chainBarPot] < 1950 || chainBarDone)
				{
					doubleSetpoint -= 300;
					wait1Msec(350);
					rollerSetpoint = rollerIn;
					wait1Msec(400);
					rollerSetpoint = rollerHold;
					innerState ++;
				}
			}
			else if(innerState == 3)
			{
				doubleSetpoint += 250;
				innerState++;
			}
			else if(innerState == 4)
			{
				if(doubleDone || abs(doubleError) < 75)
				{
					if((doubleFixedGoal + doubleStationary[currentStationary]) > doubleSetpoint) // going higher
					{
						doubleSetpoint = doubleFixedGoal + doubleStationary[currentStationary];
						wait1Msec(150 + 30 * currentStationary);
						chainBarSetpoint = currentStationary >= 5 ? chainBarReverse : chainBarReverse - 150;
						wait1Msec(250);
					}
					else
					{
						chainBarSetpoint = currentStationary >= 5 ? chainBarReverse : chainBarReverse - 150;
						doubleSetpoint = doubleFixedGoal + doubleStationary[currentStationary];
						wait1Msec(150);
					}
					if(currentStationary != 0 && currentStationary < 6)
					{
						wait1Msec(300);
						rollerSetpoint = rollerOut;
						wait1Msec(300);
					}
					activateAutoStacker = 0;
					activateStationaryMobile = 0;
					currentStacked--;
					currentStationary++;
					innerState = 0;
				}
			}
			lastAutostacker = 1;
		}
		wait1Msec(50);
	}
}

// LCD Display Code
// Some utility strings
#define LEFT_ARROW  247
#define RIGHT_ARROW 246
static  char l_arr_str[4] = { LEFT_ARROW,  LEFT_ARROW,  LEFT_ARROW,  0};
static  char r_arr_str[4] = { RIGHT_ARROW, RIGHT_ARROW, RIGHT_ARROW, 0};
// Little macro to keep code cleaner, masks both disable/ebable and auton/driver
#define vexCompetitionState (nVexRCReceiveState & (vrDisabled | vrAutonomousMode))

TControllerButtons getLcdButtons()
{
	TVexReceiverState   competitionState = vexCompetitionState;
	TControllerButtons  buttons;

	// This function will block until either
	// 1. A button is pressd on the LCD
	//    If a button is pressed when the function starts then that button
	//    must be released before a new button is detected.
	// 2. Robot competition state changes

	// Wait for all buttons to be released
	while( nLCDButtons != kButtonNone ) {
		// check competition state, bail if it changes
		if( vexCompetitionState != competitionState )
			return( kButtonNone );
		wait1Msec(10);
	}

	// block until an LCD button is pressed
	do  {
		// we use a copy of the lcd buttons to avoid their state changing
		// between the test and returning the status
		buttons = nLCDButtons;

		// check competition state, bail if it changes
		if( vexCompetitionState != competitionState )
			return( kButtonNone );

		wait1Msec(10);
	} while( buttons == kButtonNone );

	return( buttons );
}

static int MyAutonomous = -1;
/*-----------------------------------------------------------------------------*/
/*  Display autonomous selection                                               */
/*-----------------------------------------------------------------------------*/

// max number of auton choices
#define MAX_CHOICE  11

void LcdAutonomousSet( int value, bool select = false )
{
	// Cleat the lcd
	clearLCDLine(0);
	clearLCDLine(1);

	// Display the selection arrows
	displayLCDString(1,  0, l_arr_str);
	displayLCDString(1, 13, r_arr_str);

	// Save autonomous mode for later if selected
	if(select)
		MyAutonomous = value;

	// If this choice is selected then display ACTIVE
	if( MyAutonomous == value )
		displayLCDString(1, 5, "ACTIVE");
	else
		displayLCDString(1, 5, "select");

	// Show the autonomous names
	switch(value) {
	case    0:
		displayLCDString(0, 0, "SEVEN LEFT");
		break;
	case    1:
		displayLCDString(0, 0, "SEVEN RIGHT");
		break;
	case    2:
		displayLCDString(0, 0, "NINE LEFT");
		break;
	case    3:
		displayLCDString(0, 0, "NINE RIGHT");
		break;
	case    4:
		displayLCDString(0, 0, "22 LEFT");
		break;
	case    5:
		displayLCDString(0, 0, "22 RIGHT");
		break;
	case    6:
		displayLCDString(0, 0, "24 LEFT");
		break;
	case    7:
		displayLCDString(0, 0, "24 RIGHT");
		break;
	case    8:
		displayLCDString(0, 0, "STATIONARY");
		break;
	case    9:
		displayLCDString(0, 0, "STAT + LEFT");
		break;
	case    10:
		displayLCDString(0, 0, "STAT + RIGHT");
		break;
	case    11:
		displayLCDString(0, 0, "BLOCK");
		break;
	default:
		displayLCDString(0, 0, "SCREAM AT ALEX");
		break;
	}
}

void LcdAutonomousSelection()
{
	TControllerButtons  button;
	int  choice = 0;

	// Turn on backlight
	bLCDBacklight = true;

	// diaplay default choice
	LcdAutonomousSet(0);

	while( bIfiRobotDisabled )
	{
		// this function blocks until button is pressed
		button = getLcdButtons();

		// Display and select the autonomous routine
		if( ( button == kButtonLeft ) || ( button == kButtonRight ) ) {
			// previous choice
			if( button == kButtonLeft )
				if( --choice < 0 ) choice = MAX_CHOICE;
			// next choice
			if( button == kButtonRight )
				if( ++choice > MAX_CHOICE ) choice = 0;
			LcdAutonomousSet(choice);
		}

		// Select this choice
		if( button == kButtonCenter )
			LcdAutonomousSet(choice, true );

		// Don't hog the cpu !
		wait1Msec(10);
	}
}



void gyroturnPID(float degrees, int mG)
{
	degrees = degrees;
	SensorValue[gyro] = 0;
	float kP = mG ? 0.135 : 0.09;
	float kI = 0.02;
	float kD = 0.01;
	int turnDone = 0;
	int turnStartTimer, turnEndTime;
	int error, power;
	int totalError, lastError = degrees, lastTime = time1[T3]-1;
	int startTime = time1[T3]-1;
	float dedt;
	while(!turnDone)
	{
		error = degrees + SensorValue[gyro];
		dedt = (error - lastError) / (time1[T3] - lastTime);
		totalError += dedt;
		power = (error * kP) + (totalError * kI / (time1[T3] - startTime)) + (dedt * kD);
		motor[frontLeft] = motor[backLeft] = power + 35*sign(power);
		motor[frontRight] = motor[backRight] = -power - 35*sign(power);
		if(abs(error) < 50 && !turnStartTimer)
		{
			turnStartTimer = 1;
			turnEndTime = time1[T3] + 250;
		}
		else if(abs(power) < 30 && !turnStartTimer)
		{
			turnStartTimer = 1;
			turnEndTime = time1[T3] + 250;
	}
		else if(abs(time1[T3] - startTime) < 150  && degrees < 100 && !turnStartTimer)
		{
			turnStartTimer = 1;
			turnEndTime = time1[T3] + 250;
		}
		else if (abs(error) > 50 && abs(power) > 30)
		{
			turnStartTimer = 0;
		}
		if(turnStartTimer && turnEndTime < time1[T3])
		{
			turnDone = 1;
		}
		lastError = error;
		lastTime = time1[T3];
		wait1Msec(75);
	}
}
void straighten()
{
	int degrees = SensorValue[gyro];
	SensorValue[gyro] = 0;
	int absdegs = abs(degrees);
	while(abs(absdegs + SensorValue[gyro]) > 40)
	{
		motor[frontLeft]  = motor[backLeft]  =  40 * sign(degrees);
		motor[frontRight] = motor[backRight] = -40 * sign(degrees);
	}
	motor[frontLeft]  = motor[backLeft]  = -20 * sign(degrees);
	motor[frontRight] = motor[backRight] =  20 * sign(degrees);
	wait1Msec(150);
	motor[frontLeft]  = motor[backLeft]  = 0;
	motor[frontRight] = motor[backRight] = 0;
}
void gyroturnBang(int degrees)
{
	degrees = degrees;
	SensorValue[gyro] = 0;
	int absdegs = degrees;
	while(abs(absdegs + SensorValue[gyro]) > 75)
	{
		motor[frontLeft]  = motor[backLeft]  =  120 * sign(degrees);
		motor[frontRight] = motor[backRight] = -120 * sign(degrees);
	}
	motor[frontLeft]  = motor[backLeft]  = -20 * sign(degrees);
	motor[frontRight] = motor[backRight] =  20 * sign(degrees);
	wait1Msec(150);
	motor[frontLeft]  = motor[backLeft]  = 0;
	motor[frontRight] = motor[backRight] = 0;
}
void degmove(int degrees)
{
	SensorValue[gyro] = 0;
	nMotorEncoder[frontLeft] = 0;
	nMotorEncoder[frontRight] = 0;
	degrees *= 25;
	float kP = 0.24;
	//float kI = 0;
	//float kD = 0.02;
	//float gyroKP = 0;
	//float encoderkP = 0;
	//int moveDone = 0;
	//int moveStartTimer, moveEndTime;
	int error = degrees + nMotorEncoder[frontLeft], power;
	//int totalError, lastError = degrees + nMotorEncoder[frontLeft], lastTime = time1[T3]-1;
	//int startTime = time1[T3]-1;
	//float dedt;
	//int gyroAdj;
	//int encoderAdj;

	while(abs(error) > 75)
	{
			writeDebugStreamLine("ee %d", error);
		error = degrees + nMotorEncoder[frontLeft];
		power = (error * kP);
		motor[backRight] = motor[backLeft] = 120 * sign(power);
		motor[frontRight] = motor[frontLeft] = 120 * sign(power);
	}
	setAllMotors(20 * sign(degrees));
	wait1Msec(300);
	setAllMotors(0);
}
void gyroturn(int degrees)
{
	degrees *= .85;
	gyroturnBang(degrees);
	//if(1)
	//	gyroturnPID(degrees, 1);
}
void gyroturn(int degrees, int mg)
{
	gyroturn(degrees);
}
task setUpChainBar()
{
	chainBarPIDActive = 0;
	motor[chainbar] = -127;
	wait1Msec(1000);
	motor[chainbar] = 127;
	wait1Msec(750);
	motor[chainbar] = 0;
	nMotorEncoder[chainbar] = 0;
	wait1Msec(750);
	chainBarPIDActive = 1;
	chainBarSetupDone = true;
}
/*
Effects of increasing a parameter independently[21][22]
Rise time					Overshoot	Settling time	Steady-state error	Stability
kP	Decrease			Increase	Small change	Decrease						Degrade
kI	Decrease			Increase	Increase			Eliminate						Degrade
kD	Minor change	Decrease	Decrease			No effect in theory	Improve if kD small

If the system must remain online, one tuning method is to first set kI and kD values to zero. Increase the kP until the output of the
loop oscillates, then the kP should be set to approximately half of that value for a "quarter amplitude decay" type response. Then increase
kI until any offset is corrected in sufficient time for the process. However, too much kI will cause instability. Finally, increase kD,
if required, until the loop is acceptably quick to reach its reference after a load disturbance. However, too much kD will cause excessive
response and overshoot. A fast PID loop tuning usually overshoots slightly to reach the setpoint more quickly; however, some systems cannot a
ccept overshoot, in which case an overdamped closed-loop system is required, which will require a kP setting significantly less than half that
of the kP setting that was causing oscillation

*/
