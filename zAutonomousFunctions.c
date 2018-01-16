//CONFIG PARAMETERS
#define doubleDown 0
#define doublePreload 0
#define doubleMobileGoal 250
#define doubleFixedGoal 800
#define doubleKP 0.8
#define doubleKI 0
#define doubleKD 0.02
#define doubleSensor doubleLeft
#define noLiftAfterDropNum 2
const int doubleStackUp[10] = {0, 0, 150, 400, 400, 450, 700, 750, 800, 850};
int doubleSetpoint = doubleDown;
int doubleError = 0;
int doubleDone = 0;
int doubleStackLoader = 0;
int doublePIDActive = 1;

#define mobileGoalDown 3400
#define mobileGoalUp 1200
#define mobileKP 0.35
#define mobileKI 0
#define mobileKD 0
#define mobileDIVISOR 100
int mobileGoalSetpoint = mobileGoalUp;
int mobileDone = 0;
int mobilePIDActive = 1;

#define chainBarUp 0
#define chainBarDown 850
#define chainBarPreload 400
#define chainBarStack 0
#define chainBarPassPos 700
#define chainBarKP 0.6
#define chainBarKI 0
#define chainBarKD 0
#define chainBarDIVISOR 1
#define chainBarSensor chainbar
int chainBarSetpoint = chainBarUp;
int chainBarError = 0;
int chainBarDone = 0;
int chainBarPIDActive = 1;
int setUpChainbarDone = 0;

#define clawOpen 30
#define clawClosed 85
#define clawDone 1
int clawSetpoint = clawOpen;

int activateAutoStacker = 0;
int currentStacked = 0;
int pidActive = 1;

#define HOLDOUT 750

task WATCHDOG
{
	int doubleStartTimer, doubleEndTime;
	int mobileStartTimer, mobileEndTime;
	int chainBarStartTimer, chainBarEndTime;
	// DR4B PID Controller
	pos_PID doublePID;
	pos_PID_InitController(&doublePID, doubleSensor, doubleKP, doubleKI, doubleKD);
	// Chain Bar PID Controller
	pos_PID chainbarPID;
	pos_PID_InitController(&chainbarPID, chainBarSensor, chainBarKP, chainBarKI, chainBarKD);
	// Mobile Goal PID Controller
	pos_PID mobilePID;
	pos_PID_InitController(&mobilePID, mobilePot, mobileKP, mobileKI, mobileKD);
	// Claw PID Controller
	int x;
	while(1)
	{
		writeDebugStreamLine("%d", pos_PID_GetError(&chainbarPID));
		if(pidActive)
		{
			pos_PID_SetTargetPosition(&doublePID, doubleSetpoint);
			pos_PID_SetTargetPosition(&chainbarPID, chainBarSetpoint);
			pos_PID_SetTargetPosition(&mobilePID, mobileGoalSetpoint);
			doubleError = pos_PID_GetError(&doublePID);
			chainBarError = pos_PID_GetError(&chainbarPID);
			if(abs(pos_PID_GetError(&mobilePID)) > 150)
			{
				pos_PID_SetTargetPosition(&doublePID, max(doubleSetpoint, doubleMobileGoal));
			}
			if(doublePIDActive)
			{
				x = pos_PID_StepController(&doublePID);
				x = abs(x) < 15 ? 0 : x;
				SetMotor(doubleLeft, x);
				SetMotor(doubleRight, x);
			}
			if(mobilePIDActive)
			{
				x = pos_PID_StepController(&mobilePID);
				x = abs(x) < 15 ? 0 : x;
				SetMotor(mobileGoal,  x);
			}
			if(chainBarPIDActive)
			{
				SetMotor(chainbar, pos_PID_StepController(&chainbarPID));
			}
			SetMotor(claw, clawSetpoint);
			// Dones
			if(abs(pos_PID_GetError(&doublePID)) < 50)
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
			if(abs(pos_PID_GetError(&mobilePID)) < 150)
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
			if(abs(pos_PID_GetError(&chainbarPID)) < 50)
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
	while(1)
	{
		if(activateAutoStacker && !lastAutostacker)
		{
			innerState = 0;
		}
		if(!activateAutoStacker)
		{
			lastAutostacker = 0;
		}
		else if(activateAutoStacker && currentStacked < 10)
		{
			if(innerState == 0)
			{
				clawSetpoint = clawClosed;
				chainBarSetpoint = chainBarPassPos;
				innerState++;
			}
			else if(innerState == 1)
			{
				if(clawDone)
				{
					doubleDone = 0;
					doubleSetpoint = doubleStackUp[currentStacked];
					innerState++;
				}
			}
			else if(innerState == 2)
			{
				if(doubleDone)
				{
					chainBarSetpoint = chainBarUp;
					innerState++;
				}
			}
			else if(innerState == 3)
			{
				if(doubleDone)
				{
					clawSetpoint = clawOpen;
					innerState++;
				}
			}
			else if(innerState == 4)
			{
				if(doubleDone)
				{
					if(doubleStackLoader)
					{
						chainBarSetpoint = chainBarPreload;
					}
					else
					{
						chainBarSetpoint = chainBarPassPos;
					}
				}
			}
			else if(innerState == 5)
			{
				if(chainBarDone)
				{
						doubleSetpoint = doubleDown;
				}
				innerState++;
			}
			else if(innerState == 6)
			{
				if(doubleStackLoader)
				{
					chainBarSetpoint = chainBarPreload;
				}
				else
				{
					chainBarSetpoint = chainBarDown;
				}
				activateAutoStacker = 0;
				currentStacked++;
				innerState = 0;
			}

			lastAutostacker = 1;
		}
		wait1Msec(100);
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
#define MAX_CHOICE  2

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
		displayLCDString(0, 0, "MG1 Wall Left");
		break;
	case    1:
		displayLCDString(0, 0, "MG1 Wall Right");
		break;
	case    2:
		displayLCDString(0, 0, "Programming Skills");
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



int batteryOneLevel, batteryTwoLevel;
task batLevel
{
	while(1)
	{
		batteryOneLevel = nImmediateBatteryLevel;
		batteryTwoLevel = (int)((float)SensorValue[ peStatus ] * 5.48); // if wrong try 3.636
		displayLCDString(0, 0, "Cortex BL ");
		displayLCDNumber(0, 10, batteryOneLevel, 4);
		displayLCDString(0, 0, "PrwrEx BL ");
		displayLCDNumber(0, 10, batteryTwoLevel, 4);
		wait1Msec(25);
	}
}
void degmove(int distance)
{
	nMotorEncoder[frontLeft] = 0;
	distance *= 15;
	if(distance > 0) // move forward
	{
		while(abs(nMotorEncoder[frontLeft]) < distance)
		{
			motor[frontLeft] = motor[backLeft] = abs(nMotorEncoder[frontLeft] - distance) * .2 + 35;
			motor[frontRight] = motor[backRight] = abs(nMotorEncoder[frontLeft] - distance) * .2 + 35;
		}
		motor[frontLeft] = motor[backLeft] = -15;
		motor[frontRight] = motor[backRight] = -15;
		wait1Msec(100);
		motor[frontLeft] = motor[backLeft] = 0;
		motor[frontRight] = motor[backRight] = 0;
	}
	else
	{
		while(abs(nMotorEncoder[frontLeft]) < abs(distance))
		{
			motor[frontLeft] = motor[backLeft] = -1*(abs(nMotorEncoder[frontLeft] - distance) * .2 + 35);
			motor[frontRight] = motor[backRight] = -1*(abs(nMotorEncoder[frontLeft] - distance) * .2 + 35);
		}
		motor[frontLeft] = motor[backLeft] = 15;
		motor[frontRight] = motor[backRight] = 15;
		wait1Msec(100);
		motor[frontLeft] = motor[backLeft] = 0;
		motor[frontRight] = motor[backRight] = 0;
	}
}
void gyroturn(float degrees, int mG)
{
	SensorValue[in3] = 0;
	float kP = mG ? 0.125 : 0.1;
	float kI = 0;
	float kD = 0;
	int turnDone = 0;
	int turnStartTimer, turnEndTime;
	int error, power;
	int totalError, lastError = degrees - SensorValue[in3], lastTime = time1[T3];
	int startTime = time1[T3];
	float dedt;
	while(!turnDone)
	{
		error = degrees - SensorValue[in3];
		dedt = (error - lastError) / (time1[T3] - lastTime);
		totalError += dedt;
		power = (error * kP) + (totalError * kI / (time1[T3] - startTime)) + (dedt * kD);
		motor[frontLeft] = motor[backLeft] = power;
		motor[frontRight] = motor[backRight] = -power;
		if(abs(error) < 50 && !turnStartTimer)
		{
			turnStartTimer = 1;
			turnEndTime = time1[T3] + 750;
		}
		else if (abs(error) > 50)
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
void PIDDegmove(int degrees)
{
	nMotorEncoder[frontLeft] = 0;
	degrees *= 25;
	float kP = 0.25;
	float kI = 0;
	float kD = 0;
	int turnDone = 0;
	int turnStartTimer, turnEndTime;
	int error, power;
	int totalError, lastError = degrees - nMotorEncoder[frontLeft], lastTime = time1[T3];
	float dedt;
	while(!turnDone)
	{
		error = degrees - nMotorEncoder[frontLeft];
		dedt = (error - lastError) / (time1[T3] - lastTime);
		totalError += dedt;
		power = (error * kP) + (totalError * kI) + (dedt * kD);
		motor[frontLeft] = motor[backLeft] = power;
		motor[frontRight] = motor[backRight] = power;
		if(abs(error) < 50 && !turnStartTimer)
		{
			turnStartTimer = 1;
			turnEndTime = time1[T3] + 750;
		}
		else if(abs(error) > 50)
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
void setUpChainBar()
{
	motor[chainbar] = 127;
	wait1Msec(1500);
	motor[chainbar] = -80;
	wait1Msec(800);
	nMotorEncoder[chainbar] = 0;
	setUpChainbarDone = 1;
}
/*
Effects of increasing a parameter independently[21][22]
    Rise time			Overshoot	Settling time	Steady-state error	Stability
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
