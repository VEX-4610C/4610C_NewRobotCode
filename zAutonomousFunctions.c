//CONFIG PARAMETERS
#define doubleDown 0
#define doublePreload 250
#define doubleMobileGoal 250
#define doubleFixedGoal 950
#define doubleKP 1.2
#define doubleKI 0
#define doubleKD 0.1
#define doubleSensor doubleLeft
#define noLiftAfterDropNum 2
const int doubleStackUp[15] = {0, 0, 200, 350, 450, 500, 650, 700, 800, 850,
	950, 1000, 1150, 1200, 1250};
int doubleSetpoint = doubleDown;
int doubleError = 0;
int doubleDone = 0;
int doubleStackLoader = 0;
int doublePIDActive = 1;
int finishStack = 0;

#define mobileGoalDown 3400
#define mobileGoalUp 1200
#define mobileKP 0.35
#define mobileKI 0
#define mobileKD 0
int mobileGoalSetpoint = mobileGoalUp;
int mobileDone = 0;
int mobilePIDActive = 1;

#define chainBarUp 150
#define chainBarDown 3400
#define chainBarPreload 2400
#define chainBarPreloadIntake 2400
#define chainBarStack 1300
#define chainBarPassPos 2600
#define chainBarKP 0.085
#define chainBarKI 0
#define chainBarKD 0.01
#define chainBarSensor chainBarPot
int chainBarSetpoint = chainBarUp;
int chainBarError = 0;
int chainBarDone = 0;
int chainBarPIDActive = 1;
int chainBarAdjustment = 0;
bool chainBarSetupDone = false;

#define rollerIn 127
#define rollerOut -127
#define rollerStop 0
#define rollerDone 1
int rollerSetpoint = 0;

int activateAutoStacker = 0;
int currentStacked = 0;
int pidActive = 1;

#define HOLDOUT 400

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
			if(abs(pos_PID_GetError(&mobilePID)) < 150 || abs(mobilePower) < 25)
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
			if(abs(pos_PID_GetError(&chainbarPID)) < 150)
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
	int limitNum = sizeof(doubleStackUp) / sizeof(int);
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
		else if(activateAutoStacker && currentStacked < limitNum)
		{
			if(innerState == 0)
			{
				if(!doubleStackLoader)
					chainBarSetpoint = chainBarDown;
				else
					chainBarSetpoint = chainBarPreloadIntake;
				rollerSetpoint = rollerIn;
				wait1Msec(300);
				rollerSetpoint = rollerStop;
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
					doubleSetpoint = doubleStackUp[currentStacked];
					innerState++;
				}
			}
			else if(innerState == 2)
			{
				if(abs(doubleError) < 350)
				{
					chainBarSetpoint = chainBarStack;
					innerState++;
				}
			}
			else if(innerState == 3)
			{
				if(chainBarDone)
				{
					chainBarPIDActive = 0;
					motor[chainbar] = -127;
					wait1Msec(100);
					rollerSetpoint = rollerOut;
					wait1Msec(300);
					rollerSetpoint = rollerStop;
					chainBarPIDActive = 1;
					innerState++;
				}
			}
			else if(innerState == 4)
			{
				if(1)
				{
					if(doubleStackLoader)
					{
						chainBarSetpoint = chainBarPreload;
					}
					else
					{
						chainBarSetpoint = chainBarPassPos;
					}
					innerState++;
				}
			}
			else if(innerState == 5)
			{
				if(SensorValue[chainBarPot] > 2200) // encoder > 350
				{
					if(doubleStackLoader)
						doubleSetpoint = doublePreload;
					else
						doubleSetpoint = doubleDown;
					innerState++;
				}

			}
			else if(innerState == 6)
			{
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
#define MAX_CHOICE  4

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
		displayLCDString(0, 0, "MG 10 Zone");
		break;
	case    1:
		displayLCDString(0, 0, "MG 20 WallLeft");
		break;
	case    2:
		displayLCDString(0, 0, "MG 20 WallRight");
		break;
	case    3:
		displayLCDString(0, 0, "Programming Skills");
		break;
	case    4:
		displayLCDString(0, 0, "No Auto Run");
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
void gyroturn(float degrees, int mG)
{
	degrees = degrees;
	SensorValue[gyro] = 0;
	float kP = mG ? 0.18 : 0.11;
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
		motor[frontLeft] = motor[backLeft] = power;
		motor[frontRight] = motor[backRight] = -power;
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
		else if (abs(error) > 50 && abs(power) > 30)
		{
			turnStartTimer = 0;
		}
		if(turnStartTimer && turnEndTime < time1[T3])
		{
			turnDone = 1;
		}
		if((time1[T3] - startTime) > 5000)
		{
			turnDone = 1;
		}
		lastError = error;
		lastTime = time1[T3];
		wait1Msec(75);
	}
}
void degmove(int degrees)
{
	SensorValue[gyro] = 0;
	nMotorEncoder[frontLeft] = 0;
	degrees *= 25;
	float kP = 0.28;
	float kI = 0;
	float kD = 0;
	float gyroKP = 0.05;
	int moveDone = 0;
	int moveStartTimer, moveEndTime;
	int error, power;
	int totalError, lastError = degrees - nMotorEncoder[frontLeft], lastTime = time1[T3]-1;
	int startTime = time1[T3]-1;
	float dedt;
	int gyroAdj;
	while(!moveDone)
	{
		error = degrees - nMotorEncoder[frontLeft];
		dedt = (error - lastError) / (time1[T3] - lastTime);
		totalError += dedt;
		power = (error * kP) + (totalError * kI) + (dedt * kD);
		gyroAdj = SensorValue[gyro] * gyroKP;
		motor[frontLeft] = motor[backLeft] = power + gyroAdj;
		motor[frontRight] = motor[backRight] = power - gyroAdj;
		if(abs(error) < 50 && moveStartTimer == 0)
		{
			moveStartTimer = 1;
			moveEndTime = time1[T3] + 350;
		}
		else if(abs(power) < 20 && moveStartTimer == 0)
		{
			moveStartTimer = 1;
			moveEndTime = time1[T3] + 350;
		}
		if(abs(error) > 51 && abs(power) > 21)
		{
			moveStartTimer = 0;
		}
		if(moveStartTimer && moveEndTime < time1[T3])
		{
			moveDone = 1;
		}
		if((time1[T3] - startTime) > 5000)
		{
			moveDone = 1;
		}
		lastError = error;
		lastTime = time1[T3];
		wait1Msec(75);
	}
}
task setUpChainBar()
{
	if(chainBarSetupDone == false)
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
