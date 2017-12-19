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
		displayLCDString(0, 0, "Mobile Goal RED");
		break;
	case    1:
		displayLCDString(0, 0, "Mobile Goal Blue");
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

void degmove(int distance)
{
	pos_PID left, right, turn;
	pos_PID_InitController(&left, frontLeft, 0.02, 0, 0.01);
	pos_PID_InitController(&right, frontRight, 0.02, 0, 0.01);
	pos_PID_InitController(&turn, gyro, 0.04, 0, 0.00);
	distance *= 15;
	int leftGoal = nMotorEncoder[frontLeft] + distance;
	int rightGoal = nMotorEncoder[frontRight] + distance;
	pos_PID_SetTargetPosition(&left, leftGoal);
	pos_PID_SetTargetPosition(&right, rightGoal);
	pos_PID_SetTargetPosition(&turn, 0);
	int leftDone = 0, rightDone = 0, turnDone = 0;
	int leftStartTimer = 0, leftEndTime = 0;
	int rightStartTimer = 0, rightEndTime = 0;
	int turnStartTimer = 0, turnEndTime = 0;
	while(leftDone == 0 || rightDone == 0 || turnDone == 0)
	{
		SetMotor(frontLeft, pos_PID_StepController(&left) + pos_PID_StepController(&turn));
		SetMotor(backLeft, pos_PID_StepController(&left) + pos_PID_StepController(&turn));
		SetMotor(frontRight, pos_PID_StepController(&right) - pos_PID_StepController(&turn));
		SetMotor(backRight, pos_PID_StepController(&right) - pos_PID_StepController(&turn));
		// check if left side is done
		if(abs(pos_PID_GetError(&left)) < 50)
		{
			if(leftStartTimer == 0)
				leftEndTime = time10[T1] + 75;
			leftStartTimer = 1;
		}
		else
		{
			leftStartTimer = 0;
		}
		if(leftStartTimer && time10[T1] > leftEndTime)
		{
			leftDone = 1;
		}
		else
		{
			leftDone = 0;
		}
		// check if right side is done
		if(abs(pos_PID_GetError(&right)) < 50)
		{
			if(rightStartTimer == 0)
				rightEndTime = time10[T1] + 75;
			rightStartTimer = 1;
		}
		else
		{
			rightStartTimer = 0;
		}
		if(rightStartTimer && time10[T1] > rightEndTime)
		{
			rightDone = 1;
		}
		else
		{
			rightDone = 0;
		}
		// check if turn is done
		if(abs(pos_PID_GetError(&turn)) < 50)
		{
			if(turnStartTimer == 0)
				turnEndTime = time10[T1] + 75;
			turnStartTimer = 1;
		}
		else
		{
			turnStartTimer = 0;
		}
		if(turnStartTimer && time10[T1] > turnEndTime)
		{
			turnDone = 1;
		}
		else
		{
			turnDone = 0;
		}
	}
}
void gyroturn(int degs)
{
	pos_PID turn;
	degs += SensorValue[gyro];
	pos_PID_InitController(&turn, gyro, 0.04, 0, 0.00);
	pos_PID_SetTargetPosition(&turn, degs);
	int turnStartTimer = 0, turnEndTime = 0;
	int turnDone = 0;
	while(turnDone == 0)
	{
		SetMotor(frontLeft, pos_PID_StepController(&turn));
		SetMotor(backLeft, pos_PID_StepController(&turn));
		SetMotor(frontRight, -pos_PID_StepController(&turn));
		SetMotor(backRight, -pos_PID_StepController(&turn));
		if(abs(pos_PID_GetError(&turn)) < 50)
		{
			if(turnStartTimer == 0)
				turnEndTime = time10[T1] + 75;
			turnStartTimer = 1;
		}
		else
		{
			turnStartTimer = 0;
		}
		if(turnStartTimer && time10[T1] > turnEndTime)
		{
			turnDone = 1;
		}
		else
		{
			turnDone = 0;
		}
	}
}


//CONFIG PARAMETERS
int doublePreload = 100;
int doubleSetpoint = doubleDown;
<<<<<<< HEAD
int doubleMobileGoal = 25;
int doubleKP = 25, doubleKI = 0, doubleKD = 7, doubleDIVISOR = 100;
=======
>>>>>>> 82c75b9e38050d4d59a5bac07c0ee40cbb95e341
int doubleDone = 0;
int doubleStackLoader = 0;
int doublePIDActive = 1;

int mobileGoalDown = 1750;
int mobileGoalUp = 0;
#define mobileKD 7
int mobileGoalSetpoint = mobileGoalUp;
int mobileKP = 25, mobileKI = 0, mobileKD = 7, mobileDIVISOR = 100;
int mobileDone = 0;
int mobilePIDActive = 1;

int chainBarUp = 0;
int chainBarDown = 2000;
int chainBarSetpoint = chainBarUp;
<<<<<<< HEAD
int chainBarMobileGoal = 1000;
int chainBarKP = 25, chainBarKI = 0, chainBarKD = 7, chainBarDIVISOR = 100;
=======
>>>>>>> 82c75b9e38050d4d59a5bac07c0ee40cbb95e341
int chainBarDone = 0;
int chainBarPIDActive = 1;

int clawOpen = 0;
int clawClosed = 2000;
int clawSetpoint = clawOpen;
int clawKP = 25, clawKI = 0, clawKD = 7, clawDIVISOR = 100;
int clawDone = 0;
int clawPIDActive = 1;

int activateAutoStacker = 0;
int currentStacked = 0;
int pidActive = 1;


task WATCHDOG
{
	int doubleStartTimer, doubleEndTime;
	int mobileStartTimer, mobileEndTime;
	int chainBarStartTimer, chainBarEndTime;
	int clawStartTimer, clawEndTime;
	// DR4B PID Controller
	pos_PID doublePID;
	pos_PID_InitController(&doublePID, doubleRight, doubleKP, doubleKI, doubleKD);
	// Chain Bar PID Controller
	pos_PID chainbarPID;
	pos_PID_InitController(&chainbarPID, chainbar, chainBarKP, chainBarKI, chainBarKD);
	// Mobile Goal PID Controller
	pos_PID mobilePID;
	pos_PID_InitController(&mobilePID, mobileGoal, mobileKP, mobileKI, mobileKD);
	// Claw PID Controller
	pos_PID clawPIDController;
	pos_PID_InitController(&clawPIDController, clawPot, clawKP, clawKI, clawKD);

	while(1)
	{
		if(pidActive)
		{
			pos_PID_SetTargetPosition(&doublePID, doubleSetpoint);
			pos_PID_SetTargetPosition(&chainbarPID, chainBarSetpoint);
			pos_PID_SetTargetPosition(&mobilePID, mobileGoalSetpoint);
			pos_PID_SetTargetPosition(&clawPIDController, clawSetpoint);
			int mobileGoalPower = pos_PID_StepController(&mobilePID);
			if(mobileGoalPower > 15
				&& abs(nMotorEncoder[doubleRight] - doubleMobileGoal) < 50
			&& abs(nMotorEncoder[chainbar] - chainBarMobileGoal) < 50)
			{
				doubleSetpoint =  max(doubleSetpoint, doubleMobileGoal);
				chainBarSetpoint = min(chainBarSetpoint, chainBarMobileGoal);
				pos_PID_SetTargetPosition(&doublePID, doubleSetpoint);
				pos_PID_SetTargetPosition(&chainbarPID, chainBarSetpoint);
				mobileGoalPower = 0;
			}
			if(doublePIDActive)
			{
				SetMotor(doubleRight, pos_PID_StepController(&doublePID) / doubleDIVISOR);
				SetMotor(doubleLeft, pos_PID_StepController(&doublePID) / doubleDIVISOR);
			}
			if(chainBarPIDActive)
			{
				SetMotor(chainbar, pos_PID_StepController(&chainbarPID) / chainBarDIVISOR);
			}
			if(mobilePIDActive)
			{
				SetMotor(mobileGoal,  mobileGoalPower / mobileDIVISOR);
			}
			if(clawPIDActive)
			{
				SetMotor(claw, pos_PID_StepController(&clawPIDController) / clawDIVISOR);
			}
			// Dones
			if(abs(pos_PID_GetError(&doublePID)) < 50)
			{
				if(doubleStartTimer == 0)
					doubleEndTime = time10[T1] + HOLDOUT;
				doubleStartTimer = 1;
			}
			else
			{
				doubleStartTimer = 0;
			}
			if(doubleStartTimer && time10[T1] > doubleEndTime)
			{
				doubleDone = 1;
			}
			else
			{
				doubleDone = 0;
			}
			if(abs(pos_PID_GetError(&mobilePID)) < 50)
			{
				if(mobileStartTimer == 0)
					mobileEndTime = time10[T1] + HOLDOUT;
				mobileStartTimer = 1;
			}
			else
			{
				mobileStartTimer = 0;
			}
			if(mobileStartTimer && time10[T1] > mobileEndTime)
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
					chainBarEndTime = time10[T1] + HOLDOUT;
				chainBarStartTimer = 1;
			}
			else
			{
				chainBarStartTimer = 0;
			}
			if(chainBarStartTimer && time10[T1] > chainBarEndTime)
			{
				chainBarDone = 1;
			}
			else
			{
				chainBarDone = 0;
			}
			if(abs(pos_PID_GetError(&clawPIDController)) < 50)
			{
				if(clawStartTimer == 0)
					clawEndTime = time10[T1] + HOLDOUT;
				clawStartTimer = 1;
			}
			else
			{
				clawStartTimer = 0;
			}
			if(clawStartTimer && time10[T1] > clawEndTime)
			{
				clawDone = 1;
			}
			else
			{
				clawDone = 0;
			}
			wait1Msec(25);
		}
	}
}
task autoStacker
{
	int innerState = 0;
	int lastAutostacker = 0;
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
		if(activateAutoStacker && mobileDone)
		{
			if(innerState == 0)
			{
				clawSetpoint = clawClosed;
				innerState++;
			}
			else if(innerState == 1)
			{
				if(clawDone)
				{
					doubleSetpoint = doubleUp[currentStacked];
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
				if(chainBarDone)
				{
					clawSetpoint = clawOpen;
					innerState++;
				}
			}
			else if(innerState == 4)
			{
				if(clawDone)
				{
					chainBarSetpoint = chainBarDown;
					innerState++;
				}
			}
			else if(innerState == 5)
			{
				if(chainBarDone)
				{
					doubleSetpoint = doubleDown;
					innerState++;
				}
			}
			else if(innerState == 6)
			{
				if(doubleDone)
				{
					currentStacked ++;
					activateAutoStacker = 0;
					innerState = 0;
				}
			}
		}
	}
}
