// BCI RobotC Library: https://github.com/VTOW/BCI
// LICENSE: GNU AFFERO GENERAL PUBLIC LICENSE Version 3
typedef struct pos_PID_t
{
	//PID constants
	float kP;
	float kI;
	float kD;
	float kBias;

	//PID calculations
	int error;
	int prevError;
	int integral;
	int derivative;

	//PID limits
	int errorThreshold;
	int integralLimit;
	int upperBound;
	int lowerBound;

	//Timestep
	float dt;
	int currentTime;
	int prevTime;

	//Input
	tSensors sensor;
	tMotor imeMotor;
	float *var;
	bool usingIME;
	bool usingVar;
	int targetPos;
	int currentPos;

	//Output
	int outVal;
} pos_PID;

/**
 * Initializes a position PID controller
 * @param pid            PID controller to initialize
 * @param sensor         Analog or digital sensor to read from
 * @param kP             Proportional gain
 * @param kI             Integral gain
 * @param kD             Derivative gain
 * @param kBias          Controller bias (Constant force applied)
 * @param errorThreshold Minimum error to sum for integral
 * @param integralLimit  Upper limit of integral term
 */
void pos_PID_InitController(pos_PID *pid, const tSensors sensor, const float kP, const float kI, const float kD, float kBias = 0.0, int errorThreshold = 0, int integralLimit = 1000000);

/**
 * Initializes a position PID controller
 * @param pid            PID controller to initialize
 * @param imeMotor       Name of motor with IME attached
 * @param kP             Proportional gain
 * @param kI             Integral gain
 * @param kD             Derivative gain
 * @param kBias          Controller bias (Constant force applied)
 * @param errorThreshold Minimum error to sum for integral
 * @param integralLimit  Upper limit of integral term
 */
void pos_PID_InitController(pos_PID *pid, const tMotor imeMotor, const float kP, const float kI, const float kD, float kBias = 0.0, const int errorThreshold = 0, int integralLimit = 1000000);

/**
 * Initializes a position PID controller
 * @param pid            PID controller to initialize
 * @param var            Float to read from
 * @param kP             Proportional gain
 * @param kI             Integral gain
 * @param kD             Derivative gain
 * @param kBias          Controller bias (Constant force applied)
 * @param errorThreshold Minimum error to sum for integral
 * @param integralLimit  Upper limit of integral term
 */
void pos_PID_InitController(pos_PID *pid, const float *var, const float kP, const float kI, const float kD, float kBias = 0.0, int errorThreshold = 0, int integralLimit = 1000000);

/**
 * Sets a new kBias term
 * @param pid   PID controller to use
 * @param kBias New kBias term
 */
void pos_PID_ChangeBias(pos_PID *pid, const float kBias);

/**
 * Sets new upper and lower bounds for the ouput of the controller
 * @param pid   PID controller to use
 * @param upper Upper bound
 * @param lower Lower bound
 */
void pos_PID_ChangeBounds(pos_PID *pid, const int upper, const int lower);

/**
 * Changes the sensor
 * @param pid    PID controller to use
 * @param sensor New sensor
 */
void pos_PID_ChangeSensor(pos_PID *pid, const tSensors sensor);

/**
 * Changes the sensor
 * @param pid      PID controller to use
 * @param imeMotor New IME motor
 */
void pos_PID_ChangeSensor(pos_PID *pid, const tMotor imeMotor);

/**
 * Changes the sensor
 * @param pid PID controller to use
 * @param var New float to read from
 */
void pos_PID_ChangeSensor(pos_PID *pid, const float *var);

/**
 * Sets the target position
 * @param pid       PID controller to use
 * @param targetPos New target position
 */
void pos_PID_SetTargetPosition(pos_PID *pid, const int targetPos);

/**
 * Returns the current error
 * @param  pid PID controller to use
 * @return     Current error
 */
int pos_PID_GetError(pos_PID *pid);

/**
 * Returns the current position
 * @param  pid PID controller to use
 * @return     Current position
 */
int pos_PID_GetPosition(pos_PID *pid);

/**
 * Returns the current output
 * @param  pid PID controller to use
 * @return     Current output
 */
int pos_PID_GetOutput(pos_PID *pid);

/**
 * Steps the controller's calculations
 * @param  pid PID controller to use
 * @return     New output
 */
int pos_PID_StepController(pos_PID *pid);

/**
 * Steps the controller's calculations
 * @param  pid PID controller to use
 * @param  val Current position
 * @return     New output
 */

int pos_PID_StepController(pos_PID *pid, const float val);

void pos_PID_InitController(pos_PID *pid, const tSensors sensor, const float kP, const float kI, const float kD, float kBias, int errorThreshold, int integralLimit)
{
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->kBias = kBias;

	pid->error = 0;
	pid->prevError = 0;
	pid->integral = 0;
	pid->derivative = 0;

	pid->errorThreshold = errorThreshold;
	pid->integralLimit = integralLimit;
	pid->upperBound = 127;
	pid->lowerBound = -127;

	pid->dt = 0;
	pid->currentTime = 0;
	pid->prevTime = 0;

	pid->sensor = sensor;
	pid->usingIME = false;
	pid->usingVar = false;
	pid->targetPos = SensorValue[sensor];
	pid->currentPos = 0;

	pid->outVal = 0;
}

void pos_PID_InitController(pos_PID *pid, const tMotor imeMotor, const float kP, const float kI, const float kD, float kBias, int errorThreshold, int integralLimit)
{
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->kBias = kBias;

	pid->error = 0;
	pid->prevError = 0;
	pid->integral = 0;
	pid->derivative = 0;

	pid->errorThreshold = errorThreshold;
	pid->integralLimit = integralLimit;
	pid->upperBound = 127;
	pid->lowerBound = -127;

	pid->dt = 0;
	pid->prevTime = 0;

	pid->imeMotor = imeMotor;
	pid->usingIME = true;
	pid->usingVar = false;
	pid->targetPos = nMotorEncoder[imeMotor];

	pid->outVal = 0;
}

void pos_PID_InitController(pos_PID *pid, const float *var, const float kP, const float kI, const float kD, float kBias, int errorThreshold, int integralLimit)
{
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->kBias = kBias;

	pid->error = 0;
	pid->prevError = 0;
	pid->integral = 0;
	pid->derivative = 0;

	pid->errorThreshold = errorThreshold;
	pid->integralLimit = integralLimit;
	pid->upperBound = 127;
	pid->lowerBound = -127;

	pid->dt = 0;
	pid->prevTime = 0;

	pid->var = var;
	pid->usingIME = false;
	pid->usingVar = true;
	pid->targetPos = *var;

	pid->outVal = 0;
}

void pos_PID_ChangeBias(pos_PID *pid, const float kBias)
{
	pid->kBias = kBias;
}

void pos_PID_ChangeBounds(pos_PID *pid, const int upper, const int lower)
{
	pid->upperBound = upper;
	pid->lowerBound = lower;
}

void pos_PID_ChangeSensor(pos_PID *pid, const tSensors sensor)
{
	pid->sensor = sensor;
	pid->usingIME = false;
	pid->usingVar = false;
}

void pos_PID_ChangeSensor(pos_PID *pid, const tMotor imeMotor)
{
	pid->imeMotor = imeMotor;
	pid->usingIME = true;
	pid->usingVar = false;
}

void pos_PID_ChangeSensor(pos_PID *pid, const float *var)
{
	pid->var = var;
	pid->usingVar = true;
	pid->usingIME = false;
}

void pos_PID_SetTargetPosition(pos_PID *pid, const int targetPos)
{
	pid->targetPos = targetPos;
}

int pos_PID_GetError(pos_PID *pid)
{
	return pid->error;
}

int pos_PID_GetPosition(pos_PID *pid)
{
	return pid->currentPos;
}

int pos_PID_GetOutput(pos_PID *pid)
{
	return pid->outVal;
}

int pos_PID_StepController(pos_PID *pid)
{
	//Calculate error
	if (pid->usingIME)
	{
		//Calculate timestep
		getEncoderAndTimeStamp(pid->imeMotor, pid->currentPos, pid->currentTime);
		pid->dt = (pid->currentTime - pid->prevTime) / 1000.0;
		pid->prevTime = pid->currentTime;

		//Scrap dt if zero
		if (pid->dt == 0)
		{
			return 0;
		}

		pid->error = pid->targetPos - pid->currentPos;
	}
	else if (pid->usingVar)
	{
		//Calculate timestep
		pid->dt = (nSysTime - pid->prevTime) / 1000.0;
		pid->prevTime = nSysTime;

		//Scrap dt if zero
		if (pid->dt == 0)
		{
			return 0;
		}

		pid->currentPos = *(pid->var);
		pid->error = pid->targetPos - pid->currentPos;
	}
	else
	{
		//Calculate timestep
		pid->dt = (nSysTime - pid->prevTime) / 1000.0;
		pid->prevTime = nSysTime;

		//Scrap dt if zero
		if (pid->dt == 0)
		{
			return 0;
		}

		pid->currentPos = SensorValue[pid->sensor];
		pid->error = pid->targetPos - pid->currentPos;
	}

	//If error is large enough, calculate integral and limit to avoid windup
	if (abs(pid->error) > pid->errorThreshold && abs(pid->integral) < pid->integralLimit)
	{
		pid->integral = pid->integral + pid->error * pid->dt;

		//Reset integral if reached target or overshot
		if (pid->error == 0 || sgn(pid->error) != sgn(pid->prevError))
		{
			pid->integral = 0;
		}
		//Bound integral
		else
		{
			pid->integral = pid->integral * pid->kI > 127 ? 127.0 / pid->kI : pid->integral;
			pid->integral = pid->integral * pid->kI < -127 ? -127.0 / pid->kI : pid->integral;
		}
	}

	//Calculate derivative
	pid->derivative = (pid->error - pid->prevError) / pid->dt;
	pid->prevError = pid->error;

	//Calculate output
	pid->outVal = (pid->error * pid->kP) + (pid->integral * pid->kI) + (pid->derivative * pid->kD) + pid->kBias;

	//Bound output
	if (pid->outVal > pid->upperBound)
	{
		pid->outVal = pid->upperBound;
	}
	else if (pid->outVal < pid->lowerBound)
	{
		pid->outVal = pid->lowerBound;
	}

	return pid->outVal;
}

int pos_PID_StepController(pos_PID *pid, const float val)
{
	//Calculate timestep
	pid->dt = (nSysTime - pid->prevTime) / 1000.0;
	pid->prevTime = nSysTime;

	//Scrap dt if zero
	if (pid->dt == 0)
	{
		return 0;
	}

	//Calculate error
	pid->error = pid->targetPos - val;

	//If error is large enough, calculate integral and limit to avoid windup
	if (abs(pid->error) > pid->errorThreshold && abs(pid->integral) < pid->integralLimit)
	{
		pid->integral = pid->integral + pid->error * pid->dt;

		//Reset integral if reached target or overshot
		if (pid->error == 0 || sgn(pid->error) != sgn(pid->prevError))
		{
			pid->integral = 0;
		}
		//Bound integral
		else
		{
			pid->integral = pid->integral * pid->kI > 127 ? 127.0 / pid->kI : pid->integral;
			pid->integral = pid->integral * pid->kI < -127 ? -127.0 / pid->kI : pid->integral;
		}
	}

	//Calculate derivative
	pid->derivative = (pid->error - pid->prevError) / pid->dt;
	pid->prevError = pid->error;

	//Calculate output
	pid->outVal = (pid->error * pid->kP) + (pid->integral * pid->kI) + (pid->derivative * pid->kD) + pid->kBias;

	//Bound output
	if (pid->outVal > pid->upperBound)
	{
		pid->outVal = pid->upperBound;
	}
	else if (pid->outVal < pid->lowerBound)
	{
		pid->outVal = pid->lowerBound;
	}

	return pid->outVal;
}

void SUPPRESS()
{
	SUPPRESS();
	pos_PID x;
	pos_PID_InitController(&x, in1, 0,0,0);
	pos_PID_ChangeBias(&x, 0);
	pos_PID_ChangeBounds(&x, 0,0);
	pos_PID_ChangeSensor(&x, in1);
	pos_PID_ChangeSensor(&x, port2);
	pos_PID_GetError(&x);
	pos_PID_GetPosition(&x);
	pos_PID_GetOutput(&x);
}
