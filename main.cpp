//July 4th

//around ball is now smoother

//to do: make sure linear speed & whiteLine are working, and work on calculateGoalDir

// linear: done (?). Inverted. close = fast, far = slow (between 30 and 40)
// whiteline: almost done. positionFlag not working.
//score function: occasionally works, I guess


#include "HardwareInfo.c"
#include <GetAdUltrasound.h>
#include <SetLCD5Char.h>
#include <GetRemoIR.h>
#include <GetADScable10.h>
#include <SetLED.h>
#include <GetSysTimeMs.h>
#include <SetSysTime.h>
#include <SetLCDSolidCircle.h>
#include <SetMotor.h>
#include <SetLCDString.h>
#include <SetLCDClear.h>
#include <stdbool.h>

#define STOP 360
#define TURNING_SPEED 15
#define MAX_SPEED 70

#define AREA_TOP_LEFT 1
#define AREA_TOP_RIGHT 2
#define AREA_BOTTOM_LEFT 3
#define AREA_BOTTOM_RIGHT 4

const int MAX_BACKPOSITION_SPEED = 35;
const int BASE_BACKPOSITION_SPEED = 25;
const int BLOCKED_SPEED = 10;

const int MAX_CHASEBALL_SPEED = 40;
const int BASE_CHASEBALL_SPEED = 30;

const int BASE_AROUNDBALL_SPEED = 30;
const int MAX_AROUNDBALL_SPEED = 40;

int MAX_AIRWALL_SPEED = 0;
const int BASE_AIRWALL_SPEED = 0;

static int angle;
float cachedGoalDir = -1;


//Ultrasonic
int backultra=0;
int frontultra=0;
int leftultra=0;
int rightultra=0;
int hBlockThres=600;
int vBlockThres=600;
const int Y_MAX = 2000;
const int X_MAX = 1600;

//Whiteline
int grayscaleF = 0; //there is no gray scale front sensor
int grayscaleL = 0;
int grayscaleR = 0;
int grayscaleB = 0;


//Blocked
int blockedX = 0;
int blockedY = 0;

//BackToCenter
long targetX = 0;
long targetY = 0;
int positionFlag=0;

//Ball
int eyePort = 0;
int compoundEyeValue = 0;
int distanceToBall = 0; //strength
int ballAngle = 0;

// Move
const int BACK_POSITION_COMPASS_RANGE =20;
const int BALL_COMPASS_RANGE =20;
int compassRange = 25;

const int MOTOR_DIR[5] = {0,1,1,1,1};

// Speed
int dir = 0;
int speed = 0;
int motorSpeed = 0; 

//t-angle
float targetAngle = 0;


//laser
int laser = 0;

static long long lastIncrement = 0;


//Compass

// Vector
struct Vector{
	int speed;
	int angle;
};

struct PositionInfo{
	int area;
	long long timestamp;
};

struct Vector combineVectors(struct Vector v1, struct Vector v2);
struct PositionInfo positionInfo;

float constrainFloat(float input, float lower, float upper)
{
	if(input < lower)
	{
		return(lower);
	}
	else if(input > upper)
	{
		return(upper);
	}
	else
	{
		return(input);
	}
}


int constrain(double input, int lower, int upper)
{
	if(input < lower)
	{
		return(lower);
	}
	else if(input > upper)
	{
		return(upper);
	}
	else
	{
		return(input);
	}
}

double constrainDouble(double input, double lower, double upper)
{
	if(input < lower)
	{
		return(lower);
	}
	else if(input > upper)
	{
		return(upper);
	}
	else
	{
		return(input);
	}
}




// === Function: move ===
void move(int d, int s, int targetAngle){
	d = constrain(d, 0,360);
    long angleDiff = 0;
    long addition = 0;
    long BaseDirection = 0;
    long ExtraDirection = 0;
    long var0 = 0;
    int compassAngle = getCompass();
    
    int motorSpeed = s;
    long motor1;
    long motor2;
    long motor3;
    long motor4;
    int spinSpeed = TURNING_SPEED;
    
    angleDiff = getAngleDif(compassAngle, targetAngle);
    
    if ( abs(angleDiff)< compassRange )
    {
        if ( d<=45 )
        {
            ExtraDirection=tan((45-d)*0.0174533)*motorSpeed;
            motor1 =ExtraDirection;
            motor2=motorSpeed;
            motor3 = ExtraDirection*(0-1);
            motor4 =motorSpeed*(0-1);
        }
        else
        {
            if ( d<=90 )
            {
                ExtraDirection=tan((d-45)*0.0174533)*motorSpeed;
                motor1 =ExtraDirection*(0-1);
                motor2=motorSpeed;
                motor3 = ExtraDirection;
                motor4 = motorSpeed*(0-1);
            }
            else
            {
                if ( d<=135 )
                {
                    ExtraDirection=tan((135-d)*0.0174533)*motorSpeed;
                    motor1 =motorSpeed*(0-1);
                    motor2=ExtraDirection;
                    motor3 = motorSpeed;
                    motor4 = ExtraDirection*(0-1);
                }
                else
                {
                    if ( d<=180 )
                    {
                        ExtraDirection=tan((d-135)*0.0174533)*motorSpeed;
                        motor1 = motorSpeed*(0-1);
                        motor2=ExtraDirection*(0-1);
                        motor3 = motorSpeed;
                        motor4 = ExtraDirection;
                    }
                    else
                    {
                        if ( d<=225 )
                        {
                            ExtraDirection=tan((225-d)*0.0174533)*motorSpeed;
                            motor1 = ExtraDirection*(0-1);
                            motor2=motorSpeed*(0-1);
                            motor3 = ExtraDirection;
                            motor4 = motorSpeed;
                        }
                        else
                        {
                            if ( d<=270 )
                            {
                                ExtraDirection=tan((d-225)*0.0174533)*motorSpeed;
                                motor1 = ExtraDirection;
                                motor2=motorSpeed*(0-1);
                                motor3 = ExtraDirection*(0-1);
                                motor4 =motorSpeed;
                            }
                            else
                            {
                                if ( d<=315 )
                                {
                                    ExtraDirection=tan((315-d)*0.0174533)*motorSpeed;
                                    motor1 = motorSpeed;
                                    motor2=ExtraDirection*(0-1);
                                    motor3 = motorSpeed*(0-1);
                                    motor4 = ExtraDirection;
                                }
                                else
                                {
                                    if ( d<360 )
                                    {
                                        ExtraDirection=tan((d-315)*0.0174533)*motorSpeed;
                                        motor1 = motorSpeed;
                                        motor2=ExtraDirection;
                                        motor3 =motorSpeed*(0-1);
                                        motor4 = ExtraDirection*(0-1);
                                    }
                                    else
                                    {
                                        if ( d==360 )
                                        {
                                            motor1 =0;
                                            motor2=0;
                                            motor3 = 0;
                                            motor4 = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        addition=angleDiff/5;
        /*
        if(hasBall()){
		    addition = angleDiff;
        } 
        */
    }
    else{
    
        if ( angleDiff>0 )
            motor1=motor2=motor3=motor4=spinSpeed*(0-1);
        else
            motor1=motor2=motor3=motor4=spinSpeed;
            
        addition=0;        
    }

    motor1 = motor1 - addition;
    motor2 = motor2 - addition;
    motor3 = motor3 - addition;
    motor4 = motor4 - addition;
    setMotors(motor1,motor2,motor3,motor4);
}





// === Function: setMotors ===
void setMotors(int speed1, int speed2, int speed3, int speed4) {

	// Do not touch
	speed1 *= MOTOR_DIR[1];
	speed2 *= MOTOR_DIR[2];
	speed3 *= MOTOR_DIR[3];
	speed4 *= MOTOR_DIR[4];
	
	// Speed Limit
	if(speed1 > MAX_SPEED){
		speed1 = MAX_SPEED;
	}
	else if (speed1 < -MAX_SPEED){
		speed1 = -MAX_SPEED;
	}
	
	if(speed2 > MAX_SPEED){
		speed2 = MAX_SPEED;
	}
	else if (speed2 < -MAX_SPEED){
		speed2 = -MAX_SPEED;
	}
	
	if(speed3 > MAX_SPEED){
		speed3 = MAX_SPEED;
	}
	else if (speed3 < -MAX_SPEED){
		speed3 = -MAX_SPEED;
	}
	
	if(speed4 > MAX_SPEED){
		speed4 = MAX_SPEED;
	}
	else if (speed4 < -MAX_SPEED){
		speed4 = -MAX_SPEED;
	}
		
	if(speed1 < 0){
		SetMotor(_MOTOR_M1_, 2, abs(speed1));
	}else if(speed1 == 0){
		SetMotor(_MOTOR_M1_, 1, speed1);
	}else{
		SetMotor(_MOTOR_M1_, 0, speed1);
	}
 
	 if(speed2 < 0){
		SetMotor(_MOTOR_M2_, 2, abs(speed2));
	 }else if(speed2 == 0){
		SetMotor(_MOTOR_M2_, 1, speed2);
	 }else{
		SetMotor(_MOTOR_M2_, 0, speed2);
	 }
	 
	 if(speed3 < 0){
		SetMotor(_MOTOR_M3_, 2, abs(speed3));
	 }else if(speed3 == 0){
		SetMotor(_MOTOR_M3_, 1, speed3);
	 }else{
		SetMotor(_MOTOR_M3_, 0, speed3);
	 }
	 
	 if(speed4 < 0){
		SetMotor(_MOTOR_M4_, 2, abs(speed4));
	 }else if(speed4 == 0){
		SetMotor(_MOTOR_M4_, 1, speed4);
	 }else{
		SetMotor(_MOTOR_M4_, 0, speed4);
	 }
}



// === Function: getAngleDif ===
int getAngleDif(int currentAngle, int targetAngle) {
    int diff = currentAngle - targetAngle;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    return diff;
}

// === Function: isHorizontalBlocked ===
int isHorizontalBlocked(int uLeft, int uRight){
	return uRight+uLeft<1000;
}


// === Function: isVerticalBlocked ===
int isVerticalBlocked(int uFront, int uBack){
	return uBack+uFront<1200;

}


/*if(isHorizontalBlocked(ultrasoundLeft, ultrasoundRight)||isVerticalBlocked(ultrasoundFront, ultrasoundBack)){
			positionFlag = 0;
			blocked(ultrasoundBack, ultrasoundFront, ultrasoundLeft, ultrasoundRight);
		}
		
	{*/



// === Function: backPosition ===
void backPosition(int tarX, int tarY, int whiteLinePosition){
	motorSpeed = BASE_BACKPOSITION_SPEED;
	compassRange = BACK_POSITION_COMPASS_RANGE;
	int ultrasoundBack = getUltraB();
	int ultrasoundFront = getUltraF();
	int ultrasoundLeft = getUltraL();
	int ultrasoundRight = getUltraR();
	
	lastIncrement = GetSysTimeMs();
	
	
	
	
	/*
	if (isVerticalBlocked(ultrasoundBack, ultrasoundFront)) {
		// Blocked
		if (positionInfo.area < 3){
			//Trust Front
			ultrasoundBack = Y_MAX - constrain(ultrasoundFront, 0, Y_MAX);
		} else {
			//Trust Back
			ultrasoundFront =  Y_MAX - constrain(ultrasoundBack, 0, Y_MAX);
		}
	}
		
	if (isHorizontalBlocked(ultrasoundLeft, ultrasoundRight)) {
		if (positionInfo.area % 2 == 0){
			//Trust Right
			ultrasoundLeft = X_MAX - constrain(ultrasoundRight, 0, X_MAX);
		} else {
			//Trust Left
			ultrasoundRight = X_MAX - constrain(ultrasoundLeft, 0, X_MAX);
		}
	}
	*/
	
	
	SetLCDString(30, 20, "Ultrasonic", WHITE, BLACK);
	SetLCD5Char(35, 60, ultrasoundFront, YELLOW, BLACK);
	SetLCD5Char(10, 90, ultrasoundLeft, YELLOW, BLACK);
	SetLCD5Char(60, 90, ultrasoundRight, YELLOW, BLACK);
	SetLCD5Char(35, 120, ultrasoundBack, YELLOW, BLACK);
	
	SetLCD5Char(230, 150, positionInfo.area, YELLOW, BLACK);

	long posX = 0;
	long posY = 0;
	long ratio = 0;
	long distance = 0;
	

	if ( positionFlag > 2 && !whiteLinePosition)
	{
		SetLCDString(200, 100, "park ", YELLOW, BLACK);
		dir = STOP;
		return;
	}
		
			//SetLCDString(200, 100, "clear", YELLOW, BLACK);
			//posY = ultrasoundBack-ultrasoundFront;
			posY = ultrasoundBack-ultrasoundFront - tarY;
			//-targetY;
			//-targetY;
			posX = ultrasoundLeft-ultrasoundRight-tarX;
			//-targetX;
			//SetLCD5Char(20, 20, posX, RED, BLACK);
			//SetLCD5Char(80, 20, posY, RED, BLACK);
			distance=sqrt(posX*posX+posY*posY);
			
			SetLCD5Char(10, 210, posX, RED, BLACK);
			SetLCD5Char(80, 210, posY, RED, BLACK);
			SetLCD5Char(150, 210, distance, CYAN, BLACK);
			
			
			if (distance > 30){
			    positionFlag = 0;
			    motorSpeed=motorSpeed+(distance/200);
			    motorSpeed = constrain(motorSpeed, BASE_BACKPOSITION_SPEED, MAX_BACKPOSITION_SPEED);
	
			    dir = atan((posX*1.0)/(posY))*57.2958;
			    if ( posY>0 )
			    {
				if ( posX>0 )
				{
				    dir = 180+dir;
				}
				else
				{
				    dir = 180-abs(dir);
				}
			    }
			    else
			    {
				if ( posX>0 )
				{
				    dir = 360-abs(dir);
				}
				else
				{
				    dir = dir;
				}
			    }
			    if ( dir>360 )
			    {
				dir=dir-360;
			    }
			}
			else
			{
			    positionFlag+=1;
			    lastIncrement = GetSysTimeMs();
			    dir = 360;
			    
			}
		    //}
		//}
	    //}

	    speed = motorSpeed;
}

//calculates one of four quadrants (top left = 1, top right = 2, bottom left = 3, bottom right = 4)

void calculateCoord(){

	int compassAngle = getCompass();
	if(abs(getAngleDif(compassAngle, 0) > 15)) return;

	int ultraB = getUltraB();
	int ultraL = getUltraL();
	int ultraR = getUltraR();
	int ultraF = getUltraF();
	
	if (!isHorizontalBlocked(ultraL, ultraR) && !isVerticalBlocked(ultraB, ultraF)){
		int area = 0;
		
		int posX = ultraL - ultraR;
		int posY = ultraB - ultraF;
		
		if(posX > 0 && posY > 0){
			area = AREA_TOP_RIGHT;
		} else if (posX <= 0 && posY > 0) {
			area = AREA_TOP_LEFT;
		} else if (posX > 0 && posY <= 0) {
			area = AREA_BOTTOM_RIGHT;
		} else {
			area = AREA_BOTTOM_LEFT;
		}
		
		
		positionInfo.area = area;
		positionInfo.timestamp = GetSysTimeMs();
	}
}
	


//Getter methods for BALL
int getEyeValue(){
	return GetRemoIR(_FLAMEDETECT_port_);
}

int getDistanceBall(){
	return GetRemoIR(_FLAMEDETECT_distanceBall_);
}
	

int getEyeAngle(){
	return (getEyePort(getEyeValue()) - 1)*20;
}

int getEyePort(int compoundEyeValue){

    if ( compoundEyeValue < 175 )
    {
        eyePort = 1;
    }
    else
    {
        if ( compoundEyeValue < 310)
        {
            eyePort = 2;
        }
        else
        {
            if ( compoundEyeValue < 440 )
            {
                eyePort = 3;
            }
            else
            {
                if ( compoundEyeValue < 575 )
                {
                    eyePort = 4;
                }
                else
                {
                    if ( compoundEyeValue < 700 )
                    {
                        eyePort = 5;
                    }
                    else
                    {
                        if ( compoundEyeValue < 820 )
                        {
                            eyePort = 6;
                        }
                        else
                        {
                            if ( compoundEyeValue < 945 )
                            {
                                eyePort = 7;
                            }
                            else
                            {
                                if ( compoundEyeValue < 1070 )
                                {
                                    eyePort = 8;
                                }
                                else
                                {
                                    if ( compoundEyeValue < 1190 )
                                    {
                                        eyePort = 9;
                                    }
                                    else
                                    {
                                        if ( compoundEyeValue < 1310 )
                                        {
                                            eyePort = 10;
                                        }
                                        else
                                        {
                                            if ( compoundEyeValue < 1430 )
                                            {
                                                eyePort = 11;
                                            }
                                            else
                                            {
                                                if ( compoundEyeValue < 1545 )
                                                {
                                                    eyePort = 12;
                                                }
                                                else
                                                {
                                                    if ( compoundEyeValue < 1650 )
                                                    {
                                                        eyePort = 13;
                                                    }
                                                    else
                                                    {
                                                        if ( compoundEyeValue < 1745 )
                                                        {
                                                            eyePort = 14;
                                                        }
                                                        else
                                                        {
                                                            if ( compoundEyeValue < 1835 )
                                                            {
                                                                eyePort = 15;
                                                            }
                                                            else
                                                            {
                                                                if ( compoundEyeValue < 1915 )
                                                                {
                                                                    eyePort = 16;
                                                                }
                                                                else
                                                                {
                                                                    if ( compoundEyeValue < 1985 )
                                                                    {
                                                                        eyePort = 17;
                                                                    }
                                                                    else
                                                                    {
                                                                        eyePort = 18;
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return eyePort;
 }
 
int realDistanceHelper(int rawDist) {
    int compassAngle = getCompass();
    int absAngleDiff = abs(getAngleDif(compassAngle, 0));
    double theta = (double)absAngleDiff * M_PI / 180.0;
    return (int)(rawDist * fabs(cos(theta)));
}

//Getter methods for Ultrasonics
int getUltraL(){
	return realDistanceHelper(GetAdUltrasound(_ADULTRASOUND_left_));
}

int getUltraR(){
	return realDistanceHelper(GetAdUltrasound(_ADULTRASOUND_right_));
}

int getUltraF(){
	return realDistanceHelper(GetAdUltrasound(_ADULTRASOUND_front_));
}

int getUltraB(){
	return realDistanceHelper(GetAdUltrasound(_ADULTRASOUND_back_));
}

//Getter methods for Grayscales
int getGrayscaleL(){
	return GetADScable10(_SCABLEAD_LF_);
}

int getGrayscaleR(){
	return GetADScable10(_SCABLEAD_RF_);
}

int getGrayscaleB(){
	return GetADScable10(_SCABLEAD_back_);
}

int getCompass()
{
	// Upon the sensor we are choosing
	//return getJMCompass(); 
	return  getMPUCompass();
}


//Getter method(s) for Laser
int getLaser() {
	return GetRemoIR(_FLAMEDETECT_laser_);
}

static float wrap360(float a) {
    a = fmodf(a, 360.0f);
    if (a < 0) a += 360.0f;
    return a;
}


float calculateGoalDirSnapshot() {
    const float goalX = 500;  // right/left ultrasonic reading at the goal
    const float goalY = 150;  // front ultrasonic reading at the goal

    // current sensor readings
    float ux = getUltraR();
    float uy = getUltraF();
    
    float ux_L = getUltraL();

    // sensor?robot frame
    float dx = ux - goalX;
    float dy = uy - goalY;
    
    float dx_L = goalX - ux_L;

    // 0° = +Y (north), increasing clockwise:
    
    float ang_L = atan2(dx_L, dy) * (180.0f / M_PI);
    float ang = atan2f(dx, dy) * (180.0f / M_PI);
    
    if (ang_L < 0) ang_L += 360.0f;
    if (ang < 0) ang += 360.0f;
    
    ang = wrap360(ang);
    ang_L = wrap360(ang_L);
    
    //maintain accuracy 
    if (ux_L > ux) {
	return ang_L;
    } else {
	return ang;
    }
}



//returns MPU compass
int getMPUCompass()
{
	int comp = GetRemoIR(_FLAMEDETECT_compass_)/5.0;
	return constrain(comp, 0, 359);
}

void display(){
	int compassAngle = getCompass();
	SetLCDString(145, 70, "MPU:", CYAN, BLACK);
	SetLCD5Char(135, 90, compassAngle, CYAN, BLACK);
	
	SetLCDString(145, 20, "Dir:", GREEN, BLACK);
	SetLCD5Char(135, 40, dir, GREEN, BLACK);
	
	SetLCDString(145, 120, "Speed:", GREEN, BLACK);
	SetLCD5Char(135, 140, speed, GREEN, BLACK);
	
	SetLCDString(120, 170, "Ball:", YELLOW, BLACK);
	SetLCD5Char(110, 190, getEyeAngle(), YELLOW, BLACK);
	
	SetLCDString(170, 170, "Dist:", YELLOW, BLACK);
	SetLCD5Char(160, 190, getDistanceBall(), YELLOW, BLACK);

	SetLCDString(30, 20, "Ultrasonic", WHITE, BLACK);
	SetLCD5Char(35, 60, getUltraF(), YELLOW, BLACK);
	SetLCD5Char(10, 90, getUltraL(), YELLOW, BLACK);
	SetLCD5Char(60, 90, getUltraR(), YELLOW, BLACK);
	SetLCD5Char(35, 120, getUltraB(), YELLOW, BLACK);
	
	SetLCDString(220, 20, "Grayscale", WHITE, BLACK);
	SetLCD5Char(225, 60, "N/A", YELLOW, BLACK); //front grayscale not available
	
	SetLCD5Char(200, 90, getGrayscaleL(), YELLOW, BLACK);
	SetLCD5Char(250, 90, getGrayscaleR(), YELLOW, BLACK);
	SetLCD5Char(225, 120, getGrayscaleB(), YELLOW, BLACK);
	
	//SetLCDSolidCircle(70, 180, 30, YELLOW);	
}	


int min(int x, int y) {
	if (y < x) {
		return y;
	} else if (x < y) {
		return x;
	} else {
		return x; 
	}
}

int minFloat(float x, float y) {
	if (y < x) {
		return y;
	} else if (x < y) {
		return x;
	} else {
		return x; 
	}
}

void whiteLine()
{
	//system Ms at the time of func call
	long long whiteLineEnterTime = GetSysTimeMs();

	SetLCDSolidCircle(70, 180, 30, WHITE);
   
	while (GetSysTimeMs() - whiteLineEnterTime < 1000) {
	//while(getGrayscaleFF() < 2700 || getGrayscaleFL() < 2200 || getGrayscaleL() < 1800 || getGrayscaleR() < 1800) {
		//display();
		calculateCoord();
		
		switch (positionInfo.area) {	
			case AREA_TOP_LEFT: backPosition(-250, 250, 1); break; //top left
			case AREA_TOP_RIGHT: backPosition(250, 250, 1); break; //top right
			case AREA_BOTTOM_LEFT: backPosition(-250, -250, 1); break; //bottom left
			case AREA_BOTTOM_RIGHT: backPosition(250, -250, 1); break; //bottom right
		
		
		
		/*
			case AREA_TOP_LEFT: backPosition(0, 0, 1); break; //top left
			case AREA_TOP_RIGHT: backPosition(0, 0, 1); break; //top right
			case AREA_BOTTOM_LEFT: backPosition(0, 0, 1); break; //bottom left
			case AREA_BOTTOM_RIGHT: backPosition(0, 0, 1); break; //bottom right
			
		*/
		}

		move(dir, 30, 0);
	}
	
	SetLCDSolidCircle(70, 180, 30, GREEN);
	/*
	SetLCDString(250, 170, "Flag:", YELLOW, BLACK);
	SetLCD5Char(210, 190, positionFlag, YELLOW, BLACK);
	if (positionFlag > 3) {
		dir = STOP;
	}
	*/
}



void chaseBall() {
	int minDistance = 20;
	int maxDistance = 1200;
	dir = getEyeAngle();
	int ballValue = getDistanceBall();

	float ratio = 1- (float) (ballValue - minDistance) / (maxDistance - minDistance);
	
	//float ratio = (float)(maxDistance - ballValue) / (maxDistance - minDistance);
	//speed = BASE_CHASEBALL_SPEED + (MAX_CHASEBALL_SPEED - BASE_CHASEBALL_SPEED) * ratio + 10;
	//speed = constrain(speed, BASE_CHASEBALL_SPEED, MAX_CHASEBALL_SPEED);
	
	speed = BASE_CHASEBALL_SPEED + (MAX_CHASEBALL_SPEED - BASE_CHASEBALL_SPEED) * ratio;
	speed = constrain(speed, BASE_CHASEBALL_SPEED, MAX_CHASEBALL_SPEED);
    targetAngle = 0;
	
}




void score(float goalDir) {
    targetAngle = goalDir;
	
    int minD = 20, maxD = 1200;
    int ballVal = getDistanceBall();
    dir = 0;
    float ratio = 1.0f - (float)(ballVal - minD)/(maxD - minD);
    ratio = constrainDouble(ratio, 0.0, 1.0);
    speed = BASE_CHASEBALL_SPEED +
            (int)((MAX_CHASEBALL_SPEED - BASE_CHASEBALL_SPEED)*ratio);
    speed = constrain(speed, BASE_CHASEBALL_SPEED, MAX_CHASEBALL_SPEED);
}


void aroundBall(){

	positionFlag = 0;
	distanceToBall = getDistanceBall();
	ballAngle = getEyeAngle();
	compassRange = BALL_COMPASS_RANGE;

	int minDist = 25;
	int maxDist = 400;
	
	double offsetRatio = (double)(maxDist - distanceToBall)/(maxDist - minDist);
	offsetRatio = constrainDouble(offsetRatio, 0.0, 1.0);
	int offset = 0;
	int offsetMax = 0;
	if (ballAngle <= 80){
		offsetMax = ballAngle;
	} else if (ballAngle >= 280){
		offsetMax = 360 - ballAngle;
	} else {
		offsetMax = 90;
	}
	offset = offsetMax - (offsetMax * offsetRatio);
	
	SetLCD5Char(230, 180, offset, YELLOW, BLACK);
	
	if (ballAngle == 0){
		dir = 0;
	} else if (ballAngle < 180){
		dir = ballAngle + offset;
	} else {
		dir = ballAngle - offset;
	}
	
	aimGoal();
	
	//distanceToBall range: 60~210
        int minDist2 = 25;
        int maxDist2 = 470;
        
        
        double ratio = (double)(maxDist - distanceToBall)/ (maxDist - minDist);
        
	ratio = constrainDouble(ratio, 0.0, 1.0);
	//SetLCD5Char(200, 170, ratio * 100, YELLOW, BLACK);
	int chaseBallSpeed = BASE_CHASEBALL_SPEED + (MAX_CHASEBALL_SPEED - BASE_CHASEBALL_SPEED) * ratio;
        
        speed = chaseBallSpeed;
        applyAirWall();
        
}

void aimGoal() {
    // only do this when it's holding the ball
    if (getLaser() >= 50 && !(ballAngle == 0 || ballAngle == 20 || ballAngle == 340)) {
        return;
    }
    int uL = getUltraL();
    int uR = getUltraR();
    int sum = uL + uR;

    const float MAX_OFFSET = 60.0f;
    //between -1, 1
    float normalizedError = (sum != 0)
        ? (uR - uL) / (float)sum
        : 0.0f;

    float input = normalizedError * MAX_OFFSET;
    float offset = input;
    if (offset >  MAX_OFFSET) offset =  MAX_OFFSET;
    if (offset < -MAX_OFFSET) offset = -MAX_OFFSET;

    dir = wrap360(offset);
    
    /*
	if (ultraLeft > ultraRight) {
		dir = dir - 25;
	} else if (ultraRight > ultraLeft) {
		dir = dir + 25;
	}
	*/

	//dir = dir +  constrain(input, -MAX_OFFSET, MAX_OFFSET);
	//dir = dir % 360;
	

}




struct Vector combineVectors(struct Vector v1, struct Vector v2) {
    double rad1 = v1.angle * M_PI / 180.0;
    double rad2 = v2.angle * M_PI / 180.0;

    double x = v1.speed * cos(rad1) + v2.speed * cos(rad2);
    double y = v1.speed * sin(rad1) + v2.speed * sin(rad2);

    struct Vector result;
    result.speed = (int)sqrt(x * x + y * y);
    result.angle = (int)(atan2(y, x) * 180.0 / M_PI);
    if (result.angle < 0) result.angle += 360;

    return result;
}

	
int main(void)
{
	X3RCU_Init();
	
	SetSysTime();
	

	while(1){
		int eyeAngle = getEyeAngle();
		
		targetAngle = 0;
		display();
		
		calculateCoord();
		
		SetLCDString(220, 170, "Laser:", YELLOW, BLACK);
		SetLCD5Char(220, 190, getLaser(), YELLOW, BLACK);
		


		if(getGrayscaleL() > 2800 || getGrayscaleR() > 2400 || getGrayscaleB() > 3100) {
				whiteLine();
		} else if (getEyeValue() > 60)  {  
			targetAngle = 0;
			aroundBall();
			cachedGoalDir = -1;
			SetLCDString(30, 30, "aroundBall", RED, BLACK);
		}  else {
			targetAngle = 0;
			backPosition(0, 0, 0);
			SetLCDString(30, 30, "no ball", RED, BLACK);
			cachedGoalDir = -1;
			//dir = STOP;
		}
		
		move(dir, speed, 0);
	}
}
