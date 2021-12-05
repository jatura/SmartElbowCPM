#include <Arduino.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal_SI2C.h>
#include <SPI.h>
#include <L298N.h>

//define pin
#define EN 5
#define IN1 A0
#define IN2 A1
#define encoderpinA 2
#define encoderpinB 3
#define Button_up 6
#define Button_down 7
#define Button_left 8
#define Button_right 9
#define Button_ok 10
#define limitSwitch_L A2
#define limitSwitch_R A3

//define motor speed level
#define speed1 89
#define speed2 100
#define speed3 120

#define interval_updateAngle 1000
#define interval_checkButton 300

#define I2C_ADDR 0x27 
#define BACKLIGHT_PIN 3

//ประกาศตัวแปรของ class จาก lib อื่น
LiquidCrystal_I2C lcd(I2C_ADDR,2,1,0,4,5,6,7);
L298N motor(EN, IN1, IN2);

//added motor init
byte encoderPinALast;
signed long countPulse=0;//the number of the pulses
boolean Direction;//the rotation direction

//set initial value of input
char inputSideOfArm   ='L';
int inputSpeed        = 1; //constant
int inputRound        = 0;
int inputRound0       = 0; //round ตัวเลขตำแหน่ง 0
int inputRound1       = 1; //round ตัวเลขตำแหน่ง 1
int inputFreezingTime = 0; //เวลาค้าง
int inputFreezingTime0= 0;
int inputFreezingTime1= 0;
float inputWeight     = 0;
int inputWeight0      = 0;
int inputWeight1      = 0; //น้ำหนักหลัก 10
int inputWeight2      = 0; //น้ำหนักหลักหน่วย
int inputWeight3      = 0;

int motorSpeed;

//set motor at 0
int command           = 0; //เลือกว่าจะเข้าไปแก้ค่าอะไร
int previousCommand;
int okStatus          = 0; //ดูว่ากด ok รึยัง
int positionEdit      = 0; //เลือกหลักที่จะแก้เลข

//select at motor 2
int pauseSelect = 0; // 0=ทำต่อ 1=เริ่มใหม่

//value from sensor
float angle_avg         =0;
float angle_max         =0;
float angle_now         =0;
//int pulse;

//counting value
int leftRound; //จำนวนรอบที่เหลือ
int leftFreezingTime; //เวลาค้างที่เหลือ
int pauseFreezingTime;

//time for millis
unsigned long previousMillisFreeze;
unsigned long presentMillisFreeze;
unsigned long previousMillisUpdateAngle;
unsigned long presentMillisUpdateAngle;

//time for angle calibration
unsigned long previousMillisAngle = 0;
unsigned long presentMillisAngle;
float angle_calibrate=0;

//time for angle calibration
unsigned long previous = 0;
unsigned long present;

bool backFromPause      = 0;

//system control 
int motorStatus       = 0; //0=go to input function, 1=working, 2=pause
int motorRotation     = 0; //0=งอ, 1=หยุด, 2=เหยียด
int limitSwitchStatus = 0; //0=ยังไม่โดนกด, 1=โดนกดแล้ว(อยู่ที่ 0)

//pause while rerun
bool pauseRerunStatus = 0;

int runLoop           = 0;
bool lcdClearState    = 0;
bool lcdClearState1   = 0;
bool determineValue    = 0;

//checkButtonPress for the command selection and OK
bool checkRightButton = 0;
bool checkLeftButton  = 0;
unsigned long previousMillisButton;
unsigned long presentMillisButton;

//checkButtonPress inside the command (ใน switch case ตอนเปลี่ยนค่า)
bool checkPressButton1 = 0;
unsigned long previousMillisPressButton1;
unsigned long presentMillisPressButton1;

bool checkPressButton2 = 0;
unsigned long previousMillisPressButton2;
unsigned long presentMillisPressButton2;

bool checkPressButton3 = 0;
unsigned long previousMillisPressButton3;
unsigned long presentMillisPressButton3;

bool checkPressButton4 = 0;
unsigned long previousMillisPressButton4;
unsigned long presentMillisPressButton4;

//checkOKButton after switch case
bool checkOKButton = 0;
unsigned long previousMillisOKButton;
unsigned long presentMillisOKButton;

//moving avg
bool statusMA =0;
int countMA   =0;
int countMA1=0;
float angle_old=-2.35571719077467;
unsigned long previousMillisDiff;
unsigned long presentMillisDiff=0;
float diffAngle;

void showLCD_withStart();
void showLCD_withStop();
void showLCD_angleAvgMax();
void showLCD_withPause();
void showLCD_DMYT();
void checkAngle();
void wheelSpeed();

void setup() {
    Serial.begin(57600);

    //set default lcd
    lcd.begin (16,2); 
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);
    lcd.home ();
    
    //set pinmode Button
    pinMode(Button_up,    INPUT_PULLUP);
    pinMode(Button_down,  INPUT_PULLUP);
    pinMode(Button_left,  INPUT_PULLUP);
    pinMode(Button_right, INPUT_PULLUP);
    pinMode(Button_ok,    INPUT_PULLUP);
    pinMode(limitSwitch_L,INPUT_PULLUP);
    pinMode(limitSwitch_R,INPUT_PULLUP);

    pinMode(encoderpinA,  INPUT);
    pinMode(encoderpinB,  INPUT);

    showLCD_withStart();

    attachInterrupt(digitalPinToInterrupt(encoderpinB), wheelSpeed, CHANGE);
}

void loop() {
    checkMotorStatus();
}

void checkMotorStatus(){
    //motor ยังไม่ทำงาน ไป set ค่าก่อน
    if (motorStatus==0){
        motorState0();
    }

    //motor กำลังทำงาน
    if (motorStatus==1){    
        motorState1();
    }

    //motor pause อยู่
    if (motorStatus==2){
        motorState2();
    }
}

///////////////////////////////////////////////////////// motorState0 set up////////////////////////////////////////////////////////////////////////////
void motorState0(){
    if(lcdClearState==0){
        lcd.clear();
        showLCD_withStart();
        lcdClearState=1;
    }
    angle_avg = 0;
    angle_max = 0;
    
    if(okStatus==0){
        if (command==0){
            lcd.setCursor(0,0);
            lcd.blink();
        }
        else if (command==1){
            lcd.setCursor(4,0);
            lcd.blink();
            lcd.setCursor(5,0);
            lcd.blink();
        }
        else if (command==2){
            lcd.setCursor(8,0);
            lcd.blink();
            lcd.setCursor(9,0);
            lcd.blink();
        }
        else if (command==3){
            lcd.setCursor(13,0);
            lcd.blink();
        }   
        else if (command==4){
            lcd.setCursor(0,1);
            lcd.blink();
            lcd.setCursor(1,1);
            lcd.blink();
        }     
        else if (command==5){
            lcd.setCursor(10,1);
            lcd.blink();            
        } 
                
        //digital read low = 0
            if(digitalRead(Button_up)==0){
                if(command==4 || command==5){
                    command = previousCommand;
                }
            }
            if(digitalRead(Button_down)==0){
                if(command!=4 && command!=5){
                    previousCommand = command;
                }
                command=4;
            }
            if(digitalRead(Button_left)==0){
                if (checkLeftButton == 0){
                    previousMillisButton=millis();
                    checkLeftButton = 1;
                }   
            }
            if(checkLeftButton == 1){
                presentMillisButton=millis();
                if(presentMillisButton-previousMillisButton>=interval_checkButton){
                  if(command!=0 && command!=4){
                    command--;
                  }
                    checkLeftButton = 0;                
                }                           
            }            
            if(digitalRead(Button_right)==0){
                if (checkRightButton == 0){
                    previousMillisButton=millis();
                    checkRightButton = 1;
                }   
            }
            if(checkRightButton == 1){
                presentMillisButton=millis();
                if(presentMillisButton-previousMillisButton>=interval_checkButton){
                    if(command!=3 && command!=5){
                        command++;
                    }
                    checkRightButton = 0;                
                }                           
            }    
            if(digitalRead(Button_ok)==0){
                if (okStatus==0){
                    previousMillisButton=millis();
                    positionEdit=0;
                    okStatus=1;
                }     
            }
    }
          
    if(okStatus==1){
        presentMillisButton=millis();
        if(presentMillisButton-previousMillisButton>=interval_checkButton){
            switch (command){
                //input side
                case 0:
                    lcd.setCursor(2,0);
                    lcd.blink(); 
                    if(digitalRead(Button_up)==0 || digitalRead(Button_down)==0 || digitalRead(Button_left)==0 || digitalRead(Button_right)==0){
                        if(checkPressButton1==0){
                            previousMillisPressButton1=millis();
                            checkPressButton1=1;
                        }
                    }
                    if(checkPressButton1==1){
                        presentMillisPressButton1=millis();
                        if (presentMillisPressButton1-previousMillisPressButton1>=interval_checkButton){
                            if(inputSideOfArm=='L'){
                                inputSideOfArm='R';
                            }
                            else if(inputSideOfArm=='R'){
                                inputSideOfArm='L';
                            }
                            checkPressButton1=0;           
                        }
                        showLCD_withStart();
                    }                                                                 
                    break;
                
                //input speed
                case 1:
                    lcd.setCursor(6,0);
                    lcd.blink();
                    if(digitalRead(Button_up)==0 || digitalRead(Button_right)==0){
                        if(checkPressButton1==0){
                            previousMillisPressButton1=millis();
                            checkPressButton1=1;
                        }
                    }
                    
                    if(checkPressButton1==1){
                        presentMillisPressButton1=millis();
                        if (presentMillisPressButton1-previousMillisPressButton1>=interval_checkButton){                        
                            if(inputSpeed==3){
                                inputSpeed=1;
                            }
                            else{
                                inputSpeed++;
                            }
                            checkPressButton1=0;
                        }
                        showLCD_withStart();
                    }

                    if(digitalRead(Button_down)==0 || digitalRead(Button_left)==0){
                        if(checkPressButton2==0){
                            previousMillisPressButton2=millis();
                            checkPressButton2=1;
                        }
                    }
                    if(checkPressButton2==1){
                        presentMillisPressButton2=millis();
                        if (presentMillisPressButton2-previousMillisPressButton2>=interval_checkButton){                        
                            if(inputSpeed==1){
                                inputSpeed=3;
                            }
                            else{
                                inputSpeed--;
                            }
                            checkPressButton2=0;
                        }
                        showLCD_withStart();
                    }
                    break;
                
                //input round
                case 2: 
                    if (positionEdit==0){
                        lcd.setCursor(10,0);
                        lcd.blink();
                    }
                    else if (positionEdit==1){
                        lcd.setCursor(11,0);
                        lcd.blink();
                    }
       
                    if(digitalRead(Button_left)==0 || digitalRead(Button_right)==0){
                        if(checkPressButton1==0){
                            previousMillisPressButton1=millis();
                            checkPressButton1=1;
                        }
                    }
                    if(checkPressButton1==1){
                        presentMillisPressButton1=millis();
                        if (presentMillisPressButton1-previousMillisPressButton1>=interval_checkButton){                        
                            if(positionEdit==0){
                                positionEdit=1;
                            }
                            else if(positionEdit==1){
                                positionEdit=0;
                            }
                            checkPressButton1=0;
                        }
                        showLCD_withStart();
                    }
                    
                    if(digitalRead(Button_up)==0){
                        if(checkPressButton2==0){
                            previousMillisPressButton2=millis();
                            checkPressButton2=1;
                        }
                    }
                    if(checkPressButton2==1){
                        presentMillisPressButton2=millis();
                        if (presentMillisPressButton2-previousMillisPressButton2>=interval_checkButton){                        
                            if(positionEdit==0){
                                if(inputRound0==9){
                                    inputRound0=0;
                                }
                                else{
                                    inputRound0++;
                                }
                            }
                            if(positionEdit==1){
                                if(inputRound1==9){
                                    inputRound1=0;
                                }
                                else{
                                    inputRound1++;
                                }
                            }                            
                            checkPressButton2=0;
                        }
                        showLCD_withStart();
                    }                    
                    
                    if(digitalRead(Button_down)==0){
                        if(checkPressButton3==0){
                            previousMillisPressButton3=millis();
                            checkPressButton3=1;
                        }
                    }
                    if(checkPressButton3==1){
                        presentMillisPressButton3=millis();
                        if (presentMillisPressButton3-previousMillisPressButton3>=interval_checkButton){                        
                            if(positionEdit==0){
                                if(inputRound0==0){
                                    inputRound0=9;
                                }
                                else{
                                    inputRound0--;
                                }
                            }
                            if(positionEdit==1){
                                if(inputRound1==0){
                                    inputRound1=9;
                                }
                                else{
                                    inputRound1--;
                                }
                            }                      
                            checkPressButton3=0;
                        }
                        showLCD_withStart();
                    }
                    break;
                
                //input time freeze
                case 3:
                    if (positionEdit==0){
                        lcd.setCursor(14,0);
                        lcd.blink();
                    }
                    else if (positionEdit==1){
                        lcd.setCursor(15,0);
                        lcd.blink();
                    }
                    if(digitalRead(Button_left)==0 || digitalRead(Button_right)==0){
                        if(checkPressButton1==0){
                            previousMillisPressButton1=millis();
                            checkPressButton1=1;
                        }
                    }
                    if(checkPressButton1==1){
                        presentMillisPressButton1=millis();
                        if (presentMillisPressButton1-previousMillisPressButton1>=interval_checkButton){                        
                            if(positionEdit==0){
                                positionEdit=1;
                            }
                            else if(positionEdit==1){
                                positionEdit=0;
                            }
                            checkPressButton1=0;
                        }
                        showLCD_withStart();
                    }

                    if(digitalRead(Button_up)==0){
                        if(checkPressButton2==0){
                            previousMillisPressButton2=millis();
                            checkPressButton2=1;
                        }
                    }
                    if(checkPressButton2==1){
                        presentMillisPressButton2=millis();
                        if (presentMillisPressButton2-previousMillisPressButton2>=interval_checkButton){                        
                            if(positionEdit==0){
                                if(inputFreezingTime0==9){
                                    inputFreezingTime0=0;
                                }
                                else{
                                    inputFreezingTime0++;
                                }
                            }
                            if(positionEdit==1){
                                if(inputFreezingTime1==9){
                                    inputFreezingTime1=0;
                                }
                                else{
                                    inputFreezingTime1++;
                                }
                            }                              
                            checkPressButton2=0;
                        }
                        showLCD_withStart();
                    }

                    if(digitalRead(Button_down)==0){
                        if(checkPressButton3==0){
                            previousMillisPressButton3=millis();
                            checkPressButton3=1;
                        }
                    }
                    if(checkPressButton3==1){
                        presentMillisPressButton3=millis();
                        if (presentMillisPressButton3-previousMillisPressButton3>=interval_checkButton){                        
                            if(positionEdit==0){
                                if(inputFreezingTime0==0){
                                    inputFreezingTime0=9;
                                }
                                else{
                                    inputFreezingTime0--;
                                }
                            }
                            if(positionEdit==1){
                                if(inputFreezingTime1==0){
                                    inputFreezingTime1=9;
                                }
                                else{
                                    inputFreezingTime1--;
                                }
                            }                      
                            checkPressButton3=0;
                        }
                        showLCD_withStart();
                    }            
                    break;

                //input weight
                case 4:
                    if (positionEdit==0){
                        lcd.setCursor(2,1);
                        lcd.blink();
                    }
                    else if (positionEdit==1){
                        lcd.setCursor(3,1);
                        lcd.blink();
                    }
                    else if (positionEdit==2){
                        lcd.setCursor(4,1);
                        lcd.blink();
                    }
                    else if (positionEdit==3){
                        lcd.setCursor(6,1);
                        lcd.blink();
                    }
                    
                    if(digitalRead(Button_left)==0){
                        if(checkPressButton1==0){
                            previousMillisPressButton1=millis();
                            checkPressButton1=1;
                        }
                    }
                    if(checkPressButton1==1){
                        presentMillisPressButton1=millis();
                        if (presentMillisPressButton1-previousMillisPressButton1>=interval_checkButton){                        
                            if(positionEdit>0){
                                positionEdit--;
                            }
                            else if(positionEdit==0){
                                positionEdit=3;
                            }
                            checkPressButton1=0;
                        }
                        showLCD_withStart();
                    }

                    if(digitalRead(Button_right)==0){
                        if(checkPressButton4==0){
                            previousMillisPressButton4=millis();
                            checkPressButton4=1;
                        }
                    }
                    if(checkPressButton4==1){
                        presentMillisPressButton4=millis();
                        if (presentMillisPressButton4-previousMillisPressButton4>=interval_checkButton){                        
                            if(positionEdit<3){
                                positionEdit++;
                            }
                            else if(positionEdit==3){
                                positionEdit=0;
                            }
                            checkPressButton4=0;
                        }
                        showLCD_withStart();
                    }

                    if(digitalRead(Button_up)==0){
                        if(checkPressButton2==0){
                            previousMillisPressButton2=millis();
                            checkPressButton2=1;
                        }
                    }
                    if(checkPressButton2==1){
                        presentMillisPressButton2=millis();
                        if (presentMillisPressButton2-previousMillisPressButton2>=interval_checkButton){                        
                            if(positionEdit==0){
                                if(inputWeight0==9){
                                    inputWeight0=0;
                                }
                                else{
                                    inputWeight0++;
                                }
                            }
                            if(positionEdit==1){
                                if(inputWeight1==9){
                                    inputWeight1=0;
                                }
                                else{
                                    inputWeight1++;
                                }
                            }
                            if(positionEdit==2){
                                if(inputWeight2==9){
                                    inputWeight2=0;
                                }
                                else{
                                    inputWeight2++;
                                }
                            }
                            if(positionEdit==3){
                                if(inputWeight3==9){
                                    inputWeight3=0;
                                }
                                else{
                                    inputWeight3++;
                                }
                            }                              
                            checkPressButton2=0;
                        }
                        showLCD_withStart();
                    }

                    if(digitalRead(Button_down)==0){
                        if(checkPressButton3==0){
                            previousMillisPressButton3=millis();
                            checkPressButton3=1;
                        }
                    }
                    if(checkPressButton3==1){
                        presentMillisPressButton3=millis();
                        if (presentMillisPressButton3-previousMillisPressButton3>=interval_checkButton){                        
                            if(positionEdit==0){
                                if(inputWeight0==0){
                                    inputWeight0=9;
                                }
                                else{
                                    inputWeight0--;
                                }
                            }
                            if(positionEdit==1){
                                if(inputWeight1==0){
                                    inputWeight1=9;
                                }
                                else{
                                    inputWeight1--;
                                }
                            }
                            if(positionEdit==2){
                                if(inputWeight2==0){
                                    inputWeight2=9;
                                }
                                else{
                                    inputWeight2--;
                                }
                            }
                            if(positionEdit==3){
                                if(inputWeight3==0){
                                    inputWeight3=9;
                                }
                                else{
                                    inputWeight3--;
                                }
                            }                      
                            checkPressButton3=0;
                        }
                        showLCD_withStart();
                    }            
                    break;
                    
                //press start
                case 5:
                /////////////////////////////////////////////////////////////////////////////////////////
                    if (inputSideOfArm=='L' && digitalRead(limitSwitch_L)!=0){
                        motor.backward();
                        motor.setSpeed(200);
            if(limitSwitchStatus==0){
              lcd.setCursor(0,0);
              lcd.print("    WAITING     ");
              lcd.setCursor(0,1);
              lcd.print("                ");
              limitSwitchStatus=1;
              determineValue    = 1;
            }
                    }
                    else if (inputSideOfArm=='R' && digitalRead(limitSwitch_R)!=0){
                        motor.forward();
                        motor.setSpeed(200);
                        if(limitSwitchStatus==0){
              lcd.setCursor(0,0);
              lcd.print("    WAITING     ");
              lcd.setCursor(0,1);
              lcd.print("                ");
              limitSwitchStatus=1;
              determineValue    = 1;
            }
                    }             
                    if (inputSideOfArm=='L' && digitalRead(limitSwitch_L)==0){
                        if (checkPressButton1==0){
                            previousMillisPressButton1=millis();
                            checkPressButton1=1;           
                        }
                    }
                    else if (inputSideOfArm=='R' && digitalRead(limitSwitch_R)==0){
                        if (checkPressButton1==0){
                            previousMillisPressButton1=millis();
                            checkPressButton1=1;
                        }
                    }
                    if (checkPressButton1==1){
                        presentMillisPressButton1=millis();
                        if (presentMillisPressButton1-previousMillisPressButton1>=interval_checkButton){
                            motor.stop();
              if(limitSwitchStatus==1){
                lcd.setCursor(0,0);
                lcd.print("  LAY YOUR ARM  ");
                lcd.setCursor(0,1);
                lcd.print("       OK       ");
                lcd.setCursor(7,1);
                lcd.blink();
                lcd.setCursor(8,1);
                lcd.blink();
                if(digitalRead(Button_ok)==0){
                  if (checkOKButton==0){
                    previousMillisOKButton=millis();
                    checkOKButton=1;
                  }
                }
                if (checkOKButton==1){
                  presentMillisOKButton=millis();
                  if (presentMillisOKButton-previousMillisOKButton>=interval_checkButton){
                    determineValue    = 0;
                    checkOKButton=0;
                  }
                }
              }
              if(determineValue== 0){
                  countPulse=0;
                  previousMillisAngle = millis();
                  previous=millis();
        
                        inputRound=(10*inputRound0)+inputRound1;
                        inputFreezingTime=((10*inputFreezingTime0)+inputFreezingTime1)*1000;
                        inputWeight=(100*inputWeight0)+(10*inputWeight1)+inputWeight2+(0.1*inputWeight3);
                        switch (inputSpeed){
                            case 1: motorSpeed = speed1; break;
                            case 2: motorSpeed = speed2; break;
                            case 3: motorSpeed = speed3; break;
                        }
                        
                        leftRound=inputRound;
                        motorRotation=0;
                        motorStatus=1;
                        okStatus=0;
                        lcdClearState=0;
                        lcd.clear();
                        showLCD_withStop();
                        checkPressButton1=0;
                        presentMillisDiff=millis(); 
                    }
                }
            }
            ///////////////////////////////////////////////////////////////////////////////////////////                    
            }

            //check ok Status แต่command=5(start) ไม่ต้องcheck
            if(command>=0 && command<5){
                if(digitalRead(Button_ok)==0){
                    if(checkOKButton==0){
                        previousMillisOKButton=millis();
                        checkOKButton=1;
                    }
                }
                if (checkOKButton==1){
                    presentMillisOKButton=millis();
                    if (presentMillisOKButton-previousMillisOKButton>=interval_checkButton){
                        okStatus=0;
                        checkOKButton=0;
                    }
                }   
            }
        }
    }
}

///////////////////////////////////////////////////////// motorState1 working////////////////////////////////////////////////////////////////////////////
void motorState1(){
    
    runLoop=0;
    if(lcdClearState==0){
        lcd.setCursor(11,1);
        lcd.blink();
    }
    //งอแขน///////////////////////////////////////////////////////////////////////////////////
        if (motorRotation==0){
            //สั่งให้มอเตอร์หมุน
            if(inputSideOfArm=='L'){
                motor.forward();
                motor.setSpeed(motorSpeed);
            }
            else if(inputSideOfArm=='R'){
                motor.backward();
                motor.setSpeed(motorSpeed);
            }
			if(inputSpeed==1) {angle_calibrate=0.00315558837681134-0.0000429684339640191*inputWeight;}
			else if(inputSpeed==2) {angle_calibrate=0.00392167101100058-0.0000369176009093263*inputWeight;}
			else if (inputSpeed==3) {angle_calibrate=0.00489126226990784-0.0000367979031118582*inputWeight;}
      present=millis();
            //read angle and compare
			previousMillisDiff=presentMillisDiff;
			angle_old=angle_now;
			presentMillisDiff=millis();
            angle_now=(abs(countPulse))*0.00322104728347294-2.35571719077467;
                        presentMillisAngle=millis();   
        if(present-previous>1000){
            previous=millis();         
            }
			//cal diff
			diffAngle=(angle_now-angle_old)/(presentMillisDiff-previousMillisDiff);
     
      Serial.print(angle_calibrate*1000);
      Serial.print(" ");
      Serial.println(diffAngle*1000);
      
			if(diffAngle<angle_calibrate){
				countMA1++;
			}    
			else{
				countMA1=0;
			}
                        
          if(countMA1>3){
            motor.stop();
                        angle_now = (abs(countPulse))*0.00322104728347294-2.35571719077467;
                        showLCD_withStop();
                        checkAngle();
                        previousMillisFreeze = millis();
                        motorRotation=1; //เปลี่ยนสถานะเป็นมอเตอร์หยุด
                        countMA1=0;
                        countMA=0;
          }
                    else if (angle_now>150){ //เอาค่าจาก pulse มาดู ถ้าเกิน 150 องศาก็ให้พอ
                        motor.stop();
                        angle_now = (abs(countPulse))*0.00322104728347294-2.35571719077467;
                        showLCD_withStop();
                        checkAngle();
                        previousMillisFreeze = millis();
                        motorRotation=1; //เปลี่ยนสถานะเป็นมอเตอร์หยุด
                        countMA1=0;
                        countMA=0;     
                    }
          
            runLoop=1;        
        }
        
        //freeze motor////////////////////////////////////////////////////////////////////////////////
        else if (motorRotation==1){
            presentMillisFreeze = millis(); 
            if (backFromPause==0){
                //ค้างครบเวลาแล้ว
                if (presentMillisFreeze - previousMillisFreeze >= inputFreezingTime){
                    motorRotation=2;
                    runLoop=1;
                }
                //ยังค้างไม่ครบเวลา
                else {
                    leftFreezingTime = inputFreezingTime - (presentMillisFreeze - previousMillisFreeze);
                    showLCD_withStop();
                    runLoop=2;
                }
            }
            else if (backFromPause==1){
                //ค้างครบเวลาแล้ว
                if (presentMillisFreeze - previousMillisFreeze >= pauseFreezingTime){
                    motorRotation=2;
                    runLoop=1;
                    backFromPause=0;
                }
                //ยังค้างไม่ครบเวลา
                else {
                    leftFreezingTime = pauseFreezingTime - (presentMillisFreeze - previousMillisFreeze);
                    showLCD_withStop();
                    runLoop=2;
                }
            }
        }

        //เหยียดแขน/////////////////////////////////////////////////////////////////////////////////////
        else if (motorRotation==2){
            //สั่งให้มอเตอร์หมุนกลับ
            if(inputSideOfArm=='L'){
                motor.backward();
                motor.setSpeed(motorSpeed);
            }
            else if(inputSideOfArm=='R'){
                motor.forward();
                motor.setSpeed(motorSpeed);
            }
            //check if limit switch was pressed
            if (inputSideOfArm=='L' && digitalRead(limitSwitch_L)==0){
                if (checkPressButton1==0){
                    previousMillisPressButton1=millis();
                    checkPressButton1=1;
            
                }
            }
            if (inputSideOfArm=='R' && digitalRead(limitSwitch_R)==0){
                if (checkPressButton1==0){
                    previousMillisPressButton1=millis();
                    checkPressButton1=1;
                }
            }
            if (checkPressButton1==1){
                presentMillisPressButton1=millis();
                if (presentMillisPressButton1-previousMillisPressButton1>=interval_checkButton){
                    motor.stop();
                    countPulse=0;
                    leftRound--;
                    if (leftRound<=0){
                        motorRotation=3;
                        lcdClearState=0;
                    }
                    else{
                        showLCD_withStop();
                        motorRotation=0;
                        runLoop=2;
                    }
                    checkPressButton1=0;
                }
            }    
            else runLoop=1;
        }

        //show max avg angle
        else if(motorRotation==3){
            if(lcdClearState==0){
                angle_avg = angle_avg/inputRound;
                lcd.clear();
                showLCD_angleAvgMax();
                lcdClearState=1;
             }
             //send to bluetooth
             if (digitalRead(Button_ok)==0){
                if (checkPressButton2==0){
                    previousMillisPressButton2=millis();
                    checkPressButton2=1;
                }
             }
             if (checkPressButton2==1){
                presentMillisPressButton2=millis();
                if (presentMillisPressButton2-previousMillisPressButton2>=interval_checkButton){
                    //countPulse=0;
                    motorStatus=0;
                    lcdClearState=0;
                    motorRotation=0;
                    checkPressButton2=0;
                }
              }
              runLoop=0;
        }

        switch(runLoop){
            case 0: break;
            case 1:
                //check angle every 5 sec
                presentMillisUpdateAngle = millis();
                angle_now = (abs(countPulse))*0.00322104728347294-2.35571719077467;
                showLCD_withStop();
    
            case 2:
                if (digitalRead(Button_ok)==0){
                    if (checkOKButton==0){
                        previousMillisOKButton=millis();
                        checkOKButton=1;
                    }
                }
                if (checkOKButton==1){
                    presentMillisOKButton=millis();
                    if (presentMillisOKButton-previousMillisOKButton>=interval_checkButton){
                        motor.stop();
                        motorStatus=2;
                        lcdClearState=0;
                        checkOKButton=0;
                        pauseSelect=0;
                    }
                }
                break;                
        }
}
///////////////////////////////////////////////////////// motorState2 pause////////////////////////////////////////////////////////////////////////////
void motorState2(){
    pauseFreezingTime = leftFreezingTime;
    if(lcdClearState==0){
        lcd.clear();
        showLCD_withPause();
        lcdClearState=1;
    }
    if (pauseSelect==0){
            lcd.setCursor(1,1);
            lcd.blink();
        }
    else if (pauseSelect==1){
            lcd.setCursor(9,1);
            lcd.blink();
    }
    if(digitalRead(Button_up)==0 || digitalRead(Button_down)==0 || digitalRead(Button_left)==0 || digitalRead(Button_right)==0){
        if(checkPressButton1==0){
            previousMillisPressButton1=millis();
            checkPressButton1=1;
        }
    }
    if(checkPressButton1==1){
        presentMillisPressButton1=millis();
        if (presentMillisPressButton1-previousMillisPressButton1>=interval_checkButton){
            if(pauseSelect==0){
                pauseSelect=1;
                lcdClearState1=0;
            }
            else if(pauseSelect==1){
                pauseSelect=0;
            }
            checkPressButton1=0;
        }
    }
        
    if(digitalRead(Button_ok)==0){
        if(checkOKButton==0){
            previousMillisOKButton=millis();
            checkOKButton=1;
        }
    }
    if (checkOKButton==1){
        presentMillisOKButton=millis();
        if (presentMillisOKButton-previousMillisOKButton>=interval_checkButton){
            if(pauseSelect==0){
                motorStatus=1;
                lcd.clear();
                checkOKButton=0;
                backFromPause=1;
                previousMillisFreeze=millis();
            }
            if(pauseSelect==1){
                if(lcdClearState1==0){
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("    WAITING     ");
                    lcd.setCursor(0,1);
                    lcd.print("     >PAUSE     ");
                    lcdClearState1=1;
                }
                lcd.setCursor(5,1);
                lcd.blink();
        
        if (pauseRerunStatus==0){                   
          if(inputSideOfArm=='L'){
            motor.backward();
            motor.setSpeed(motorSpeed);
          }
          else if(inputSideOfArm=='R'){
            motor.forward();
            motor.setSpeed(motorSpeed);
          }
          
          if (digitalRead(Button_ok)==0){
            if(checkPressButton3==0){
              previousMillisPressButton3=millis();
              checkPressButton3=1;
            }
          }
        
          if(checkPressButton3==1){
            presentMillisPressButton3=millis();
            if (presentMillisPressButton3-previousMillisPressButton3>=interval_checkButton){
              motor.stop();
              pauseRerunStatus=1;
              lcd.setCursor(0,0);
              lcd.print("   >CONTINUE    ");
              lcd.setCursor(0,1);
              lcd.print("                ");
              checkPressButton3=0;
            }
          }
        }
        
        if (pauseRerunStatus==1){
          lcd.setCursor(3,0);
                    lcd.blink();
          if (digitalRead(Button_ok)==0){
            if(checkPressButton4==0){
              previousMillisPressButton4=millis();
              checkPressButton4=1;
            }           
          }
          if(checkPressButton4==1){
            presentMillisPressButton4=millis();
            if (presentMillisPressButton4-previousMillisPressButton4>=interval_checkButton){
              pauseRerunStatus=0;
              checkPressButton4=0;
            }
          }         
        }                   

                if(inputSideOfArm=='L' && digitalRead(limitSwitch_L)==0){
                    if(checkPressButton2==0){
                        previousMillisPressButton2=millis();
                        checkPressButton2=1;
                    }
                }
                if(inputSideOfArm=='R' && digitalRead(limitSwitch_R)==0){
                    if(checkPressButton2==0){
                        previousMillisPressButton2=millis();
                        checkPressButton2=1;
                    }
                }
                if(checkPressButton2==1){
                    presentMillisPressButton2=millis();
                    if (presentMillisPressButton2-previousMillisPressButton2>=interval_checkButton){
                        motor.stop();
                        countPulse=0;
                        pauseSelect==0;
                        motorStatus=0;
                        lcdClearState=0;
                        checkPressButton2=0;
                        checkOKButton=0;
                    }
                }               
            }
        }
    }  
}
void showLCD_withStart(){
    lcd.setCursor(0,0);
    lcd.print("S:");
    lcd.setCursor(2,0);
    lcd.print(inputSideOfArm);

    lcd.setCursor(4,0);
    lcd.print("SP");
    lcd.setCursor(6,0);
    lcd.print(inputSpeed);

    lcd.setCursor(8,0);
    lcd.print("RD");
    lcd.setCursor(10,0);
    lcd.print(inputRound0);
    lcd.setCursor(11,0);
    lcd.print(inputRound1);

    lcd.setCursor(13,0);
    lcd.print("T");
    lcd.setCursor(14,0);
    lcd.print(inputFreezingTime0);
    lcd.setCursor(15,0);
    lcd.print(inputFreezingTime1);

    lcd.setCursor(0,1);
    lcd.print("WT");
    lcd.setCursor(2,1);
    lcd.print(inputWeight0);
    lcd.setCursor(3,1);
    lcd.print(inputWeight1);
    lcd.setCursor(4,1);
    lcd.print(inputWeight2);
    lcd.setCursor(5,1);
    lcd.print(".");
    lcd.setCursor(6,1);
    lcd.print(inputWeight3);

    lcd.setCursor(10,1);
    lcd.print(">START");
}

void showLCD_withStop(){
    lcd.setCursor(0,0);
    lcd.print("S:");
    lcd.setCursor(2,0);
    lcd.print(inputSideOfArm);

    lcd.setCursor(4,0);
    lcd.print("SP");
    lcd.setCursor(6,0);
    lcd.print(inputSpeed);

    lcd.setCursor(8,0);
    lcd.print("RD");
    lcd.setCursor(10,0);
    lcd.print(leftRound/10);
    lcd.setCursor(11,0);
    lcd.print(leftRound%10);
    
    lcd.setCursor(13,0);
    lcd.print("T");
    if(motorRotation==1){
        lcd.setCursor(14,0);
        lcd.print((leftFreezingTime/1000)/10);
        lcd.setCursor(15,0);
        lcd.print((leftFreezingTime/1000)%10);
    }
    else{
        lcd.setCursor(14,0);
        lcd.print(inputFreezingTime0);
        lcd.setCursor(15,0);
        lcd.print(inputFreezingTime1);
    }

    lcd.setCursor(0,1);
    lcd.print("ANGLE");
    lcd.setCursor(5,1);
    lcd.print(angle_now);
    
    lcd.setCursor(11,1);
    lcd.print(">STOP");
}

void showLCD_angleAvgMax(){
    lcd.setCursor(0,0);
    lcd.print("AVG");
    lcd.setCursor(4,0);
    lcd.print("ANGLE");
    lcd.setCursor(10,0);
    lcd.print(angle_avg);

    lcd.setCursor(0,1);
    lcd.print("MAX");
    lcd.setCursor(4,1);
    lcd.print("ANGLE");
    lcd.setCursor(10,1);
    lcd.print(angle_max);
}

void showLCD_withPause(){
    lcd.setCursor(6,0);
    lcd.print("PAUSE");
    lcd.setCursor(1,1);
    lcd.print(">RESUME");
    lcd.setCursor(9,1);
    lcd.print(">RERUN");
}

//func check angle max and avg
void checkAngle(){
    angle_avg = angle_avg + angle_now;
    if(angle_now>angle_max){
        angle_max=angle_now;
    }
}

//func มากับ code encoder
void wheelSpeed(){
    if( digitalRead(encoderpinB) == 0 ) {
        if ( digitalRead(encoderpinA) == 0 ) {
            // A fell, B is low
            countPulse--; // moving reverse
        } 
        else {
            // A rose, B is low
            countPulse++; // moving forward
        }
    }
}
