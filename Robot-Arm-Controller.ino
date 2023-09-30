#include <Adafruit_LSM6DS33.h>

int c,turny=1000,rotx=1450,turnz=1500,j=900, daclaw=1450, cclaw=550,IR_ana=0;
float x[6],tiempo,mueves,avg,passed;
Adafruit_LSM6DS33 lsm6ds33;
float AccErrorX=0,AccErrorY=0,AccErrorZ=0;
float AccX=0,AccY=0,AccZ=0, TotAcc=0, Xlp,Ylp,distance;
float gx,gy,degx,degy,sensclaw;
const int Basex=4,Barmz=18,Marmy=5,clw=19;//servo pins
const int pingPin = 12,IR_sesnor=26;// Trigger Pin of Ultrasonic Sensors
const int echoPin = 13; // Echo Pin of Ultrasonic Sensors
const int INTRUPT = 27,over=14;//buttons
volatile int count = 0;

void IRAM_ATTR buttonpressed() {//Interrupt function
    c=0;rotx=1450;turnz=1500;turny=1000;  
    mg995(Barmz,turnz);
    mg995(Basex,rotx);
    mg995(Marmy,turny); 
    //Serial.println(rotx);Serial.println(turny);Serial.println(turnz);Serial.println(c);
}


long PlusUltra(int inpin,int outpin) {//Ultrasonic sensor
   long duration, cm=0,avg=0;

   //while(i<5){
   digitalWrite(inpin, LOW);
   delayMicroseconds(2);
   digitalWrite(inpin, HIGH);
   delayMicroseconds(10);
   digitalWrite(inpin, LOW);
   
   pinMode(outpin, INPUT);
   duration = pulseIn(outpin, HIGH);

   cm = (duration*0.0343)/2;
   if(cm != 0){
    return cm;
   }
   
}
void motor1(int pine,int rotx){//50Hz servo PWM
  digitalWrite(pine, HIGH);   
  delayMicroseconds(rotx);                       
  digitalWrite(pine, LOW);    
  delayMicroseconds(20005-rotx);
 // Serial.print("motor_output:");
  //Serial.print(rotx);
}


void mg995(double jim, int pine){//range limiting functions
  int jimbo = (int)jim;
  int turn1 = (jimbo);

    if(pine==Marmy)//this is the x-axis
  {
  if(turn1>1.8){turny+=28;
  if(turny>1600){turny=1600;}}
  if(turn1<-1.8){turny-=28;
  if(turny<1000){turny=1000;}}
  motor1(Marmy,turny);
  }
    if(pine==Basex)//this is the x-axis
  {
  if(turn1>2.5){
  rotx=rotx+100;
  if(rotx>2100){rotx=2100;}
  
  }
  if(turn1<-2.5){
  
  rotx=rotx-100;
  if(rotx<800){rotx=800;}
  }
  //Serial.print("\nTheActual::");
  //Serial.print(rotx);
  motor1(Basex,rotx);
  }
  if(pine==Barmz)//this is the x-axis
  {
  if(turn1>avg+4&&turn1<900){turnz+=28;
  if(turnz>2100){turnz=2100;}}
  if(turn1<avg-4){turnz-=28;
  if(turnz<1500){turnz=1500;}}
  motor1(Barmz,turnz);
  }
}


void AccError(float x, float y, float z){//sampling averages function
    AccErrorX = AccErrorX + x;
    AccErrorY = AccErrorY + y;
    AccErrorZ = AccErrorZ + z;

}

void setup(void) {
  pinMode(Barmz, OUTPUT);
  pinMode(Marmy, OUTPUT);
  pinMode(Basex, OUTPUT);
  pinMode(clw, OUTPUT);
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(INTRUPT, INPUT_PULLUP);
  pinMode(over, INPUT_PULLUP);
  attachInterrupt(INTRUPT, buttonpressed, FALLING);
  
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DS33 test!");

  if (!lsm6ds33.begin_I2C()) {
    // if (!lsm6ds33.begin_SPI(LSM_CS)) {
    // if (!lsm6ds33.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  }
}

void loop() {
  //delay(1000000);
  IR_ana=analogRead(IR_sesnor);//2300
  int ovride = digitalRead(over);
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  
  //Averages intial accelorometor data to get the drift/noise error 
  while(c<50){
    AccError(accel.acceleration.x,accel.acceleration.y, accel.acceleration.z);
    c++;
  if(c==50){
    AccErrorX = (AccErrorX/50);//-0.03;
    AccErrorY = (AccErrorY/50);//+0.18;
    AccErrorZ = (AccErrorZ/50);//+0.63;
    avg=PlusUltra(pingPin,echoPin);
  }
  }

  //Serial.print(avg);
  //Serial.print("cm\n");
  //sensclaw = PlusUltra(sonar,pong);
  passed = PlusUltra(pingPin,echoPin);
  AccX=accel.acceleration.x-AccErrorX;
  AccY=accel.acceleration.y-AccErrorY;
  //AccZ=accel.acceleration.z-AccErrorZ;
//Passes sensor data to motors
  mg995(passed,Barmz);
  mg995(AccY,Basex);
  mg995(AccX,Marmy); 
 /* Serial.print("\t");
  Serial.print(AccX);
  Serial.print("cm\n");
  Serial.print("\t");
  Serial.print(AccY);
  Serial.print("cm\n");*/
 
  if(IR_ana>2200){motor1(clw,cclaw);}//closes claw if object within 5cm of sensor
  if(ovride==LOW){motor1(clw,daclaw);}//opens claw with button
}
