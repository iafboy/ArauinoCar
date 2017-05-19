#define DHT11_PIN 7

#include <Wire.h>
#include <HMC5883L.h>
#include <string.h>
#include <SoftwareSerial.h>

HMC5883L compass;

int pinLB=15;     // 定義15腳位 左後
int pinLF=14;     // 定義14腳位 左前

int pinRB=16;    // 定義16腳位 右前
int pinRF=17;    // 定義17腳位 右後

int MotorRPWM=3;
int MotorLPWM=5;

int inputPin = 9;  // 定義超音波信號接收腳位
int outputPin =8;  // 定義超音波信號發射腳位

int Fspeedd = 0;      // 前速
int Rspeedd = 0;      // 右速
int Lspeedd = 0;      // 左速
int directionn = 0;   // 前=8 後=2 左=4 右=6
int delay_time = 250; // 伺服馬達轉向後的穩定時間

int Fgo = 8;         // 前進
int Rgo = 6;         // 右轉
int Lgo = 4;         // 左轉
int Bgo = 2;         // 倒車
int stop=1;

float Fdistance=999.0;

int minstop=10;
int minturn=25;
int baseLimitPWM=100;
int LLimitPWM=400;
int RLimitPWM=400;
int LLimitPWM_=LLimitPWM;
int RLimitPWM_=RLimitPWM;

int history_ad_time=0;  //记录上次前进时间

float headingDegrees=0.0;

bool debug=true;
char debugmsg[127];
//18,13,12 ,0,1 空余脚位
//蓝牙通讯接口
SoftwareSerial BTSerial(4, 6);//RX,TX for BLE

void cleanParam(){
  Fspeedd = 0;
  Rspeedd = 0;
  Lspeedd = 0;
  directionn = 0;
}

// byte read_dht11_dat()
// {
// 	byte i = 0;
// 	byte result=0;
// 	for(i=0; i< 8; i++){
// 	     while(!(PINC & _BV(DHT11_PIN)));  // wait for 50us
// 	     delayMicroseconds(30);
// 	     if(PINC & _BV(DHT11_PIN))
// 	     result |=(1<<(7-i));
//              while((PINC & _BV(DHT11_PIN)));  // wait '1' finish
// 	}
// 	return result;
// }
void advance(float t_angle,int runtime){
  char buf[10];
  memset(debugmsg,0,127);
  //getEnvInfo();
  ask_pin_F();
  //判断前方距离如果小于10cm则停止
  if(Fspeedd < minstop){
    stopp(1);
    strcat(debugmsg,"stop");
    return;
  }

  checkDirection();
  //检查前进角度，并对步进电机进行调速控制转角
  if(round(headingDegrees)>=270&&round(t_angle)<=90){
    RLimitPWM_++;
    LLimitPWM_--;
  }
  if(round(headingDegrees)<round(t_angle)){
    RLimitPWM_++;
    LLimitPWM_--;
  }else if(round(headingDegrees)>round(t_angle)){
    RLimitPWM_--;
    LLimitPWM_++;
  }
  if(RLimitPWM_<50){
    RLimitPWM_=RLimitPWM;
  }
  if(LLimitPWM_<50){
    LLimitPWM_=LLimitPWM;
  }
  if(RLimitPWM_>400){
    RLimitPWM_=RLimitPWM;
  }
  if(LLimitPWM_>400){
    LLimitPWM_=LLimitPWM;
  }
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW);
  analogWrite(MotorRPWM,RLimitPWM_);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);
  analogWrite(MotorLPWM,LLimitPWM_);
  delay(runtime * 10);

  //create BLE notification message
    //发送状态信息
    memset(buf,0,9);
    sprintf(buf, "%d",(int)round(t_angle));
    strcat(debugmsg,buf);
    strcat(debugmsg,",");
    memset(buf,0,9);
    sprintf(buf, "%d",(int)round(headingDegrees));
    strcat(debugmsg,buf);
    strcat(debugmsg,",");
    memset(buf,0,9);
    sprintf(buf, "%d",(int)round(Fdistance));
    strcat(debugmsg,buf);
    strcat(debugmsg,",");
    memset(buf,0,9);
    sprintf(buf, "%d",RLimitPWM_);
    strcat(debugmsg,buf);
    strcat(debugmsg,",");
    memset(buf,0,9);
    sprintf(buf, "%d",LLimitPWM_);
    strcat(debugmsg,buf);

}

void turnR(int d)        //右轉(雙輪)
    {
     digitalWrite(pinRB,HIGH);  //使馬達（右後）動作
     digitalWrite(pinRF,LOW);
     analogWrite(MotorRPWM,baseLimitPWM);
     digitalWrite(pinLB,LOW);
     digitalWrite(pinLF,HIGH);  //使馬達（左前）動作
     analogWrite(MotorLPWM,baseLimitPWM);
     delay(d * 60);
    }

void turnL(int e)        //左轉(雙輪)
    {
     digitalWrite(pinRB,LOW);
     digitalWrite(pinRF,HIGH);   //使馬達（右前）動作
     analogWrite(MotorRPWM,baseLimitPWM);
     digitalWrite(pinLB,HIGH);   //使馬達（左後）動作
     digitalWrite(pinLF,LOW);
     analogWrite(MotorLPWM,baseLimitPWM);
     delay(e * 60);
    }

void stopp(int f)         //停止
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,HIGH);
     delay(f * 100);
    }

void back(int g)          //後退
    {

     digitalWrite(pinRB,LOW);  //使馬達（右後）動作
     digitalWrite(pinRF,HIGH);
     analogWrite(MotorRPWM,RLimitPWM);
     digitalWrite(pinLB,LOW);  //使馬達（左後）動作
     digitalWrite(pinLF,HIGH);
     analogWrite(MotorLPWM,LLimitPWM);
     delay(g * 100);
    }

void ask_pin_F()   // 量出前方距離
    {
      digitalWrite(outputPin, LOW);   // 讓超聲波發射低電壓2μs
      delayMicroseconds(2);
      digitalWrite(outputPin, HIGH);  // 讓超聲波發射高電壓10μs，這裡至少是10μs
      delayMicroseconds(10);
      digitalWrite(outputPin, LOW);    // 維持超聲波發射低電壓
      Fdistance = pulseIn(inputPin, HIGH);  // 讀差相差時間
      Fdistance= Fdistance/5.8/10;       // 將時間轉為距離距离（單位：公分）
      Fspeedd = Fdistance;              // 將距離 讀入Fspeedd(前速)
    }

void checkDirection(){
  Vector norm = compass.readNormalize();
  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
  // Convert to degrees
  headingDegrees = heading * 180/M_PI;
}
//取到当前温度信息以及湿度信息
// void getEnvInfo(){
//   byte dht11_dat[5];
// 	byte dht11_in;
// 	byte i;
// 	// start condition
// 	// 1. pull-down i/o pin from 18ms
// 	PORTC &= ~_BV(DHT11_PIN);
// 	delay(18);
// 	PORTC |= _BV(DHT11_PIN);
// 	delayMicroseconds(40);
// 	DDRC &= ~_BV(DHT11_PIN);
// 	delayMicroseconds(40);
// 	dht11_in = PINC & _BV(DHT11_PIN);
// 	if(dht11_in){
//     // dht11 start condition 1 not met
// 		strcat(debugmsg,"err1,");
// 		return;
// 	}
// 	delayMicroseconds(80);
// 	dht11_in = PINC & _BV(DHT11_PIN);
// 	if(!dht11_in){
//     //dht11 start condition 2 not met
// 		strcat(debugmsg,"err2,");
// 		return;
// 	}
// 	delayMicroseconds(80);
// 	// now ready for data reception
// 	for (i=0; i<5; i++)
// 		dht11_dat[i] = read_dht11_dat();
// 	DDRC |= _BV(DHT11_PIN);
// 	PORTC |= _BV(DHT11_PIN);
//   byte dht11_check_sum = dht11_dat[0]+dht11_dat[1]+dht11_dat[2]+dht11_dat[3];
// 	// check check_sum
// 	if(dht11_dat[4]!= dht11_check_sum)
// 	{
//     // DHT11 checksum error
// 		strcat(debugmsg,"err3,");
// 	}
//   char info[10];
//   memset(info,0,9);
// 	strcat(debugmsg,",");
// 	sprintf(info,"%d",dht11_dat[0]);
// 	strcat(debugmsg,".");
//   memset(info,0,9);
// 	sprintf(info,"%d",dht11_dat[1]);
//   memset(info,0,9);
// 	strcat(debugmsg,",");
// 	sprintf(info,"%d",dht11_dat[2]);
// 	strcat(debugmsg,".");
//   memset(info,0,9);
// 	sprintf(info,"%d",dht11_dat[3]);
//   strcat(debugmsg,",");
// }

//初始化
 void setup(){
   //DDRC |= _BV(DHT11_PIN);
   //PORTC |= _BV(DHT11_PIN);

   Serial.begin(9600);
   BTSerial.begin(9600);

   pinMode(pinLB,OUTPUT); // 腳位 8 (PWM)
   pinMode(pinLF,OUTPUT); // 腳位 9 (PWM)
   pinMode(pinRB,OUTPUT); // 腳位 10 (PWM)
   pinMode(pinRF,OUTPUT); // 腳位 11 (PWM)

   pinMode(MotorLPWM,  OUTPUT);  // 腳位 3 (PWM)
   pinMode(MotorRPWM,  OUTPUT);  // 腳位 5 (PWM)

   pinMode(inputPin, INPUT);    // 定義超音波輸入腳位
   pinMode(outputPin, OUTPUT);  // 定義超音波輸出腳位
   while (!compass.begin()){
     Serial.println("err0,");
     delay(500);
   }
     // Set measurement range
     compass.setRange(HMC5883L_RANGE_1_3GA);
     // Set measurement mode
     compass.setMeasurementMode(HMC5883L_CONTINOUS);
     // Set data rate
     compass.setDataRate(HMC5883L_DATARATE_30HZ);
     // Set number of samples averaged
     compass.setSamples(HMC5883L_SAMPLES_8);
     // Set calibration offset. See HMC5883L_calibration.ino
     compass.setOffset(0, 0);
     Serial.println("start");
  }

//运行主体
void loop(){
  //从BLE中读取新指令
   String cmd="";
   String taskId="";
   while (BTSerial.available() > 0)
    {
        cmd += char(BTSerial.read());
        delay(2);
    }
    //for debug
    cmd="a1,90.0,1";

    if (cmd.length() > 0)
    {
        if(debug){
          Serial.println(cmd);
        }
        //取得指令(任务号，角度,-1/0/1/2/3)最后-1倒退0为停止1为前进2为左转3为右转
        int index = cmd.indexOf(',');
        taskId= cmd.substring(0, index);
        index=index+1;
        float angle_ = cmd.substring(index, cmd.length()-2).toFloat();
        int oper=cmd.substring(cmd.length()-1, cmd.length()).toInt();
        switch (oper){
          case -1:
            back(1);
            break;
          case 0:
            stopp(1);
            break;
          case 1:
            advance(angle_,1);
            break;
          case 2:
            turnL(1);
            break;
          case 3:
            turnL(1);
            break;
        }

        //通过BLE广播状态信息
        BTSerial.println(taskId+","+debugmsg);
      }
      if(debug){
        Serial.println(taskId+","+debugmsg);
        Serial.println("### Loop ###");
      }
 }
