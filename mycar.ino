//超声波避障&寻迹+LCD显示debug信息
#include <Servo.h>
#include<stdlib.h>
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
Servo myservo;        // 設 myservo
int delay_time = 250; // 伺服馬達轉向後的穩定時間

int Fgo = 8;         // 前進
int Rgo = 6;         // 右轉
int Lgo = 4;         // 左轉
int Bgo = 2;         // 倒車


int LCD1602_RS=13;
int LCD1602_RW=18;
int LCD1602_EN=19;
int DB[] = { 4,6,7,12};
char distanceMsg[10];

void setup()
 {
  Serial.begin(9600);     // 定義馬達輸出腳位
  pinMode(pinLB,OUTPUT); // 腳位 8 (PWM)
  pinMode(pinLF,OUTPUT); // 腳位 9 (PWM)
  pinMode(pinRB,OUTPUT); // 腳位 10 (PWM)
  pinMode(pinRF,OUTPUT); // 腳位 11 (PWM)

  pinMode(MotorLPWM,  OUTPUT);  // 腳位 3 (PWM)
  pinMode(MotorRPWM,  OUTPUT);  // 腳位 5 (PWM)

  pinMode(inputPin, INPUT);    // 定義超音波輸入腳位
  pinMode(outputPin, OUTPUT);  // 定義超音波輸出腳位

  myservo.attach(10);    // 定義伺服馬達輸出第10腳位(PWM)
  //init LCD
  int i = 0;
  for (i=6; i <= 12; i++)
   {
     pinMode(i,OUTPUT);
   }
  delay(100);
  LCD_Command_Write(0x28);//4线 2行 5x7
  delay(50);
  LCD_Command_Write(0x06);
  delay(50);
  LCD_Command_Write(0x0c);
  delay(50);
  LCD_Command_Write(0x80);
  delay(50);
  LCD_Command_Write(0x01);
  delay(50);
 }
/*
//in loop function
LCD_Command_Write(0x01);
delay(50);
LCD_Write_String(0,0,str3);
delay(50);
LCD_Write_String(0,1,str4);
*/
 void LCD_Command_disp(char *command1,char * command2){
   LCD_Command_Write(0x01);
   delay(50);
   LCD_Write_String(0,0,command1);
   delay(50);
   LCD_Write_String(0,1,command2);
 }
 void LCD_Command_Write(int command)
 {
 int i,temp;
 digitalWrite( LCD1602_RS,LOW);
 digitalWrite( LCD1602_RW,LOW);
 digitalWrite( LCD1602_EN,LOW);

 temp=command & 0xf0;
 for (i=DB[0]; i <= 9; i++)
 {
    digitalWrite(i,temp & 0x80);
    temp <<= 1;
 }

 digitalWrite( LCD1602_EN,HIGH);
 delayMicroseconds(1);
 digitalWrite( LCD1602_EN,LOW);

 temp=(command & 0x0f)<<4;
 for (i=DB[0]; i <= 9; i++)
 {
    digitalWrite(i,temp & 0x80);
    temp <<= 1;
 }

 digitalWrite( LCD1602_EN,HIGH);
 delayMicroseconds(1);
 digitalWrite( LCD1602_EN,LOW);
 }

 void LCD_Data_Write(int dat)
 {
 int i=0,temp;
 digitalWrite( LCD1602_RS,HIGH);
 digitalWrite( LCD1602_RW,LOW);
 digitalWrite( LCD1602_EN,LOW);

 temp=dat & 0xf0;
 for (i=DB[0]; i <= 9; i++)
 {
    digitalWrite(i,temp & 0x80);
    temp <<= 1;
 }

 digitalWrite( LCD1602_EN,HIGH);
 delayMicroseconds(1);
 digitalWrite( LCD1602_EN,LOW);

 temp=(dat & 0x0f)<<4;
 for (i=DB[0]; i <= 9; i++)
 {
    digitalWrite(i,temp & 0x80);
    temp <<= 1;
 }

 digitalWrite( LCD1602_EN,HIGH);
 delayMicroseconds(1);
 digitalWrite( LCD1602_EN,LOW);
 }

 void LCD_SET_XY( int x, int y )
 {
   int address;
   if (y ==0)    address = 0x80 + x;
   else          address = 0xC0 + x;
   LCD_Command_Write(address);
 }

 void LCD_Write_Char( int x,int y,int dat)
 {
   LCD_SET_XY( x, y );
   LCD_Data_Write(dat);
 }

 void LCD_Write_String(int X,int Y,char *s)
 {
     LCD_SET_XY( X, Y );    //设置地址
     while (*s)             //写字符串
     {
       LCD_Data_Write(*s);
       s ++;
     }
 }

void advance(int a)     // 前進
    {
     digitalWrite(pinRB,HIGH);  // 使馬達（右後）動作
     digitalWrite(pinRF,LOW);
     analogWrite(MotorRPWM,150);
     digitalWrite(pinLB,HIGH);  // 使馬達（左後）動作
     digitalWrite(pinLF,LOW);
     analogWrite(MotorLPWM,150);
     delay(a * 100);
    }

void right(int b)        //右轉(單輪)
    {
     digitalWrite(pinRB,LOW);   //使馬達（右後）動作
     digitalWrite(pinRF,HIGH);
     analogWrite(MotorRPWM,200);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,HIGH);
     delay(b * 100);
    }
void left(int c)         //左轉(單輪)
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);   //使馬達（左後）動作
     digitalWrite(pinLF,HIGH);
     analogWrite(MotorLPWM,200);
     delay(c * 100);
    }
void turnR(int d)        //右轉(雙輪)
    {
     digitalWrite(pinRB,HIGH);  //使馬達（右後）動作
     digitalWrite(pinRF,LOW);
     analogWrite(MotorRPWM,100);
     digitalWrite(pinLB,LOW);
     digitalWrite(pinLF,HIGH);  //使馬達（左前）動作
     analogWrite(MotorLPWM,100);
     delay(d * 60);
    }
void turnL(int e)        //左轉(雙輪)
    {
     digitalWrite(pinRB,LOW);
     digitalWrite(pinRF,HIGH);   //使馬達（右前）動作
     analogWrite(MotorRPWM,100);
     digitalWrite(pinLB,HIGH);   //使馬達（左後）動作
     digitalWrite(pinLF,LOW);
     analogWrite(MotorLPWM,100);
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
     analogWrite(MotorRPWM,150);
     digitalWrite(pinLB,LOW);  //使馬達（左後）動作
     digitalWrite(pinLF,HIGH);
     analogWrite(MotorLPWM,100);
     delay(g * 70);
    }

void detection()        //測量3個角度(0.90.179)
    {
      int delay_time = 200;   // 伺服馬達轉向後的穩定時間
      ask_pin_F();            // 讀取前方距離

     if(Fspeedd < 10)         // 假如前方距離小於10公分
      {
      stopp(1);               // 清除輸出資料
      back(2);                // 後退 0.2秒
      }

      if(Fspeedd < 25)         // 假如前方距離小於25公分
      {
        stopp(1);               // 清除輸出資料
        ask_pin_L();            // 讀取左方距離
        delay(delay_time);      // 等待伺服馬達穩定
        ask_pin_R();            // 讀取右方距離
        delay(delay_time);      // 等待伺服馬達穩定

        if(Lspeedd > Rspeedd)   //假如 左邊距離大於右邊距離
        {
         directionn = Lgo;      //向左走
        }

        if(Lspeedd <= Rspeedd)   //假如 左邊距離小於或等於右邊距離
        {
         directionn = Rgo;      //向右走
        }

        if (Lspeedd < 15 && Rspeedd < 15)   //假如 左邊距離和前方距離和右邊距離皆小於15公分
        {

         directionn = Bgo;      //向後走
        }
      }
      else                      //假如前方不小於(大於)25公分
      {
        directionn = Fgo;        //向前走
      }

    }
void ask_pin_F()   // 量出前方距離
    {
      myservo.write(90);
      digitalWrite(outputPin, LOW);   // 讓超聲波發射低電壓2μs
      delayMicroseconds(2);
      digitalWrite(outputPin, HIGH);  // 讓超聲波發射高電壓10μs，這裡至少是10μs
      delayMicroseconds(10);
      digitalWrite(outputPin, LOW);    // 維持超聲波發射低電壓
      float Fdistance = pulseIn(inputPin, HIGH);  // 讀差相差時間
      Fdistance= Fdistance/5.8/10;       // 將時間轉為距離距离（單位：公分）
      Serial.print("F distance:");      //輸出距離（單位：公分）
      Serial.println(Fdistance);         //顯示距離
      char msg=dtostrf(Fdistance,6,2,distanceMsg);
      LCD_Command_disp("F distance:",msg);
      Fspeedd = Fdistance;              // 將距離 讀入Fspeedd(前速)
    }
 void ask_pin_L()   // 量出左邊距離
    {
      myservo.write(5);
      delay(delay_time);
      digitalWrite(outputPin, LOW);   // 讓超聲波發射低電壓2μs
      delayMicroseconds(2);
      digitalWrite(outputPin, HIGH);  // 讓超聲波發射高電壓10μs，這裡至少是10μs
      delayMicroseconds(10);
      digitalWrite(outputPin, LOW);    // 維持超聲波發射低電壓
      float Ldistance = pulseIn(inputPin, HIGH);  // 讀差相差時間
      Ldistance= Ldistance/5.8/10;       // 將時間轉為距離距离（單位：公分）
      Serial.print("L distance:");       //輸出距離（單位：公分）
      Serial.println(Ldistance);         //顯示距離
      char msg=dtostrf(Ldistance,6,2,distanceMsg);
      LCD_Command_disp("L distance:",msg);
      Lspeedd = Ldistance;              // 將距離 讀入Lspeedd(左速)
    }
void ask_pin_R()   // 量出右邊距離
    {
      myservo.write(177);
      delay(delay_time);
      digitalWrite(outputPin, LOW);   // 讓超聲波發射低電壓2μs
      delayMicroseconds(2);
      digitalWrite(outputPin, HIGH);  // 讓超聲波發射高電壓10μs，這裡至少是10μs
      delayMicroseconds(10);
      digitalWrite(outputPin, LOW);    // 維持超聲波發射低電壓
      float Rdistance = pulseIn(inputPin, HIGH);  // 讀差相差時間
      Rdistance= Rdistance/5.8/10;       // 將時間轉為距離距离（單位：公分）
      Serial.print("R distance:");       //輸出距離（單位：公分）
      Serial.println(Rdistance);
      char msg=dtostrf(Rdistance,6,2,distanceMsg);
      LCD_Command_disp("R distance:",msg);         //顯示距離
      Rspeedd = Rdistance;              // 將距離 讀入Rspeedd(右速)
    }

void loop()
 {
    myservo.write(90);  //讓伺服馬達回歸 預備位置 準備下一次的測量
    detection();        //測量角度 並且判斷要往哪一方向移動

   if(directionn == 2)  //假如directionn(方向) = 2(倒車)
   {
     back(8);                    //  倒退(車)
     turnL(2);                   //些微向左方移動(防止卡在死巷裡)
     Serial.print(" Reverse ");   //顯示方向(倒退)
     LCD_Command_disp(" Reverse ","    ");
   }
   if(directionn == 6)           //假如directionn(方向) = 6(右轉)
   {
     back(1);
     turnR(6);                   // 右轉
     Serial.print(" Right ");    //顯示方向(左轉)
     LCD_Command_disp(" Right ","    ");
   }
   if(directionn == 4)          //假如directionn(方向) = 4(左轉)
   {
     back(1);
     turnL(6);                  // 左轉
     Serial.print(" Left ");     //顯示方向(右轉)
     LCD_Command_disp(" Left ","    ");
   }
   if(directionn == 8)          //假如directionn(方向) = 8(前進)
   {
    advance(1);                 // 正常前進
    Serial.print(" Advance ");   //顯示方向(前進)
    Serial.print("   ");
    LCD_Command_disp(" Advance ","    ");
   }
 }
