#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

#include <DS3231.h>
DS3231  rtc(SDA, SCL);
Time  t;

int pinA = 2; int gtlen;  // Lên là trừ
int pinB = 3; int gtxuong; // Xuống là cộng
int enSW= 10; int gtmenu; 
int macdinh = 1;

int in1 = 5; int in2 = 6;int in3=7;int in4=8;
int en1 =4;int en2=9;

int dongco1=A0;
int dongco2=A1;

//Day la phan khai bao do toc do
//Dong co 1
int encoder_pin1 = 11;             
unsigned int rpm1 = 0;           
float velocity1 = 0;                
volatile byte pulses1 = 0;       
unsigned long timeold1 = 0;  
unsigned int pulsesperturn1 = 20; 
const int wheel_diameter1 = 64;   
static volatile unsigned long debounce1 = 0;
//Dong co 2
int encoder_pin2 = 12;             
unsigned int rpm2 = 0;           
float velocity2 = 0;                
volatile byte pulses2 = 0;       
unsigned long timeold2 = 0;  
unsigned int pulsesperturn2 = 20; 
const int wheel_diameter2 = 64;   
static volatile unsigned long debounce2 = 0;



//Day la phan khai bao con tro
int contro = 0; int contro_dk = 5; int hang = 0;

int congtru_tong = 0; int congtru_menudk = 0;int congtru_dongco1=0;
int congtru_dongco2=0;int congtru_menudc=0;int congtru_2dongco=0;
int demtong = 0;

int ngay = 1; int thang = 1; int namng = 0; int namtr = 0; int namch = 0; int namdv = 0; int namtong = 0; //SETUP DATE
int gio = 0; int phut = 0; int giay = 0; //SETUP TIME

int ton1 = 0; int pon1 = 0; int tof1 = 0; int pof1 = 0; //LỆNH 1
int ton2 = 0; int pon2 = 0; int tof2 = 0; int pof2 = 0; //LỆNH 2
int ton3 = 0; int pon3 = 0; int tof3 = 0; int pof3 = 0; //LỆNH 3
int ton4 = 0; int pon4 = 0; int tof4 = 0; int pof4 = 0; //LỆNH 4
int ton5 = 0; int pon5 = 0; int tof5 = 0; int pof5 = 0; //LỆNH 5
int re1 = 1; int re2 = 1; int re3 = 1; int re4 = 1; int re5 = 1; //RELAY

volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
long encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
long oldPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; 
void PinA() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge

    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}
void manhinh()  // HIỂN THỊ MÀN HÌNH CHÍNH
{
  lcd.setCursor(0,0);
  lcd.print("DATE: ");
  lcd.setCursor(0,1);
  lcd.print("TIME: ");
  lcd.setCursor(6,0);
  lcd.print(rtc.getDateStr());
  lcd.setCursor(6,1);
  lcd.print(rtc.getTimeStr());  
}

void menu_tong() // HIỂN THỊ MENU TỔNG
{
  if (congtru_tong == 0)
  {
    lcd.clear();
    lcd.print(">Quay lai");
    lcd.setCursor(0, 1);
    lcd.print(" Chinh ngay");  
  }
  else if (congtru_tong == 1)
  {
    lcd.clear();
    lcd.print(" Quay lai");
    lcd.setCursor(0, 1);
    lcd.print(">Chinh ngay");    
  }
  else if (congtru_tong == 2)
  {
    lcd.clear();
    lcd.print(">Chinh gio");
    lcd.setCursor(0, 1);
    lcd.print(" Hen gio");   
  }
  else if (congtru_tong == 3)
  {
    lcd.clear();
    lcd.print(" Chinh gio");
    lcd.setCursor(0, 1);
    lcd.print(">Hen gio");    
    
  }
  else if (congtru_tong == 4)
  {
    lcd.clear();
    lcd.print(">Dieu khien dong");
    lcd.setCursor(1,1);
    lcd.print("co bang tay");
  }
} 

void chonmenu_tong() // CHỌN MENU TỔNG
{
  switch (congtru_tong) 
  {
    case 0:
      //BACK
      break;
    case 1:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ngay hien tai");
      lcd.setCursor(12,1);
      lcd.print("BACK");      
      break;
    case 2:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Gio hien tai");
      lcd.setCursor(12,1);
      lcd.print("BACK");       
      break;
    case 3:
      // LỆNH ĐIỀU KHIỂN
      break;
     case 4:
     // Lenh dieu khien dong co
     break;                       
  }
}

void menu_dongco()//Hien thi menu dong co
{
  if(congtru_menudc==0)
  {
    lcd.clear();
    lcd.print(">Quay lai");
    lcd.setCursor(0,1);
    lcd.print(" Dong co 1");
    
  }
  else if(congtru_menudc==1)
  {
    lcd.clear();
    lcd.print(" Quay lai");
    lcd.setCursor(0,1);
    lcd.print(">Dong co 1");
  }
  else if(congtru_menudc==2)
  {
    lcd.clear();
    lcd.print(">Dong co 2");
    lcd.setCursor(0,1);
    lcd.print(" Hai dong co");
  }
   else if(congtru_menudc==3)
  {
    lcd.clear();
    lcd.print(" Dong co 2");
    lcd.setCursor(0,1);
    lcd.print(">Hai dong co");
  }
}

void chonmenu_dongco()
{
  switch(congtru_menudc)
  {
    case 0:
    //BACK
    break;
    case 1:
    //Dong co 1
    break;
    case 2:
    //Dong co 2
    break;
    case 3:
    //Ca hai dong co
    break;
  }
}

void dongco_1()
{
  if(congtru_dongco1==0)
  {
    lcd.clear();
    lcd.print(">Quay lai");
    lcd.setCursor(0,1);
    lcd.print(" Bat");
  }
  else if(congtru_dongco1==1)
  {
    lcd.clear();
    lcd.print(" Quay lai");
    lcd.setCursor(0,1);
    lcd.print(">Bat");
  }
  else if(congtru_dongco1==2)
  {
    lcd.clear();
    lcd.print(">Tat");
  }
}

void dieukhien_dongco1()
{
  switch(congtru_dongco1)
  {
    case 0:
    break;
    case 1:
    break;
    break;
    case 2:
    break;
         
  }
}

void batdongco1()
{
          int giatri1=analogRead(dongco1);
          int tocdo1 = map(giatri1,0,1023,10,255);
          digitalWrite(in1,HIGH);
          digitalWrite(in2,LOW);
          analogWrite(en1,tocdo1);
          
}
void tatdongco1()
{
        
          digitalWrite(in1,LOW);
          digitalWrite(in2,LOW);
          analogWrite(en1,0);
          lcd.clear();
          lcd.setCursor(5,0);
          lcd.print("Da tat!");
          delay(500);
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print(">Tat");
}

void dongco_2()
{
  if(congtru_dongco2==0)
  {
    lcd.clear();
    lcd.print(">Quay lai");
    lcd.setCursor(0,1);
    lcd.print(" Bat");
  }
  else if(congtru_dongco2==1)
  {
    lcd.clear();
    lcd.print(" Quay lai");
    lcd.setCursor(0,1);
    lcd.print(">Bat");
  }
  else if(congtru_dongco2==2)
  {
    lcd.clear();
    lcd.print(">Tat");
  }
}
void batdongco2()
{
          int giatri2=analogRead(dongco2);
          int tocdo2 = map(giatri2,0,1023,10,255);
          digitalWrite(in3,HIGH);
          digitalWrite(in4,LOW);
          analogWrite(en2,tocdo2);
}
void tatdongco2()
{
        
          digitalWrite(in3,LOW);
          digitalWrite(in4,LOW);
          analogWrite(en2,0);
          lcd.clear();
          lcd.setCursor(5,0);
          lcd.print("Da tat!");
          delay(500);
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print(">Tat");
}

void dieukhien_dongco2()
{
  switch(congtru_dongco2)
  {
    case 0:
    break;
    case 1:
    break;
    break;
    case 2:
    break;
         
  }
}
//Bat ca hai dong co
void hienthi_2dongco()
{
 
  if(congtru_2dongco==0)
  {
    lcd.clear();
    lcd.print(">Quay lai");
    lcd.setCursor(0,1);
    lcd.print(" Bat");
  }
  else if(congtru_2dongco==1)
  {
    lcd.clear();
    lcd.print(" Quay lai");
    lcd.setCursor(0,1);
    lcd.print(">Bat");
  }
  else if(congtru_2dongco==2)
  {
    lcd.clear();
    lcd.print(">Tat");
  }

}

void dieukhien_2dongco()
{
  switch(congtru_2dongco)
  {
    case 0:
    break;
    case 1:
    break;
    break;
    case 2:
    break;
         
  }
}
void bat_2dongco()
{
          int giatri1=analogRead(dongco1);
          int giatri2=analogRead(dongco1);
          int tocdo1 = map(giatri1,0,1023,10,255);
          int tocdo2 = map(giatri2,0,1023,10,255);
          digitalWrite(in1,HIGH);
          digitalWrite(in2,LOW);
          analogWrite(en1,tocdo1);
          digitalWrite(in3,HIGH);
          digitalWrite(in4,LOW);
          analogWrite(en2,tocdo2);
          
}
void tat_2dongco()
{
        
          digitalWrite(in1,LOW);
          digitalWrite(in2,LOW);
          analogWrite(en1,0);
          digitalWrite(in3,LOW);
          digitalWrite(in4,LOW);
          analogWrite(en2,0);
          lcd.clear();
          lcd.setCursor(5,0);
          lcd.print("Da tat!");
          delay(500);
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print(">Tat");
}



void menu_dieukhien() // HIỂN THỊ MENU ĐIỀU KHIỂN
{
  if (congtru_menudk == 0)
  {
    lcd.clear();
    lcd.print(">Quay lai");
    lcd.setCursor(0, 1);
    lcd.print(" Hen gio 1");  
  }
  else if (congtru_menudk == 1)
  {
    lcd.clear();
    lcd.print(" Quay lai");
    lcd.setCursor(0, 1);
    lcd.print(">Hen gio 1");    
  }
  else if (congtru_menudk == 2)
  {
    lcd.clear();
    lcd.print(">Hen gio 2");
    lcd.setCursor(0, 1);
    lcd.print(" Hen gio 3");    
  } 
  else if (congtru_menudk == 3)
  {
    lcd.clear();
    lcd.print(" Hen gio 2");
    lcd.setCursor(0, 1);
    lcd.print(">Hen gio 3");    
  } 
  else if (congtru_menudk == 4)
  {
    lcd.clear();
    lcd.print(">Hen gio 4");
    lcd.setCursor(0, 1);
    lcd.print(" Hen gio 5");    
  } 
  else if (congtru_menudk == 5)
  {
    lcd.clear();
    lcd.print(" Hen gio 4");
    lcd.setCursor(0, 1);
    lcd.print(">Hen gio 5");    
  }        
}

void chonmenu_dieukhien() // CHỌN MENU ĐIỀU KHIỂN
{
  switch (congtru_menudk) 
  {
    case 0:
      //BACK
      break;
    case 1:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Bat: ");
      lcd.setCursor(11,0);
      lcd.print("DC: ");      
      lcd.setCursor(0,1);
      lcd.print("Tat: ");
      lcd.setCursor(12,1);
      lcd.print("BACK");
      break;
    case 2:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Bat: ");
      lcd.setCursor(11,0);
      lcd.print("DC: ");      
      lcd.setCursor(0,1);
      lcd.print("Tat: ");
      lcd.setCursor(12,1);
      lcd.print("BACK");     
      break;
    case 3:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Bat: ");
      lcd.setCursor(11,0);
      lcd.print("DC: ");      
      lcd.setCursor(0,1);
      lcd.print("Tat: ");
      lcd.setCursor(12,1);
      lcd.print("BACK");     
      break;
    case 4:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Bat: ");
      lcd.setCursor(11,0);
      lcd.print("DC: ");      
      lcd.setCursor(0,1);
      lcd.print("Tat: ");
      lcd.setCursor(12,1);
      lcd.print("BACK");       
      break;
    case 5:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Bat: ");
      lcd.setCursor(11,0);
      lcd.print("DC: ");      
      lcd.setCursor(0,1);
      lcd.print("Tat: ");
      lcd.setCursor(12,1);
      lcd.print("BACK");       
      break;                             
  }
}

void HTset(int &ton, int &pon, int &tof, int &pof, int &re) //Hiển thị Setup ON/OFF
{
  if (ton < 10){
    lcd.setCursor(5,0); lcd.print("0");
    lcd.setCursor(6,0); lcd.print(ton);} 
  else
    {lcd.setCursor(5,0); lcd.print(ton);}

  lcd.setCursor(7,0); lcd.print(":");

  if (pon < 10){
    lcd.setCursor(8,0); lcd.print("0"); 
    lcd.setCursor(9,0); lcd.print(pon);} 
  else
    {lcd.setCursor(8,0); lcd.print(pon);}    

  lcd.setCursor(14,0); lcd.print(re); //relay 

  if (tof < 10){
    lcd.setCursor(5,1); lcd.print("0");
    lcd.setCursor(6,1); lcd.print(tof);}
  else
    {lcd.setCursor(5,1); lcd.print(tof);}

  lcd.setCursor(7,1); lcd.print(":");

  if (pof < 10){
    lcd.setCursor(8,1); lcd.print("0"); 
    lcd.setCursor(9,1); lcd.print(pof);}
  else
    {lcd.setCursor(8,1); lcd.print(pof);}    

  lcd.setCursor(contro_dk, hang); 
  lcd.cursor();
  delay(50);   
}

void on(int &relay)
{
  if(relay == 1)
  {
      if (digitalRead(in1) == LOW && digitalRead(in2)==LOW)
      {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(en1,255);
      }
      else 
      {
        digitalWrite(in1, HIGH); 
        digitalWrite(in2, LOW);
        digitalWrite(en1,255);
      }
   }

      else if(relay == 2)
      {
        if (digitalRead(in3) == LOW && digitalRead(in4)==LOW)
        {
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          digitalWrite(en2,255);
        }
        
        else 
        {
          digitalWrite(in3, HIGH);  
          digitalWrite(in4, LOW);
          digitalWrite(en2,255);
        }
      }

      else if(relay == 3)
      {
        if (digitalRead(in1) == LOW && digitalRead(in2)==LOW)
        {
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(en1,255);
        }
        else 
        {
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(en1,255);
        }
       if (digitalRead(in3) == LOW && digitalRead(in4)==LOW)
        {
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          digitalWrite(en2,255);
        }
       else 
       {
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW); 
          digitalWrite(en2,255);
       }
      }  
}

//Lenh tat
void off(int &relay)
{
  if(relay == 1)
  {
      if (digitalRead(in1) == HIGH && digitalRead(in2)==LOW)
        {
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(en1,0);
        }
      else 
      {
        digitalWrite(in1, LOW); 
        digitalWrite(in2, LOW);
        digitalWrite(en1,0);
      }
  }
    
    else if(relay == 2)
    {
      if (digitalRead(in3) == HIGH && digitalRead(in4) ==LOW)
        {
          digitalWrite(in3, LOW);
          digitalWrite(in4, LOW);
          digitalWrite(en2,0);
        }
      else 
      {
        digitalWrite(in3, LOW); 
        digitalWrite(in4, LOW);
        digitalWrite(en2,0);
      }
    }    
    
    else if(relay == 3)
    {
      if (digitalRead(in1) == HIGH && digitalRead(in2) ==LOW)
        {
          digitalWrite(in1, LOW);
          digitalWrite(in2,LOW);
          digitalWrite(en1,0);
        }
      else 
      {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(en1,0);
      }
      if (digitalRead(in3) == HIGH && digitalRead(in4)==LOW)
        {
          digitalWrite(in3, LOW);
          digitalWrite(in4,LOW);
          digitalWrite(en2,0);
        }
      else 
      {
        digitalWrite(in3, LOW); 
        digitalWrite(in4,LOW);
        digitalWrite(en2,0);
      }
     }          
}

void CongRelay(int &relay) //Cộng relay
{
  relay ++;
  if (relay > 3)
    relay = 1;
}
void TruRelay(int &relay) //Trừ Relay
{
  relay --;
  if (relay < 1)
    relay = 3;
}

void CongGio(int &ton, int &tof) //+ Giờ ON/OFF
{
  if (hang == 0){ //GIỜ ON 
    ton ++;
    if (ton > 23)
      ton = 0; } 
  else if (hang == 1){ //GIỜ OFF
    tof ++;
    if (tof > 23)
      tof = 0; }
}
void CongPhut(int &pon, int &pof) //+ Phút ON/OFF
{
  if (hang == 0){ //PHÚT ON
    pon ++;
    if (pon > 59)
      pon = 0; } 
  else if (hang == 1){ //PHÚT OFF
    pof ++;
    if (pof > 59)
      pof = 0; }
}
void TruGio(int &ton, int &tof) //- Giờ ON/OFF
{
  if (hang == 0){ //GIỜ ON 
    ton --;
    if (ton < 0)
      ton = 23; } 
  else if (hang == 1){ //GIỜ OFF
    tof --;
    if (tof < 0)
      tof = 23; }
}
void TruPhut(int &pon, int &pof) //- Phút ON/OFF
{
  if (hang == 0){ //PHÚT ON
    pon --;
    if (pon < 0)
      pon = 59; } 
  else if (hang == 1){ //PHÚT OFF
    pof --;
    if (pof < 0)
      pof = 59; }
}

void setup() 
{
  Serial.begin(9600);
  rtc.begin();
  lcd.init();
  lcd.backlight();
  
  
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(enSW, INPUT_PULLUP);
  attachInterrupt(0,PinA,RISING);
  attachInterrupt(1,PinB,RISING);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en1,INPUT);
  pinMode(dongco1,INPUT);
  
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(en2,INPUT);
  pinMode(dongco2,INPUT);

  pinMode(encoder_pin1, INPUT);
  attachPCINT(digitalPinToPCINT(encoder_pin1), counter1, RISING);
  pulses1 = 0;
  rpm1= 0;
  timeold1 = 0;

  pinMode(encoder_pin2, INPUT);
  attachPCINT(digitalPinToPCINT(encoder_pin2), counter2, RISING);
  pulses1 = 0;
  rpm1= 0;
  timeold1 = 0;
  
}

void counter1()
{
  if(  digitalRead (encoder_pin1) && (micros()-debounce1 > 500) && digitalRead (encoder_pin1) ) 
  { 
        debounce1 = micros(); 
        pulses1++;
        } 
        else ; 
 }
 void counter2()
{
  if(  digitalRead (encoder_pin2) && (micros()-debounce2 > 500) && digitalRead (encoder_pin2) ) 
  { 
        debounce2 = micros(); 
        pulses2++;
        } 
        else ; 
 }
 
 
 void dotocdo1()
 {
  if (millis() - timeold1 >= 1000)
  {  
      noInterrupts(); 
      rpm1 = (60 * 1000 / pulsesperturn1 )/ (millis() - timeold1)* pulses1; 
      velocity1 = rpm1 * 3.1416 * wheel_diameter1 * 60 / 1000000; 
      timeold1 = millis();
      //Serial.print(millis()/1000); Serial.print("       ");
      //lcd.print(rpm,DEC);
      //Serial.print(pulses,DEC); Serial.print("     ");
      //Serial.println(velocity,2); 
      pulses1 = 0;  
      interrupts(); 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Toc do hien tai");
      lcd.setCursor(0,1);
      lcd.print("la:");
      lcd.setCursor(4,1);
      lcd.print(rpm1,DEC);
      lcd.setCursor(9,1);
      lcd.print("vong/ph");
   }
 }
 void dotocdo2()
 {
  if (millis() - timeold2 >= 1000)
  {  
      noInterrupts(); 
      rpm2 = (60 * 1000 / pulsesperturn2 )/ (millis() - timeold2)* pulses2; 
      velocity2 = rpm2 * 3.1416 * wheel_diameter2 * 60 / 1000000; 
      timeold2 = millis();
      //Serial.print(millis()/1000); Serial.print("       ");
      //lcd.print(rpm,DEC);
      //Serial.print(pulses,DEC); Serial.print("     ");
      //Serial.println(velocity,2); 
      pulses2 = 0;  
      interrupts(); 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Toc do hien tai");
      lcd.setCursor(0,1);
      lcd.print("la:");
      lcd.setCursor(4,1);
      lcd.print(rpm2,DEC);
      lcd.setCursor(9,1);
      lcd.print("vong/ph");
   }
 }
 
void do_2tocdo()
 {
  if (millis() - timeold2 >= 1000 && millis() - timeold1 >= 1000)
  {  
      noInterrupts(); 
      rpm2 = (60 * 1000 / pulsesperturn2 )/ (millis() - timeold2)* pulses2; 
      rpm1 = (60 * 1000 / pulsesperturn1 )/ (millis() - timeold1)* pulses1;
      velocity2 = rpm2 * 3.1416 * wheel_diameter2 * 60 / 1000000;
      velocity1 = rpm1 * 3.1416 * wheel_diameter1 * 60 / 1000000; 
      timeold2 = millis();
      timeold1 = millis();
      //Serial.print(millis()/1000); Serial.print("       ");
      //lcd.print(rpm,DEC);
      //Serial.print(pulses,DEC); Serial.print("     ");
      //Serial.println(velocity,2); 
      pulses1 = 0;  
      pulses2 = 0; 
      interrupts(); 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("DC1:");
      lcd.setCursor(4,0);
      lcd.print(rpm1,DEC);
      lcd.setCursor(9,0);
      lcd.print("vong/ph");
      
      lcd.setCursor(0,1);
      lcd.print("DC2:");
      lcd.setCursor(4,1);
      lcd.print(rpm2,DEC);
      lcd.setCursor(9,1);
      lcd.print("vong/ph");
     
   }
 }






void loop() 
{
  t = rtc.getTime();
  
  //gtlen = digitalRead(pinA);  
  //gtxuong = digitalRead(pinB);  
  gtmenu = digitalRead(enSW);
  // Serial.print("LÊN: "); Serial.print(gtlen); Serial.print("    ");
  // Serial.print("XUỐNG: "); Serial.print(gtxuong); Serial.print("    ");
  // Serial.print("Menu: "); Serial.println(gtmenu);

if (oldPos!= encoderPos)  // NÚT LÊN
{
  if (oldPos < encoderPos) // Khi nhấn nút lên
  {
    if (demtong == 1)   // LÊN Ở MENU TỔNG  
    {
      if (congtru_tong >= 4)
      { 
        congtru_tong = 0;  
      }
      else
      { 
        congtru_tong++;  
      }   
      menu_tong();
    }
    
    else if (demtong == 2)    //+ Con trỏ Set Date/Time và lên Menu Điều khiển
    {
      if(congtru_tong == 1 or congtru_tong == 2) // + CON TRỎ ở SET DATE và TIME
      {
        contro ++;
        if (contro > 15)
          contro = 0;
      }
      else if(congtru_tong == 3) //Lên ở MENU ĐIỀU KHIỂN
      {
        congtru_menudk ++;
        if (congtru_menudk > 5)
          congtru_menudk = 0;
        menu_dieukhien();
      }
      else if(congtru_tong==4)
      {
        congtru_menudc++;
        if(congtru_menudc>3)
        {
          congtru_menudc=0;
        }
        menu_dongco();
      }
    }

    else if (demtong == 3)   
    {
      if(congtru_tong == 1) // - ở NGÀY, THÁNG, NĂM
      {
        if ((contro == 0 or contro == 1)){ //NGÀY
          ngay --;
          if (ngay < 1)
            ngay = 31;}
        else if ((contro == 3 or contro == 4)){ //THÁNG
          thang --;
          if (thang < 1)
            thang = 12;}

          else if (contro == 6){ //NĂM NGHÌN
            namng --;
            if (namng < 0)
              namng = 9;}
          else if (contro == 7){ //NĂM TRĂM
            namtr --;
            if (namtr < 0)
              namtr = 9;}
          else if (contro == 8){ //NĂM CHỤC
            namch --;
            if (namch < 0)
              namch = 9;}
          else if (contro == 9){ //NĂM ĐƠN VỊ
            namdv --;
            if (namdv < 0)
              namdv = 9;}
      }

      else if(congtru_tong == 2) // - ở GIỜ : PHÚT : GIÂY
      {
        if ((contro == 0 or contro == 1)){ //GIỜ
          gio --;
          if (gio < 0)
            gio = 23;}
        else if ((contro == 3 or contro == 4)){ //PHÚT
          phut --;
          if (phut < 0)
            phut = 59;}
        else if ((contro == 6 or contro == 7)){ //GIÂY
          giay --;
          if (giay < 0)
            giay = 59;}
      }

      else if(congtru_tong == 3) // + CON TRỎ ở ĐIỀU KHIỂN
      {
        contro_dk ++;
        if (hang == 0){
          if (contro_dk > 15)
          { contro_dk = 5;
            hang = 1;}
        } else {
          if (contro_dk > 15)
          { contro_dk = 5;
            hang = 0;}       
        }
      }
      else if(congtru_tong==4)
      {
        if(congtru_menudc==1)
        {
          congtru_dongco1++;
          if(congtru_dongco1>2)
          {
            congtru_dongco1=0;
          }
          dongco_1();
        }
        else if(congtru_menudc==2)
        {
          congtru_dongco2++;
          if(congtru_dongco2>2)
          {
            congtru_dongco2=0;
          }
          dongco_2();
        }
        else if(congtru_menudc==3)
        {
          congtru_2dongco++;
          if(congtru_2dongco>2)
          {
            congtru_2dongco=0;
          }
          hienthi_2dongco();
        }
      }
    }
    
    else if (demtong == 4 && congtru_tong == 3) // - ĐIỀU KHIỂN | GIỜ ON - OFF, RELAY
    {
      if(contro_dk == 5 or contro_dk == 6) //- Giờ ON/OFF
      {
        if(congtru_menudk == 1) //Giờ 1
          TruGio(ton1, tof1);
        else if(congtru_menudk == 2) //Giờ 2
          TruGio(ton2, tof2);
        else if(congtru_menudk == 3) //Giờ 3
          TruGio(ton3, tof3);
        else if(congtru_menudk == 4) //Giờ 4
          TruGio(ton4, tof4);
        else if(congtru_menudk == 5) //Giờ 5
          TruGio(ton5, tof5);                              
      }
      else if(contro_dk == 8 or contro_dk == 9) //- Phút ON/OFF
      {
        if(congtru_menudk == 1) //Phút 1
          TruPhut(pon1, pof1);
        else if(congtru_menudk == 2) //Phút 2
          TruPhut(pon2, pof2);
        else if(congtru_menudk == 3) //Phút 3
          TruPhut(pon3, pof3);
        else if(congtru_menudk == 4) //Phút 4
          TruPhut(pon4, pof4);
        else if(congtru_menudk == 5) //Phút 5
          TruPhut(pon5, pof5);                              
      }                               

      else if(hang == 0 && contro_dk == 14) // - RELAY (chọn relay on/off)
      {
        if(congtru_menudk == 1) //Lệnh 1
          TruRelay(re1);
        else if (congtru_menudk == 2) //Lệnh 2
          TruRelay(re2);
        else if (congtru_menudk == 3) //Lệnh 3
          TruRelay(re3);
        else if (congtru_menudk == 4) //Lệnh 4
          TruRelay(re4);     
        else if (congtru_menudk == 5) //Lệnh 5
          TruRelay(re5);                       
      }                         
    }                                         
   
    
  }
/*  oldPos =encoderPos;
  delay(50);
}*/





/*if (oldPos !=encoderPos) // NÚT XUỐNG
{*/
  if (oldPos > encoderPos) //Khi nhấn nút xuống
  {
    if (demtong == 1)   // XUỐNG Ở MENU TỔNG
    {
      if (congtru_tong <= 0)
      { 
        congtru_tong = 4;  
      }
      else
      { 
        congtru_tong--;  
      }
      menu_tong();
    }
    
    else if (demtong == 2)   //- Con trỏ Set Date/Time và Xuống Menu Điều khiển
    {
      if(congtru_tong == 1 or congtru_tong == 2) // - CON TRỎ ở SET DATE và TIME
      {
        contro --;
        if (contro < 0)
          contro = 15;
      }
      else if(congtru_tong == 3)  //Xuống ở MENU ĐIỀU KHIỂN
      {
        congtru_menudk --;
        if (congtru_menudk < 0)
          congtru_menudk = 5;
        menu_dieukhien();
      }
      else if(congtru_tong==4)
      {
        congtru_menudc--;
        if(congtru_menudc<0)
        {
          congtru_menudc=3;
        }
        menu_dongco();
      }
    }

    else if (demtong == 3)
    {
      if(congtru_tong == 1)  // + ở NGÀY, THÁNG, NĂM
      {
        if (contro == 0 or contro == 1){ // + NGÀY ở DATE
          ngay ++;
          if (ngay > 31)
            ngay = 1;}
        else if (contro == 3 or contro == 4){ // + THÁNG ở DATE
          thang ++;
          if (thang > 12)
            thang = 1;}
        else if (contro == 6){ // + NĂM NGHÌN
          namng ++;
          if (namng > 9)
            namng = 0;}
        else if (contro == 7){// + NĂM TRĂM
          namtr ++;
          if (namtr > 9)
            namtr = 0;}
        else if (contro == 8){// + NĂM CHỤC
          namch ++;
          if (namch > 9)
            namch = 0;}
        else if (contro == 9){// + NĂM ĐƠN VỊ
          namdv ++;
          if (namdv > 9)
            namdv = 0;}
      }

      else if(congtru_tong == 2) //+ ở GIỜ : PHÚT :GIÂY
      {
        if (contro == 0 or contro == 1){ //GIỜ
          gio ++;
          if (gio > 23)
            gio = 0;}
        else if (contro == 3 or contro == 4){ //PHÚT
          phut ++;
          if (phut > 59)
            phut = 0;}
        else if (contro == 6 or contro == 7){ //GIÂY
          giay ++;
          if (giay > 59)
            giay = 0;}
      }

      else if(congtru_tong == 3) // - CON TRỎ ở ĐIỀU KHIỂN
      {
        contro_dk --;
        if (hang == 0){
          if (contro_dk < 5)
          { contro_dk = 15;
            hang = 1;}
        } else {
          if (contro_dk < 5)
          { contro_dk = 15;
            hang = 0;}        
        } 
      }

      else if(congtru_tong==4)
      {
        if(congtru_menudc==1)
        {
          congtru_dongco1--;
          if(congtru_dongco1<0)
          {
            congtru_dongco1=2;
          }
          dongco_1();
        }
        else if(congtru_menudc==2)
        {
          congtru_dongco2--;
          if(congtru_dongco2<0)
          {
            congtru_dongco2=2;
          }
          dongco_2();
        }
        else if(congtru_menudc==3)
        {
          congtru_2dongco--;
          if(congtru_2dongco<0)
          {
            congtru_2dongco=2;
          }
          hienthi_2dongco();
        }
      }
    } 

    else if (demtong == 4 && congtru_tong == 3) //+ ĐIỀU KHIỂN | GIỜ ON - OFF, RELAY
    {
      if(contro_dk == 5 or contro_dk == 6) //+ Giờ ON/OFF
      {
        if(congtru_menudk == 1) //Giờ 1
          CongGio(ton1, tof1);
        else if(congtru_menudk == 2) //Giờ 2
          CongGio(ton2, tof2);
        else if(congtru_menudk == 3) //Giờ 3
          CongGio(ton3, tof3);
        else if(congtru_menudk == 4) //Giờ 4
          CongGio(ton4, tof4);
        else if(congtru_menudk == 5) //Giờ 5
          CongGio(ton5, tof5);                              
      }
      else if(contro_dk == 8 or contro_dk == 9) //+ Phút ON/OFF
      {
        if(congtru_menudk == 1) //Phút 1
          CongPhut(pon1, pof1);
        else if(congtru_menudk == 2) //Phút 2
          CongPhut(pon2, pof2);
        else if(congtru_menudk == 3) //Phút 3
          CongPhut(pon3, pof3);
        else if(congtru_menudk == 4) //Phút 4
          CongPhut(pon4, pof4);
        else if(congtru_menudk == 5) //Phút 5
          CongPhut(pon5, pof5);                              
      }                        

      else if(hang == 0 && contro_dk == 14) // + RELAY (chọn relay on/off)
      {
        if(congtru_menudk == 1) //Lệnh 1
          CongRelay(re1);
        else if (congtru_menudk == 2) //Lệnh 2
          CongRelay(re2);
        else if (congtru_menudk == 3) //Lệnh 3
          CongRelay(re3);
        else if (congtru_menudk == 4) //Lệnh 4
          CongRelay(re4);      
        else if (congtru_menudk == 5) //Lệnh 5
          CongRelay(re5);                        
      }                    
    }                                                  

    
  }
  oldPos = encoderPos;
  delay(10);
}





if (gtmenu != macdinh)    // NÚT MENU
{  
  if (gtmenu == 0) //Khi nhấn nút
  {  
    demtong ++;

    if (demtong == 1) //Ở menu tổng
    { 
      menu_tong(); 
    }
    else if (demtong == 2) 
    {
      if(congtru_tong ==0 ) //Nhấn BACK từ Menu tổng về màn hình
      {
        demtong = 0;
        manhinh(); 
      }
      else if(congtru_tong == 1 or congtru_tong == 2) // chọn menu tổng DATE or TIME
      {
        chonmenu_tong();
      }
      else if(congtru_tong == 3) //Menu ĐIỀU KHIỂN
      {
        menu_dieukhien();
      }
      else if(congtru_tong == 4)
      {
        menu_dongco();
        
      }
    }                                           

    else if (demtong == 4) //Thoát CON TRỎ
    {
      if(congtru_tong == 1) //Thoát CON TRỎ từ SET DATE ra
      {
        demtong = 2;
        chonmenu_tong();
      }
      else if(congtru_tong == 2) //Thoát CON TRỎ từ SET TIME ra
      {
        demtong = 2;
        chonmenu_tong(); 
      }

      /*else if(congtru_tong==4 && congtru_menudc==1 && congtru_dongco1==0)
      {
        demtong=3;
        menu_dongco();
      }
       else if(congtru_tong==4 && congtru_menudc==2 && congtru_dongco2==0)
      {
        demtong=3;
        menu_dongco();
      }*/

      
      else if (congtru_tong == 3) //Thoát từ SET BT ra menu báo thức
      {
        if((contro_dk == 12 or contro_dk == 13 or contro_dk == 14 or contro_dk == 15) && hang == 1)
        {
          menu_dieukhien();
          demtong = 2;
          congtru_tong = 3;
          contro_dk = 5;
          hang = 0;
          lcd.noCursor();
        }
      }
      else if(congtru_tong==4 && congtru_menudc ==1 && congtru_dongco1==0 )
      {
          demtong=2;
          menu_dongco();
      }
      else if(congtru_tong==4 && congtru_menudc ==2 && congtru_dongco2==0 )
      {
          demtong=2;
          menu_dongco();
      }
      else if(congtru_tong==4 && congtru_menudc ==3 && congtru_2dongco==0 )
      {
          demtong=2;
          menu_dongco();
      }
      
      
    }       
    
    else if (demtong == 3) 
    {
      if((congtru_tong == 2 or congtru_tong == 1) && 
          (contro == 12 or contro == 13 or contro == 14 or contro == 15)) // Thoát từ SET DATE or TIME ra
      {
        demtong = 1;
        congtru_tong = 0;
        contro = 0;
        menu_tong();
        lcd.noCursor();
      }
      else if(congtru_tong == 3 && congtru_menudk == 0) //từ Menu ĐIỀU KHIỂN về Menu TỔNG
      {
        demtong = 1;
        congtru_menudk = 0;
        menu_tong();
      }
      else if(congtru_tong ==4 && congtru_menudc==0)
      {
        demtong=1;
        congtru_menudc=0;
        menu_tong();
      }
      
      
     /* else if(congtru_tong ==3)
      {
        if(congtru_menudc==1)
        {
          dongco_1();
        }
        else if(congtru_menudc==2)
        {
          dongco_2();
        }
      }
      */
      
      
      
      else if(congtru_tong == 3 && (congtru_menudk == 1 or congtru_menudk == 2 
              or congtru_menudk == 3 or congtru_menudk == 4 or congtru_menudk == 5)) //chọn menu ĐK
      {
        chonmenu_dieukhien();
      }
     /* else if(congtru_tong==4 && congtru_menudc==1 && congtru_menudc==2)
      {
        chonmenu_dongco();
      }
      */
      else if(congtru_tong==4)  //Thoat tu dong co ra menu dong co bang tay
      {
        if(congtru_menudc==1)
        {
          dongco_1();
        }
        else if(congtru_menudc==2)
        {
          dongco_2();
        }
        else if(congtru_menudc==3)
        {
          hienthi_2dongco();
        }
      }
    }   

    else if (demtong == 5 && congtru_tong == 3 && (congtru_menudk == 1 or congtru_menudk == 2
            or congtru_menudk == 3 or congtru_menudk == 4 or congtru_menudk == 5))  //Từ CON TRỎ ĐK chức năng SET ra
      {
        chonmenu_dieukhien();
        demtong = 3;
      }
      
      else if(demtong==5 && congtru_tong==4 && congtru_menudc==1 && (congtru_dongco1==1 or congtru_dongco1==2) )
     {
        demtong=3;
        dongco_1();
       
      }

      else if(demtong==5 && congtru_tong==4 && congtru_menudc==2 && (congtru_dongco2==1 or congtru_dongco2==2) )
     {
        demtong=3;
        dongco_2();
      }
      else if(demtong==5 && congtru_tong==4 && congtru_menudc==3 && (congtru_2dongco==1 or congtru_2dongco==2) )
     {
        demtong=3;
        hienthi_2dongco();
      }
      /*else if(demtong==3 && congtru_tong==4 && (congtru_menudc==1 or congtru_menudc==2))
      {
        demtong=2;
        menu_dongco();
      }*/
      else if(demtong ==4 && congtru_tong==4 && congtru_menudc ==1 && (congtru_dongco1==1 or congtru_dongco1==2))
      {
        dieukhien_dongco1();
      }
      else if(demtong ==4 && congtru_tong==4 && congtru_menudc ==2 && (congtru_dongco2==1 or congtru_dongco2==2))
      {
        dieukhien_dongco2();
      }
      else if(demtong ==4 && congtru_tong==4 && congtru_menudc ==3 && (congtru_2dongco==1 or congtru_2dongco==2))
      {
        dieukhien_2dongco();
      }
      
      

    delay(100);
  }
  macdinh = gtmenu;
}

if (demtong == 0 && congtru_tong == 0) // MÀN HÌNH
{
  manhinh();
  lcd.noCursor();

// re1 = 1; re2 = 3; re3 = 2; re4 = 3; re5 = 1; //RELAY

// ton1 = 10; pon1 = 29; tof1 = 10; pof1 = 30; // 1 
// ton2 = 10; pon2 = 30; tof2 = 10; pof2 = 31; // 1, 2
// ton3 = 10; pon3 = 31; tof3 = 10; pof3 = 32; // 2
// ton4 = 10; pon4 = 32; tof4 = 10; pof4 = 33; // 1, 2
// ton5 = 10; pon5 = 33; tof5 = 10; pof5 = 34; // 1

  if(t.sec == 2) //ON
  {
    if(t.hour == ton1 && t.min == pon1) //LỆNH 1
      on(re1);
    else if(t.hour == ton2 && t.min == pon2) //LỆNH 2
      on(re2);
    else if(t.hour == ton3 && t.min == pon3) //LỆNH 3
      on(re3);
    else if(t.hour == ton4 && t.min == pon4) //LỆNH 4
      on(re4);
    else if(t.hour == ton5 && t.min == pon5) //LỆNH 5
      on(re5);                  
  }
  if(t.sec == 1) //OFF
  {
    if(t.hour == tof1 && t.min == pof1) //LỆNH 1
      off(re1);
    else if(t.hour == tof2 && t.min == pof2) //LỆNH 2
      off(re2);
    else if(t.hour == tof3 && t.min == pof3) //LỆNH 3
      off(re3);
    else if(t.hour == tof4 && t.min == pof4) //LỆNH 4
      off(re4);
    else if(t.hour == tof5 && t.min == pof5) //LỆNH 5
      off(re5);    
  }      
}

else if ((demtong == 2 or demtong == 3) && congtru_tong != 3) //SETUP DATE / TIME
{
  if(congtru_tong == 1) //DATE
  {
    if (ngay < 10){
      lcd.setCursor(0,1); lcd.print("0");
      lcd.setCursor(1,1); lcd.print(ngay);
    } else {
      lcd.setCursor(0,1); lcd.print(ngay);    
    }
    lcd.setCursor(2,1); lcd.print("/");
    if (thang < 10){
      lcd.setCursor(3,1); lcd.print("0"); 
      lcd.setCursor(4,1); lcd.print(thang);
    } else {
      lcd.setCursor(3,1); lcd.print(thang);    
    }
    lcd.setCursor(5,1); lcd.print("/"); 
    lcd.setCursor(6,1); lcd.print(namng); lcd.setCursor(7,1); lcd.print(namtr);
    lcd.setCursor(8,1); lcd.print(namch); lcd.setCursor(9,1); lcd.print(namdv);  
    
    namtong = (namng * 1000) + (namtr * 100) + (namch * 10) + namdv;

    lcd.setCursor(contro, 1); 
    lcd.cursor();
    delay(50);  

    rtc.setDate(ngay, thang, namtong);
  }
  else if (congtru_tong == 2) //TIME
  {
    if (gio < 10){
      lcd.setCursor(0,1); lcd.print("0");
      lcd.setCursor(1,1); lcd.print(gio);
    } else {
      lcd.setCursor(0,1); lcd.print(gio);
    }
    lcd.setCursor(2,1); lcd.print(":");
    if (phut < 10){
      lcd.setCursor(3,1); lcd.print("0"); 
      lcd.setCursor(4,1); lcd.print(phut);
    } else {
      lcd.setCursor(3,1); lcd.print(phut);    
    }  
    lcd.setCursor(5,1); lcd.print(":");
    if (giay < 10){
      lcd.setCursor(6,1); lcd.print("0"); 
      lcd.setCursor(7,1); lcd.print(giay);
    } else {
      lcd.setCursor(6,1); lcd.print(giay);    
    }

    lcd.setCursor(contro, 1); 
    lcd.cursor();
    delay(50);

    rtc.setTime(gio, phut, giay); 
  }
}

else if ((demtong == 3 or demtong == 4) && congtru_tong == 3) //Hiển thị SETUP ON/OFF
{ 
  if(congtru_menudk == 1) //Lệnh 1
    HTset(ton1, pon1, tof1, pof1, re1);     
  else if(congtru_menudk == 2) //Lệnh 2
    HTset(ton2, pon2, tof2, pof2, re2);  
  else if(congtru_menudk == 3) //Lệnh 3
    HTset(ton3, pon3, tof3, pof3, re3); 
  else if(congtru_menudk == 4) //Lệnh 4
    HTset(ton4, pon4, tof4, pof4, re4); 
  else if(congtru_menudk == 5) //Lệnh 5
    HTset(ton5, pon5, tof5, pof5, re5);              
}
else if(demtong ==4 && congtru_tong==4 && congtru_menudc ==1)
{
  if(congtru_dongco1==1 )
  {
    batdongco1();
    dotocdo1();
  }
  else if(congtru_dongco1==2)
  {
    tatdongco1();
    demtong=3;
  }
      
}

else if(demtong ==4 && congtru_tong==4 && congtru_menudc ==2)
{
  if(congtru_dongco2==1 )
  {
    batdongco2();
    dotocdo2();
  }
  else if(congtru_dongco2==2)
  {
    tatdongco2();
    demtong=3;
  }
      
}
else if(demtong ==4 && congtru_tong==4 && congtru_menudc ==3)
{
  if(congtru_2dongco==1 )
  {
    bat_2dongco();
    do_2tocdo();
  }
  else if(congtru_2dongco==2)
  {
    tat_2dongco();
    demtong=3;
  }
      
}


  Serial.print("Đếm tổng: "); Serial.print(demtong); Serial.print("    ");
  Serial.print("+/- tổng: "); Serial.print(congtru_tong); Serial.print("    ");
  Serial.print("CT Điều khiển: "); Serial.print(contro_dk); Serial.print("    ");
  Serial.print("Hàng: "); Serial.print(hang); Serial.print("    ");   
  Serial.print("+/- menu báo thức: "); Serial.print(congtru_menudk); Serial.print("    ");
  Serial.print("Con trỏ: "); Serial.print(contro);
  Serial.print("menudongco: ");Serial.println(congtru_menudc);Serial.print("    ");
  Serial.print("dong co 1: ");Serial.println(congtru_dongco1);Serial.print("    ");
  Serial.print(oldPos);Serial.print("    ");
  Serial.print(encoderPos);Serial.println("    ");
  

} //Kết thúc chương trình



  
