#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal_I2C.h>

//LCD
LiquidCrystal_I2C lcd(0x27,16,2);

//Photoresistor
const int C_pR = A1;
const int D_pR = 7;
const int E_pR = 8;
const int F_pR = 9;
const int G_pR = 10;
const int A_pR = 13;
const int B_pR = A0;
int C_in, D_in, E_in, F_in, G_in, A_in, B_in;

//Ultrasonic sensor
const int Cm = 261;
const int Dm = 294;
const int Em = 329;
const int Fm = 349;
const int Gm = 392;
const int Am = 440;
const int Bm = 493;

const int Ct = 523;
const int Dt = 587;
const int Et = 659;
const int Ft = 698;
const int Gt = 784;
const int At = 880;
const int Bt = 987;

const int trigPin = 3;
const int echoPin = 4;
long duration;
int distance;
int CHz, DHz, EHz, FHz, GHz, AHz, BHz;

//Buzzer & Button
const int buzzer = 12;
const int ModeBtn = 0; // INT0 is on pin2

//RGB LED
const int RED = 11;
const int GREEN = 6;
const int BLUE = 5;
boolean shine = true;

enum State{
  NORMAL,
  RECORD,
  PLAYING
}state;

//Record Mode
int myRecord[200] = {0};
int myTime[200] = {0};
int myNum = 0;
int nowHz = 0;
int start_time = 0;

void setup() {
  Serial.begin(9600);

  pinMode(C_pR, INPUT);
  pinMode(D_pR, INPUT);
  pinMode(E_pR, INPUT);
  pinMode(F_pR, INPUT);
  pinMode(G_pR, INPUT);
  pinMode(A_pR, INPUT);
  pinMode(B_pR, INPUT);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(ModeBtn, INPUT_PULLUP);
  attachInterrupt(ModeBtn, BtnClick, RISING);

  pinMode(buzzer, OUTPUT);

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  state = NORMAL;

  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Timer1
  cli(); // stop interrupts, atomic access to reg.
  TCCR1A = 0;    TCCR1B = 0;    TCNT1 = 0;
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1<<CS12) | (1<<CS10); // 1024 prescaler
  OCR1A = 7812;  //1 sec = 16 MHz/1024 = 15625 Hz
  TIMSK1 |= (1<<OCIE1A); // enable timer compare int.
  sei(); // enable all interrupts
}

void loop() {
  while(state == NORMAL) {
    lcd.setCursor(2,0);
    lcd.print(F("Free to Play"));
    
    C_in = analogRead(C_pR);
    D_in = digitalRead(D_pR);
    E_in = digitalRead(E_pR);
    F_in = digitalRead(F_pR);
    G_in = digitalRead(G_pR);
    A_in = digitalRead(A_pR);
    B_in = analogRead(B_pR);

    testDistance();

    if(C_in < 430) {
      buzz(CHz);
    } else if(!D_in) {
      buzz(DHz);
    } else if(!E_in) {
      buzz(EHz);
    } else if(!F_in) {
      buzz(FHz);
    } else if(!G_in) {
      buzz(GHz);
    } else if(!A_in) {
      buzz(AHz);
    } else if(B_in < 430) {
      buzz(BHz);
    } else {
      buzz(0);
    }
    delay(10);
  }

  while(state == RECORD) {
    lcd.setCursor(0,0);
    lcd.print(F("Recording...    "));
    
    C_in = analogRead(C_pR);
    D_in = digitalRead(D_pR);
    E_in = digitalRead(E_pR);
    F_in = digitalRead(F_pR);
    G_in = digitalRead(G_pR);
    A_in = digitalRead(A_pR);
    B_in = analogRead(B_pR);

    testDistance();

    if(C_in < 430) {
      Recording(CHz);
    } else if(!D_in) {
      Recording(DHz);
    } else if(!E_in) {
      Recording(EHz);
    } else if(!F_in) {
      Recording(FHz);
    } else if(!G_in) {
      Recording(GHz);
    } else if(!A_in) {
      Recording(AHz);
    } else if(B_in < 430) {
      Recording(BHz);
    } else {
      Recording(0);
    }

    delay(10);
  }

  while(state == PLAYING) {
    lcd.setCursor(0,0);
    lcd.print(F("Now Playing...  "));
    
    delay(500);
    for(int i = 1; i < myNum; i ++) {
      
      buzz(myRecord[i]);
      delay(myTime[i]);
    }

    buzz(0);
    delay(500);
    
    /*state = NORMAL;
    myNum = 0;*/
    lcd.clear();
  }
}

ISR(TIMER1_COMPA_vect) {
  if(state == NORMAL) {
    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
  } else if(state == RECORD) {
    if(shine) {
      analogWrite(RED, 255);
      shine = false;
    } else {
      analogWrite(RED, 0);
      shine = true;
    }
  } else if(state == PLAYING) {
    analogWrite(RED, 0);
    analogWrite(GREEN, 255);
  }
}

void BtnClick() {
  static unsigned long LastIntTime = 0;
  unsigned long IntTime = millis();

  if(IntTime - LastIntTime > 200) {
    switch(state) {
      case NORMAL:
        state = RECORD;
        start_time = millis();
        nowHz = 0;
        Serial.println(F("RECORD"));
        break;

      case RECORD:
        state = PLAYING;
        Serial.println(F("PLAYING"));
        break;

      case PLAYING:
        state = NORMAL;
        Serial.println(F("NORMAL"));
        myNum = 0;
        break;
    }
  }

  LastIntTime = IntTime;
}

void testDistance() {
  digitalWrite(trigPin, LOW); // Clears the trigPin
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = duration*0.034/2;

  if(distance < 7) {
    CHz = Ct;
    DHz = Dt;
    EHz = Et;
    FHz = Ft;
    GHz = Gt;
    AHz = At;
    BHz = Bt;
  } else {
    CHz = Cm;
    DHz = Dm;
    EHz = Em;
    FHz = Fm;
    GHz = Gm;
    AHz = Am;
    BHz = Bm;
  }
}

void Recording(int nextHz) {
  buzz(nextHz);
  
  if(nowHz != nextHz) {
    Serial.println(myNum);
    myRecord[myNum] = nowHz;
    myTime[myNum++] = millis() - start_time;
    //Serial.println(String(record_Hz) + " " + String(count));
    
    nowHz = nextHz;

    start_time = millis();
  }
}

void buzz(int in_Hz) {
  lcd.setCursor(0,1);
  if(in_Hz == 0) {
    noTone(buzzer);
    
    lcd.print(F("                "));
    
  } else {
    tone(buzzer, in_Hz);
    
    switch(in_Hz) {
      case Cm:
        lcd.print(F("Pitch : Middle C"));
        break;
      case Dm:
        lcd.print(F("Pitch : Middle D"));
        break;
      case Em:
        lcd.print(F("Pitch : Middle E"));
        break;
      case Fm:
        lcd.print(F("Pitch : Middle F"));
        break;
      case Gm:
        lcd.print(F("Pitch : Middle G"));
        break;
      case Am:
        lcd.print(F("Pitch : Middle A"));
        break;
      case Bm:
        lcd.print(F("Pitch : Middle B"));
        break;
  
      case Ct:
        lcd.print(F("Pitch : Tenor  C"));
        break;
      case Dt:
        lcd.print(F("Pitch : Tenor  D"));
        break;
      case Et:
        lcd.print(F("Pitch : Tenor  E"));
        break;
      case Ft:
        lcd.print(F("Pitch : Tenor  F"));
        break;
      case Gt:
        lcd.print(F("Pitch : Tenor  G"));
        break;
      case At:
        lcd.print(F("Pitch : Tenor  A"));
        break;
      case Bt:
        lcd.print(F("Pitch : Tenor  B"));
        break;
    }
  }
}
