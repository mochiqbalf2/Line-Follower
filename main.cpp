/*
  Pololu Robot D:12cm
  Mochammad Iqbal Faisal
  Mechatronics Enggineering
  EEPIS
  Arduino Source Code with PIO Visual Studio
*/
#include <Arduino.h>
//Inisialisasi Pin
#define in1 7
#define en1 5
#define in2 8
#define en2 6
#define buzz 9
#define swa 2
#define swb 3
#define swc 4
#define sw_line_depan 0
#define sw_line_belakang 1

#include <EEPROM.h>

//Insialisasi LCD
#include<LiquidCrystal_SR.h>
LiquidCrystal_SR lcd(11, 12, 10);

#include <avr/pgmspace.h>
const char menu_calib[] PROGMEM = "--Calibration--";
const char menu_readSensor[] PROGMEM = "--Read Sensor --";
const char menu_motorTest[] PROGMEM = "-- Motor Test --";
const char menu_pathplanning[] PROGMEM = "---Path Plan---";
const char menu_running[] PROGMEM = "-Running Robot -";

void menu_calibration();
void menu_read_sensor();
void menu_motor_test();
void menu_path_planning();
void menu_running_robot();

typedef void(*function)();
const function main_menu_functions[] = {menu_calibration, menu_read_sensor, menu_motor_test, menu_path_planning, menu_running_robot};
const char *const main_menu_options[] PROGMEM = {menu_calib, menu_readSensor, menu_motorTest, menu_pathplanning, menu_running};
//const char main_menu_length = sizeof(main_menu_options) / sizeof(main_menu_options[0]);

//Karakter
const char back_line2[] PROGMEM = "\6B";
const char menu_line2[] PROGMEM = "\x7f" "A \xa5" "B C\x7e";

int index = 0, index_calib = 0, index_running = 0;
char buffer[100];
boolean wait = false;

//////////////////////////////////////////////////////////////////////
void beep(int tim);
void resetVariable_calib();
void drive_motor(int logic_left, int logic_right, int pwm_left, int pwm_right);
void mode_sensor(int mode_sensor);
void turn_left(int mode_sensor, int heading);
void turn_right(int mode_sensor, int heading);
void turn_beep(int mode, int time_delay);
///////////////////////////////////////////////////////////
void menu_select() {
  wait = false;
  while (1) {
    digitalWrite(sw_line_depan, LOW);
    strcpy_P(buffer, (char*)pgm_read_word(&(main_menu_options[index])));
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(buffer);
    lcd.setCursor(0, 1); lcd.write(3);
    lcd.setCursor(7, 1); lcd.print("B.");
    lcd.setCursor(15, 1); lcd.write(2);
    delay(100);

    if (digitalRead(swa) == LOW) {//Button Left
      index -= 1;
      beep(100);
    }
    while (digitalRead(swa) == LOW);
    if (digitalRead(swc) == LOW) {//Button Right
      index += 1;
      beep(100);
    }
    while (digitalRead(swa) == LOW);
    if (digitalRead(swb) == LOW) { //Button Ok
      delay(500);
      while (digitalRead(swb) != LOW) {
        lcd.clear();
        lcd.setCursor(6, 0); lcd.print("wait");
        for (int a = 0; a < 16; a++) {
          lcd.setCursor(a, 1); lcd.print(".");
          delay(100);
        }
        beep(300);
        delay(1000);
        main_menu_functions[index]();
      }
    }
    if (index >= 4) {
      index = 4;
    }
    if (index <= 0) {
      index = 0;
    }
  }
}
///////////////////////////////////////////////////////////
const char menu_kalib_putih[] PROGMEM = "---Scan White---";
const char menu_kalib_hitam[] PROGMEM = "---Scan Black---";
const char menu_setting_run[] PROGMEM = "-Setting Param-";
const char menu_back_calib[] PROGMEM = "------Back------";

void kalibrasi_putih();
void kalibrasi_hitam();
void setting_param();
void back_calib();
const function main_menu_functions_calibration[] = {kalibrasi_putih, kalibrasi_hitam, setting_param, back_calib};
const char *const main_menu_options_calibration[] PROGMEM = {menu_kalib_putih, menu_kalib_hitam, menu_setting_run, menu_back_calib};

bool calib_white = false;
bool calib_black = false;
int pil_sensor = 0;//1 Sensor front 2 sensor back

void menu_calibration() {
  while (1) {
    calib_white = false;
    calib_black = false;
    pil_sensor = 0;
    resetVariable_calib();
    strcpy_P(buffer, (char*)pgm_read_word(&(main_menu_options_calibration[index_calib])));
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(buffer);
    lcd.setCursor(0, 1); lcd.write(3);
    lcd.setCursor(7, 1); lcd.print("B.");
    lcd.setCursor(15, 1); lcd.write(2);
    delay(100);

    if (digitalRead(swa) == LOW) {//Button Left
      index_calib -= 1;
      beep(100);
    }
    while (digitalRead(swa) == LOW);
    if (digitalRead(swc) == LOW) {//Button Right
      index_calib += 1;
      beep(100);
    }
    while (digitalRead(swa) == LOW);
    if (digitalRead(swb) == LOW) { //Button Ok
      delay(250);
      main_menu_functions_calibration[index_calib]();
    }
    while (digitalRead(swb) == LOW);
    if (index_calib >= 3) {
      index_calib = 3;
    }
    if (index_calib <= 0) {
      index_calib = 0;
    }
  }
}
void back_calib() {
  menu_select();
}
//Konstanta PID
double error, error_previous;
double pv, sp = 0;
double kp = 8.8, kd = 4.2, ki = 6.6, I1, I2, I3, D1, D2, D3;
double P, D, I, PD, PID;
double ts;
double ve = 70;
double vel_left, vel_right;
//Konstanta Maze
int mode_simpangan = 0; //0 tidak ada,1 Item, 2 Putih, 3 Kiri, 4 Kanan
bool detect_tengah = false;
//Subrutin Kalibrasi
int averagep0_front, averagep1_front, averagep2_front, averagep3_front, averagep4_front, averageh0_front, averageh1_front, averageh2_front, averageh3_front, averageh4_front;
int averagep0_back, averagep1_back, averagep2_back, averagep3_back, averagep4_back, averageh0_back, averageh1_back, averageh2_back, averageh3_back, averageh4_back;
int ap0, ap1, ap2, ap3, ap4, ah0, ah1, ah2, ah3, ah4;
int a0[10], a1[10], a2[10], a3[10], a4[10];
int median_calib_front[5];
int median_calib_back[5];
int dataADC_front[5];
int dataADC_back[5];
const int pin_sensor[] = {A0, A1, A2, A3, A4};
unsigned int biner[5];
unsigned int desimal_sensor;

void mode_sensor(int mode_sensor) {
  //1 Sensor Front 2 Sensor Back
  if (mode_sensor == 1) {
    digitalWrite(sw_line_depan, HIGH);
    digitalWrite(sw_line_belakang, LOW);
  }
  if (mode_sensor == 2) {
    digitalWrite(sw_line_depan, LOW);
    digitalWrite(sw_line_belakang, HIGH);
  }
  if (mode_sensor == 0) {
    digitalWrite(sw_line_depan, LOW);
    digitalWrite(sw_line_belakang, LOW);
  }

}
int ArraytoInt(unsigned int *x) {
  unsigned int Desimal;
  return Desimal = (x[4] * 16)  +
                   (x[3] * 8)   +
                   (x[2] * 4)   +
                   (x[1] * 2) +
                   (x[0] * 1);
}
void kalibrasi_putih() {
  while (calib_white == false) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("---Scan White---");
    lcd.setCursor(0, 1); lcd.print("A"); lcd.write(4);
    lcd.setCursor(7, 1); lcd.write(1);//lcd.print("B.");
    lcd.setCursor(14, 1); lcd.write(5); lcd.print("C");
    delay(200);
    if (digitalRead(swb) == LOW) {
      delay(250);
      menu_calibration();
    }
    if (digitalRead(swa) == LOW) {
      pil_sensor = 1;
      delay(500);
      calib_white = true;
    }
    if (digitalRead(swc) == LOW) {
      pil_sensor = 2;
      delay(500);
      calib_white = true;
    }
  }
  mode_sensor(pil_sensor);
  delay(2000);

  for (byte a = 0; a < 10; a++)
  {
    a0[a] = analogRead(A0);
    a1[a] = analogRead(A1);
    a2[a] = analogRead(A2);
    a3[a] = analogRead(A3);
    a4[a] = analogRead(A4);
    delay(20);
  }
  for (byte a = 0; a < 10; a++)
  {
    ap0 = ap0 + a0[a];
    ap1 = ap1 + a1[a];
    ap2 = ap2 + a2[a];
    ap3 = ap3 + a3[a];
    ap4 = ap4 + a4[a];
  }
  if (pil_sensor == 1) {
    averagep0_front = (averagep0_front + ap0) / 10;
    averagep1_front = (averagep1_front + ap1) / 10;
    averagep2_front = (averagep2_front + ap2) / 10;
    averagep3_front = (averagep3_front + ap3) / 10;
    averagep4_front = (averagep4_front + ap4) / 10;
  }
  if (pil_sensor == 2) {
    averagep0_back = (averagep0_back + ap0) / 10;
    averagep1_back = (averagep1_back + ap1) / 10;
    averagep2_back = (averagep2_back + ap2) / 10;
    averagep3_back = (averagep3_back + ap3) / 10;
    averagep4_back = (averagep4_back + ap4) / 10;
  }

  for (int a = 0; a < 16; a++) {
    lcd.setCursor(a, 1); lcd.print(".");
    delay(30);
  }
  mode_sensor(0);
  resetVariable_calib();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("---Scan White---");
  lcd.setCursor(6, 1); lcd.print("done");
  beep(300);
  delay(1000);
  menu_calibration();
}

void kalibrasi_hitam() {
  while (calib_black == false) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("---Scan Black---");
    lcd.setCursor(0, 1); lcd.print("A"); lcd.write(4);
    lcd.setCursor(7, 1); lcd.write(1);//lcd.print("B.");
    lcd.setCursor(14, 1); lcd.write(5); lcd.print("C");
    delay(200);
    if (digitalRead(swb) == LOW) {
      delay(250);
      menu_calibration();
    }
    if (digitalRead(swa) == LOW) {
      pil_sensor = 1;
      delay(500);
      calib_black = true;
    }
    if (digitalRead(swc) == LOW) {
      pil_sensor = 2;
      delay(500);
      calib_black = true;
    }
  }
  mode_sensor(pil_sensor);
  delay(2000);

  for (byte a = 0; a < 10; a++)
  {
    a0[a] = analogRead(A0);
    a1[a] = analogRead(A1);
    a2[a] = analogRead(A2);
    a3[a] = analogRead(A3);
    a4[a] = analogRead(A4);
    delay(20);
  }
  for (byte a = 0; a < 10; a++)
  {
    ah0 = ah0 + a0[a];
    ah1 = ah1 + a1[a];
    ah2 = ah2 + a2[a];
    ah3 = ah3 + a3[a];
    ah4 = ah4 + a4[a];

  }
  if (pil_sensor == 1) {
    averageh0_front = (averageh0_front + ah0) / 10;
    averageh1_front = (averageh1_front + ah1) / 10;
    averageh2_front = (averageh2_front + ah2) / 10;
    averageh3_front = (averageh3_front + ah3) / 10;
    averageh4_front = (averageh4_front + ah4) / 10;
  }
  if (pil_sensor == 2) {
    averageh0_back = (averageh0_back + ah0) / 10;
    averageh1_back = (averageh1_back + ah1) / 10;
    averageh2_back = (averageh2_back + ah2) / 10;
    averageh3_back = (averageh3_back + ah3) / 10;
    averageh4_back = (averageh4_back + ah4) / 10;
  }

  for (int a = 0; a < 16; a++) {
    lcd.setCursor(a, 1); lcd.print(".");
    delay(30);
  }
  mode_sensor(0);
  resetVariable_calib();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("---Scan Black---");
  lcd.setCursor(6, 1); lcd.print("done");
  beep(300);
  delay(1000);
  menu_calibration();
}
void resetVariable_calib() {
  for (byte a = 0; a < 10; a++)
  {
    a0[a] = 0;
    a1[a] = 0;
    a2[a] = 0;
    a3[a] = 0;
    a4[a] = 0;

    ah0 = ap0 = 0;
    ah1 = ap1 = 0;
    ah2 = ap2 = 0;
    ah3 = ap3 = 0;
    ah4 = ap4 = 0;
  }
}
/*
  void PV_Sensor(unsigned int x) {
  switch (x) {
    case 0b00000: pv = 0; break;
    case 0b11111:
      if (pv > 5) {
        pv = 7;
      }
      if (pv < -5) {
        pv = -7;
      }
      break;

    case 0b11110: pv = -7; break;
    case 0b11100: pv = -4;  break;
    case 0b11101: pv = -3; break;
    case 0b11001: pv = -2; break;
    case 0b11011: pv = 0;  break;
    case 0b10001: pv = 0;  break;
    case 0b10011: pv = 2;  break;
    case 0b10111: pv = 3;  break;
    case 0b00111: pv = 4;  break;
    case 0b01111: pv = 7;  break;

    case 0b11000:  pv=-7; break;
    case 0b00011:  pv =7; break;

  }
  }
*/
void PV_Sensor(unsigned int x) { //Maze Solving
  switch (x) {
    case 0b00000: pv = 0; mode_simpangan = 1; detect_tengah=false;break;
    case 0b11111:
      detect_tengah=false;
      mode_simpangan = 2;
      //if (pv > 5) {
      //  pv = 7;
      //}
      //if (pv < -5) {
      //  pv = -7;
      //}
      break;

    case 0b11110: pv = -7; detect_tengah=false;break;
    case 0b11100: pv = -4;  detect_tengah=false;break;
    case 0b11101: pv = -3; detect_tengah=false;break;
    case 0b11001: pv = -2; detect_tengah=false;break;
    case 0b11011: pv = 0;  detect_tengah=true;break;
    case 0b10001: pv = 0;  detect_tengah=false;break;
    case 0b10011: pv = 2;  detect_tengah=false;break;
    case 0b10111: pv = 3;  detect_tengah=false;break;
    case 0b00111: pv = 4;  detect_tengah=false;break;
    case 0b01111: pv = 7;  detect_tengah=false;break;

    case 0b11000: mode_simpangan = 4; break;
    case 0b00011: mode_simpangan = 3;  break;

  }
}
void read_sensor_front() {
  median_calib_front[0] = (averagep0_front + averageh0_front) / 2;
  median_calib_front[1] = (averagep1_front + averageh1_front) / 2;
  median_calib_front[2] = (averagep2_front + averageh2_front) / 2;
  median_calib_front[3] = (averagep3_front + averageh3_front) / 2;
  median_calib_front[4] = (averagep4_front + averageh4_front) / 2;

  for (int a = 0; a <= 4; a++) {
    dataADC_front[a] = analogRead(pin_sensor[a]);
  }
  for (int a = 0; a <= 4; a++) {
    if (dataADC_front[a] < median_calib_front[a])biner[a] = 1;
    else if (dataADC_front[a] > median_calib_front[a])biner[a] = 0;
  }
  desimal_sensor = ArraytoInt(biner);
  PV_Sensor(desimal_sensor);

}
void read_sensor_back() {
  median_calib_back[0] = (averagep0_back + averageh0_back) / 2;
  median_calib_back[1] = (averagep1_back + averageh1_back) / 2;
  median_calib_back[2] = (averagep2_back + averageh2_back) / 2;
  median_calib_back[3] = (averagep3_back + averageh3_back) / 2;
  median_calib_back[4] = (averagep4_back + averageh4_back) / 2;

  for (int a = 0; a <= 4; a++) {
    dataADC_back[a] = analogRead(pin_sensor[a]);
  }
  for (int a = 0; a <= 4; a++) {
    if (dataADC_back[a] < median_calib_back[a])biner[a] = 1;
    else if (dataADC_back[a] > median_calib_back[a])biner[a] = 0;
  }
  desimal_sensor = ArraytoInt(biner);
  PV_Sensor(desimal_sensor);

}
void display_setting_param(int index_param, int row, int col) {
  lcd.clear();
  lcd.setCursor(1, 0); lcd.print("v:"); lcd.print(ve, 0); lcd.setCursor(9, 0); lcd.print("i:"); lcd.print(ki, 1);
  lcd.setCursor(1, 1); lcd.print("p:"); lcd.print(kp, 1); lcd.setCursor(9, 1); lcd.print("d:"); lcd.print(kd, 1);
  lcd.setCursor(15, 1); lcd.print("B");
  if (index_param <= 4) {
    lcd.setCursor(row, col); lcd.write(2);
  }
  if (index_param == 5) {
    lcd.setCursor(row, col); lcd.write(5);
  }
  delay(100);

}
void setting_param() {
  int row, col;
  int index_param = 1;
  bool set = false;
  while (1) {
    display_setting_param(index_param, row, col);

    if (digitalRead(swa) == LOW) {
      index_param -= 1;
    }
    while (digitalRead(swa) == LOW);

    if (digitalRead(swc) == LOW) {
      index_param += 1;
    }
    while (digitalRead(swc) == LOW);

    if (digitalRead(swb) == LOW) {
      set = false;
      beep(200);
      while (set == false) {
        display_setting_param(index_param, row, col);
        if (digitalRead(swa) == LOW) {
          if (index_param == 1) {
            ve += 5;
          }
          if (index_param == 2) {
            kp += 0.2;
          }
          if (index_param == 3) {
            ki += 0.2;
          }
          if (index_param == 4) {
            kd += 0.2;
          }
        }
        while (digitalRead(swa) == LOW);

        if (digitalRead(swc) == LOW) {
          if (index_param == 1) {
            ve -= 5;
          }
          if (index_param == 2) {
            kp -= 0.2;
          }
          if (index_param == 3) {
            ki -= 0.2;
          }
          if (index_param == 4) {
            kd -= 0.2;
          }
        }
        while (digitalRead(swc) == LOW);

        if (digitalRead(swb) == LOW) {
          if (index_param <= 4) {
            beep(200);
            set = true;
          }
          if (index_param == 5) {
            beep(200);
            menu_calibration();
          }

        }
      }
    }
    
    if (index_param <= 0) {
      index_param = 1;
    }
    if (index_param >= 6) {
      index_param = 5;
    }

    if (index_param == 1) {
      row = 0; col = 0;
    }
    if (index_param == 2) {
      row = 0; col = 1;
    }
    if (index_param == 3) {
      row = 8; col = 0;
    }
    if (index_param == 4) {
      row = 8; col = 1;
    }
    if (index_param == 5) {
      row = 15; col = 0;
    }



    delay(100);
  }
}

///////////////////////////////////////////////////////////
void menu_read_sensor() {
  pil_sensor = 0;
  while (1) {

    lcd.clear();
    lcd.setCursor(3, 0); lcd.print(biner[4]);
    lcd.setCursor(5, 0); lcd.print(biner[3]);
    lcd.setCursor(7, 0); lcd.print(biner[2]);
    lcd.setCursor(9, 0); lcd.print(biner[1]);
    lcd.setCursor(11, 0); lcd.print(biner[0]);
    lcd.setCursor(0, 1); lcd.print("A"); lcd.write(4); lcd.print(" "); lcd.write(5); lcd.print("C");
    lcd.setCursor(7, 1); lcd.write(1);
    lcd.setCursor(9, 1); lcd.print("pv:"); lcd.print(pv, 1);
    delay(100);
    mode_sensor(pil_sensor);

    if (digitalRead(swa) == LOW) {
      pil_sensor = 1;
    }
    if (digitalRead(swc) == LOW) {
      pil_sensor = 2;
    }
    if (pil_sensor == 1) {
      read_sensor_front();
    }
    if (pil_sensor == 2) {
      read_sensor_back();
    }

    if (digitalRead(swb) == LOW) {
      delay(250);
      mode_sensor(0);
      menu_select();
    }
  }
}
void menu_motor_test() {
  int pwm_left = 0, pwm_right = 0;
  while (1) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(pwm_left);
    lcd.setCursor(6, 0); lcd.print("-.-");
    lcd.setCursor(13, 0); lcd.print(pwm_right);
    lcd.setCursor(0, 1); lcd.print("M1");
    lcd.setCursor(7, 1); lcd.write(1);
    lcd.setCursor(14, 1); lcd.print("M2");
    delay(100);
    drive_motor(1, 0, pwm_left, pwm_right);
    if (digitalRead(swa) == LOW) {//Button Left
      pwm_left += 5;
      beep(100);
    }
    while (digitalRead(swa) == LOW);
    if (digitalRead(swc) == LOW) {//Button Right
      pwm_right += 5;
      beep(100);
    }
    while (digitalRead(swc) == LOW);

    if (pwm_left >= 255)pwm_left = 255;
    if (pwm_right >= 255)pwm_right = 255;

    if (digitalRead(swb) == LOW) {//Button Right
      delay(250);
      menu_select();
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_path_planning() {
  while (1) {
    if (digitalRead(swb) == LOW) {
      delay(250);
      menu_select();
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
const char menu_race_name[] PROGMEM = "--Racing Plane--";
const char menu_maze_name[] PROGMEM = "--Maze Solving--";
const char menu_back_running[] PROGMEM = "------Back------";

void menu_race();
void menu_maze();
void back_running();
typedef void(*function)();
const function main_menu_functions_running[] = {menu_race, menu_maze, back_running};
const char *const main_menu_options_running[] PROGMEM = {menu_race_name, menu_maze_name, menu_back_running};

void menu_running_robot() {
  while (1) {
    strcpy_P(buffer, (char*)pgm_read_word(&(main_menu_options_running[index_running])));
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(buffer);
    lcd.setCursor(0, 1); lcd.write(3);
    lcd.setCursor(7, 1); lcd.print("B.");//lcd.print("B.");
    lcd.setCursor(15, 1); lcd.write(2);
    delay(100);

    if (digitalRead(swa) == LOW) {//Button Left
      index_running -= 1;
      beep(100);
    }
    while (digitalRead(swa) == LOW);
    if (digitalRead(swc) == LOW) {//Button Right
      index_running += 1;
      beep(100);
    }
    while (digitalRead(swa) == LOW);
    if (digitalRead(swb) == LOW) { //Button Ok
      delay(250);
      main_menu_functions_running[index_running]();
    }
    while (digitalRead(swb) == LOW);

    if (index_calib >= 2) {
      index_calib = 2;
    }
    if (index_calib <= 0) {
      index_calib = 0;
    }
  }
}
void back_running() {
  menu_select();
}
void running_pid(int mode_running, int sensor_mode) {
  /*
        kp = 10;
        kd = 8;
        ki = 4;
        P besar, overshoot besar
        D besar, memperbaiki eror
        I besar, memperbaiki overshoot
        //Forward
        Fast
        v=150,p=14.8,i=7.4,d=10.8
        vel_left=+,vel_right-
        Slow
        v=95,p=8.8,i=4.2,d=6.6
        vel_left=+,vel_right-
        //Backward
        Fast
        v=150,p=14.8,i=7.2,d=10.6
        vel_left=-,vel_right+
        Slow
        v=95,p=8.8,i=4.2,d=6.6
        vel_left=-,vel_right+
  */
  if (mode_running == 0 || sensor_mode == 0) {
    mode_sensor(0);
    vel_left = vel_right = 0;
    drive_motor(0, 0, vel_left, vel_right);
  }
  if (mode_running == 1 && sensor_mode == 1) { //Race Mode
    mode_sensor(1);
    read_sensor_front();
    sp = 0;
    ts = 0.016;
    error = sp - pv;
    //Kontrol Propotional
    P = kp * error;
    //Kontrol Integral
    I = ki * ((error + error_previous) * ts / 2)  ;
    //Kontrol Derivative
    D = (kd * 1) * ((error - error_previous) / ts);
    PD = P + D;
    PID = PD + I;
    vel_left = ((int)ve + 0) + PID;
    vel_right = ((int)ve + 0) - PID;
    drive_motor(1, 1, vel_left, vel_right);
    if (vel_left >= 255) {
      vel_left = 250;
    }
    if (vel_left < 0) {
      vel_left = 0;
    }
    if (vel_right >= 255) {
      vel_right = 250;
    }
    if (vel_right < 0) {
      vel_right = 0;
    }
    error_previous = error;

  }

  if (mode_running == 1 && sensor_mode == 2) { //Race Mode
    mode_sensor(2);
    read_sensor_back();
    sp = 0;
    ts = 0.016;
    error = sp - pv;
    //Kontrol Propotional
    P = kp * error;
    //Kontrol Integral
    I = ki * ((error + error_previous) * ts / 2)  ;
    //Kontrol Derivative
    D = (kd * 1) * ((error - error_previous) / ts);
    PD = P + D;
    PID = PD + I;
    vel_left = ((int)ve + 0) - PID;
    vel_right = ((int)ve + 0) + PID;
    drive_motor(0, 0, vel_left, vel_right);
    if (vel_left >= 255) {
      vel_left = 250;
    }
    if (vel_left < 0) {
      vel_left = 0;
    }
    if (vel_right >= 255) {
      vel_right = 250;
    }
    if (vel_right < 0) {
      vel_right = 0;
    }
    error_previous = error;
  }


}
void menu_race() {
  bool race_running = false;
  int b = 0;
  beep(200);
  while (race_running == false) {
    lcd.clear();
    lcd.setCursor(1, 0); lcd.print("v:"); lcd.print(ve, 0); lcd.setCursor(9, 0); lcd.print("i:"); lcd.print(ki, 1);
    lcd.setCursor(1, 1); lcd.print("p:"); lcd.print(kp, 1); lcd.setCursor(9, 1); lcd.print("d:"); lcd.print(kd, 1);
    lcd.setCursor(15, 0); lcd.write(5); lcd.setCursor(15, 1); lcd.print("B");
    lcd.setCursor(0, 0); lcd.write(5); lcd.setCursor(0, 1); lcd.print("C");
    delay(100);
    if (digitalRead(swb) == LOW) {
      beep(200);
      menu_select();
    }
    if (digitalRead(swa) == LOW || digitalRead(swc) == LOW  ) {
      if (digitalRead(swa) == LOW) {
        b = 1;
      }
      if (digitalRead(swc) == LOW) {
        b = 2;
      }
      lcd.clear();
      lcd.setCursor(5, 0); lcd.print("ready!");
      for (int a = 3; a > 0; a--) {
        lcd.setCursor(7, 1); lcd.print(a);
        beep(500);
      }
      lcd.setCursor(7, 1); lcd.print("GO");
      race_running = true;
    }
  }
  while (race_running == true) {
    running_pid(1, b);
    if (digitalRead(swb) == LOW) {
      delay(250);
      mode_sensor(0);
      drive_motor(1, 1, 0, 0);
      menu_select();
    }

  }
}
void menu_maze() {
  unsigned long time, time_previous = 0;
  int mode_maze = 1;
  int counter = 0;
  int flag=0;
  bool maze_running=false;
  beep(200);
  while(maze_running==false){
    counter=flag=mode_simpangan=0;
    lcd.clear();
    lcd.setCursor(1,0);lcd.print("Manor Maze Solv");
    lcd.setCursor(5,1);lcd.print("Ready!");
    delay(200);
    if(digitalRead(swa)==LOW||digitalRead(swc)==LOW){
      counter=flag=mode_simpangan=0;
      lcd.clear();
      lcd.setCursor(5, 0); lcd.print("ready!");
      for (int a = 3; a > 0; a--) {
        lcd.setCursor(7, 1); lcd.print(a);
        beep(500);
      }
      lcd.setCursor(7, 1); lcd.print("GO");
      maze_running = true;
    }
  }
  while (maze_running==true) {
awal:
    time = millis();
    if (mode_maze == 1) {
      running_pid(1, 1);
    }
    if (mode_maze == 2) {
      running_pid(1, 2);
    }
    if (digitalRead(swb) == LOW) {
      counter=flag=mode_simpangan=0;
      delay(250);
      drive_motor(0, 0, 0, 0);
      mode_sensor(0);
      menu_select();
    }

    if (time - time_previous > 200) {//Putih Semua
      if (mode_simpangan == 2 && flag==0) {
        if(counter==1){
          mode_maze=2;
          turn_beep(2,100);
          running_pid(1,2);
        }
        if(counter==3){
          mode_maze=1;
          turn_beep(1,100);
          running_pid(1,1);
        }
        counter += 1;
        flag=1;
        mode_simpangan = 0;
        goto awal;
      }
      time_previous = time;
    }
    if (mode_simpangan == 2) { //Hitam Semua
      ;
    }

    if (mode_simpangan == 4) {//Simpangan Kanan
      if(counter==0){
        turn_right(1, 90);
      }
      flag=0;
      counter += 1;
      mode_simpangan = 0;
      goto awal;
    }
    if (mode_simpangan == 3) {//Simpangan Kiri
      if(counter==2){
        turn_left(2 , 90);
      }
      flag=0;
      counter+=1;
      mode_simpangan = 0;
      goto awal;
    }
  }
}

//Subrutin LCD MENU
void inisialisasi() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("-Initialization-");
  for (int a = 0; a < 16; a++) {
    lcd.setCursor(a, 1); lcd.print(".");
    delay(30);
  }
  beep(300);
  mode_sensor(0);

  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("--Manor Racing--");
  lcd.setCursor(5, 1); lcd.print("Ready!");
  delay(2500);

}
void beep(int tim) {
  digitalWrite(buzz, HIGH);
  delay(tim);
  digitalWrite(buzz, LOW);
  delay(tim / 2);

}

void blink_sensor(bool sensor_front, bool sensor_back, int loop, int tim) {
  for (int a = 0; a <= loop; a++) {
    if (sensor_front == true) {
      digitalWrite(sw_line_depan, HIGH);
      delay(tim);
      digitalWrite(sw_line_belakang, LOW);
      delay(tim / 2);
    }
    if (sensor_back == true) {
      digitalWrite(sw_line_belakang, HIGH);
      delay(tim);
      digitalWrite(sw_line_depan, LOW);
      delay(tim / 2);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
void drive_motor(int logic_left, int logic_right, int pwm_left, int pwm_right) {
  /*
    1 Forward, 0 Backward
    Velocity =80;
    Keterangan
    Motor Kiri, Forward LOW Reverse HIGH
    Motor Kanan, Forward HIGH Reverse LOW

    Motor Kanan+10;

  */
  if (logic_left == 1) {//Forward
    digitalWrite(in2, LOW);
  }
  if (logic_left == 0 || logic_left == 2) { //Backward
    digitalWrite(in2, HIGH);
  }

  analogWrite(en2, pwm_left);

  if (logic_right == 1) {//Forward
    digitalWrite(in1, HIGH);
  }
  if (logic_right  == 0 || logic_left == 2) { //Backward
    digitalWrite(in1, LOW);
  }

  analogWrite(en1, pwm_right);

}

void turn_left(int mode_sensor, int heading) {
  int tim = 0,step=0,sensor=0;
  if (heading == 90) {
    tim = 500;
  }
  if (heading == 180) {
    tim = 1000;
  }
  if (mode_sensor == 1 || mode_sensor == 2) {
    if(mode_sensor==1){
      sensor=1;
    }
    if(mode_sensor==2){
      sensor=2;
    }
    if(step==0){
      drive_motor(0, 0, 0, 0);
      beep(500);
      drive_motor(mode_sensor, mode_sensor, 65, 65);
      delay(150);
      drive_motor(0, 0, 0, 0);
      delay(500);
      step=1;  
    }
    if(step==1){
      if(sensor==1){
        read_sensor_front();  
      }
      if(sensor==2){
        read_sensor_back();    
      }
      while(detect_tengah!=true){  
        if(sensor==1){
          read_sensor_front();  
        }
        if(sensor==2){
          read_sensor_back();    
        }
        drive_motor(0, 1, 50, 50);
      }
      drive_motor(0, 0, 0, 0);
      delay(750);
      step=2;
    }
  }
}


void turn_right(int mode_sensor, int heading) {
  int tim = 0,step=0,sensor=0;
  if (heading == 90) {
    tim = 500;
  }
  if (heading == 180) {
    tim = 1000;
  }
  if (mode_sensor == 1 || mode_sensor == 2) {
    if(mode_sensor==1){
      sensor=1;
    }
    if(mode_sensor==2){
      sensor=2;
    }
    if(step==0){
      drive_motor(0, 0, 0, 0);
      beep(500);
      drive_motor(mode_sensor, mode_sensor, 65, 65);
      delay(150);
      drive_motor(0, 0, 0, 0);
      delay(500);
      step=1;  
    }
    if(step==1){
      if(sensor==1){
        read_sensor_front();  
      }
      if(sensor==2){
        read_sensor_back();    
      }
      while(detect_tengah!=true){  
        if(sensor==1){
          read_sensor_front();  
        }
        if(sensor==2){
          read_sensor_back();    
        }
        drive_motor(1, 0, 50, 50);
      }
      drive_motor(0, 0, 0, 0);
      delay(750);
      step=2;
    }
  }
}
/*
void turn_left(int mode_sensor, int heading) {
  int tim = 0;
  if (heading == 90) {
    tim = 500;
  }
  if (heading == 180) {
    tim = 1000;
  }
  if (mode_sensor == 1 || mode_sensor == 2) {
    drive_motor(0, 0, 0, 0);
    beep(500);
    drive_motor(mode_sensor, mode_sensor, 65, 65);
    delay(220);
    drive_motor(0, 0, 0, 0);
    delay(500);
    drive_motor(0, 1, 60, 60);
    delay(tim);
    drive_motor(0, 0, 0, 0);
    delay(750);
  }

}

void turn_right(int mode_sensor, int heading) {
  int tim = 0;
  if (heading == 90) {
    tim = 500;
  }
  if (heading == 180) {
    tim = 1000;
  }
  if (mode_sensor == 1 || mode_sensor == 2) {
    drive_motor(0, 0, 0, 0);
    beep(500);
    drive_motor(mode_sensor, mode_sensor, 65, 65);
    delay(220);
    drive_motor(0, 0, 0, 0);
    delay(500);
    drive_motor(1, 0, 60, 60);
    delay(tim);
    drive_motor(0, 0, 0, 0);
    delay(750);

  }

}
*/
void turn_beep(int mode, int time_delay) {
  drive_motor(0, 0, 0, 0);
  beep(time_delay);
  drive_motor(mode, mode, 65, 65);
  delay(time_delay);
  drive_motor(0, 0, 0, 0);
  delay(500);
  
}

byte back_arrow[8] = {
  0b00000,
  0b00010,
  0b00001,
  0b00101,
  0b01001,
  0b11110,
  0b01000,
  0b00100,
};

byte love[8] {
  0b00000,
  0b00000,
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
};
byte note[8] = {
  0b00100,
  0b00110,
  0b00101,
  0b00101,
  0b00100,
  0b11100,
  0b11100,
  0b00000,
};
byte selectedR[8] {
  0b00000,
  0b01000,
  0b01100,
  0b01110,
  0b01111,
  0b01110,
  0b01100,
  0b01000,
};
byte selectedL[8] {
  0b00000,
  0b00010,
  0b00110,
  0b01110,
  0b11110,
  0b01110,
  0b00110,
  0b00010,
};

byte blink1[8] {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
};
byte blink2[8] {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
};
byte arrow_up[8] {
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
};
byte arrow_down[8] {
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b10101,
  0b01110,
  0b00100,
};

void setup() {
  //Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.createChar(1, back_arrow);
  lcd.createChar(2, selectedR);
  lcd.createChar(3, selectedL);
  lcd.createChar(4, arrow_up);
  lcd.createChar(5, arrow_down);
  lcd.createChar(7, love);

  //OUTPUT
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(buzz, OUTPUT);
  pinMode(sw_line_depan, OUTPUT);
  pinMode(sw_line_belakang, OUTPUT);
  //Digital Input
  pinMode(swa, INPUT_PULLUP);
  pinMode(swb, INPUT_PULLUP);
  pinMode(swc, INPUT_PULLUP);
  //Line Sensor
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

  inisialisasi();
  //Rotasi
  //RKanan-kiri 90deg 60 60 d:500
  //RKanan-kiri 90deg 60 60 d:1000

}

void loop() {
  menu_select();
}
