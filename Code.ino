#include <afstandssensor.h>  //Ultrasonic sensor library
#include <AceRoutine.h>      //Multitasking library
#include <SPI.h>             //SPI mode library
#include <SD.h>              //SD library
#include <SoftwareSerial.h>  //Bluetooth Serial

using namespace ace_routine;

File myMap;
//CS pin
const int chipSelect = 53;

//Setup of the ultrasonic sensor pin. The first is the trigger and the second is the echo
AfstandsSensor afstandssensor2(A2, A3);

SoftwareSerial BTserial(11, 10);  // RX | TX

//Setup of the stepper motor pin
const int mr1 = 4;
const int mr2 = 5;
const int mr3 = 6;
const int mr4 = 7;
const int ml1 = A7;
const int ml2 = A6;
const int ml3 = A5;
const int ml4 = A4;

//Setup of shock detector (buttons)
const int fbutton = A0;  //first button (left)
const int sbutton = A1;  //second button (right)

int motordelay = 3;     //Milliseconds of delay between motor's coils activation
int counter = 0;        //Counter that in the coroutine follow_wall, turns off the controller until the left turn is completed
int counter2 = 0;       //Variable that counts the number of times the coroutine sd is performed
int x = 0;              //Variable to manage the movement on rows
int y = 0;              //Variable to manage movement on columns
int z = 0;              //Variable containing the modes of movement (up, down, right, left)
int Data;               //Variable containing an input value
int isObstaclePin = 8;  //Set up of the ir sensor pin
int isObstacle = HIGH;

float distance;  //Left ultrasonic sensor's output

bool bwall = true;        //Boolean variable that activates the zigzag coroutine
bool bcontroller = true;  //Boolean variable that activates the controller coroutine
bool bshock = true;       //Boolean variable that activates the shock coroutine
bool bsd = true;          //Boolean variable that activates the Sd coroutine
bool bright = false;      //Boolean variable that activates the right coroutine
bool bleft = false;       //Boolean variable that activates the left coroutine
bool bforward = false;    //Boolean variable that activates the forward coroutine
bool bcopyx = false;      //Boolean variable that activates the copy_back_x coroutine
bool bcopyy = false;      //Boolean variable that activates the copy_back_y coroutine
bool bforwardl = false;   //Boolean variable that activates the forward_left coroutine
bool bforwardr = false;   //Boolean variable that activates the forward_right coroutine

//Matrix containing the map
char matrix[70][70];
char convert;

//Coroutine that allows to follow a perimeter and manage the controller
COROUTINE(follow_wall) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bwall);  //Yields until condition become true
    COROUTINE_DELAY(2000);
    isObstacle = digitalRead(isObstaclePin); //Reading the status of the infrared sensor
    distance = afstandssensor2.afstandCM();  //Lateral distance from wall in cm
    if (isObstacle == LOW) {
      bright = true;
    } else {
      if (distance > 7 && distance < 8.5) {
        bforward = true;
        bcontroller = true;
      } else {
        if (distance < 7) {
          bforwardr = true;
          bcontroller = true;
        } else {
          if (distance > 8.5 && distance < 9.5) {
            bforwardl = true;
            bcontroller = true;
          } else {
            if (distance > 9.5 && distance < 16) {
              bcontroller = false;
              for (int i = 0; i < 680; i++) {
                digitalWrite(ml1, HIGH);
                digitalWrite(mr1, LOW);
                digitalWrite(ml2, LOW);
                digitalWrite(mr2, LOW);
                digitalWrite(ml3, LOW);
                digitalWrite(mr3, LOW);
                digitalWrite(ml4, LOW);
                digitalWrite(mr4, HIGH);
                delay(motordelay);
                digitalWrite(ml1, LOW);
                digitalWrite(mr1, LOW);
                digitalWrite(ml2, HIGH);
                digitalWrite(mr2, LOW);
                digitalWrite(ml3, LOW);
                digitalWrite(mr3, HIGH);
                digitalWrite(ml4, LOW);
                digitalWrite(mr4, LOW);
                delay(motordelay);
                digitalWrite(ml1, LOW);
                digitalWrite(mr1, LOW);
                digitalWrite(ml2, LOW);
                digitalWrite(mr2, HIGH);
                digitalWrite(ml3, HIGH);
                digitalWrite(mr3, LOW);
                digitalWrite(ml4, LOW);
                digitalWrite(mr4, LOW);
                delay(motordelay);
                digitalWrite(ml1, LOW);
                digitalWrite(mr1, HIGH);
                digitalWrite(ml2, LOW);
                digitalWrite(mr2, LOW);
                digitalWrite(ml3, LOW);
                digitalWrite(mr3, LOW);
                digitalWrite(ml4, HIGH);
                digitalWrite(mr4, LOW);
                delay(motordelay);
              }
              bforward = true;
              COROUTINE_DELAY(1000);
              for (int i = 0; i < 620; i++) {
                digitalWrite(mr1, HIGH);
                digitalWrite(ml1, LOW);
                digitalWrite(mr2, LOW);
                digitalWrite(ml2, LOW);
                digitalWrite(mr3, LOW);
                digitalWrite(ml3, LOW);
                digitalWrite(mr4, LOW);
                digitalWrite(ml4, HIGH);
                delay(motordelay);
                digitalWrite(mr1, LOW);
                digitalWrite(ml1, LOW);
                digitalWrite(mr2, HIGH);
                digitalWrite(ml2, LOW);
                digitalWrite(mr3, LOW);
                digitalWrite(ml3, HIGH);
                digitalWrite(mr4, LOW);
                digitalWrite(ml4, LOW);
                delay(motordelay);
                digitalWrite(mr1, LOW);
                digitalWrite(ml1, LOW);
                digitalWrite(mr2, LOW);
                digitalWrite(ml2, HIGH);
                digitalWrite(mr3, HIGH);
                digitalWrite(ml3, LOW);
                digitalWrite(mr4, LOW);
                digitalWrite(ml4, LOW);
                delay(motordelay);
                digitalWrite(mr1, LOW);
                digitalWrite(ml1, HIGH);
                digitalWrite(mr2, LOW);
                digitalWrite(ml2, LOW);
                digitalWrite(mr3, LOW);
                digitalWrite(ml3, LOW);
                digitalWrite(mr4, HIGH);
                digitalWrite(ml4, LOW);
                delay(motordelay);
              }
              bcontroller = true;
            } else {
              if (distance > 16 && counter < 2) {
                counter++;
                if (counter == 2) {
                  bcontroller = false;
                }
                bforward = true;
                COROUTINE_DELAY(10000);
                bleft = true;
              }
            }
          }
        }
      }
    }
    COROUTINE_YIELD();  //Yields execution back to the caller
  }
}

//Coroutine that every 20 cm records the characters on the map depending on the mode set.
COROUTINE(controller) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bcontroller);
    COROUTINE_DELAY(9000);
    switch (z) {
      case 0:
        y++;
        matrix[x][y] = 'W';
        break;
      case 1:
        x++;
        matrix[x][y] = 'W';
        break;
      case 2:
        y--;
        if (y == -1) {
          y = 0;
          bcopyy = true;
        } else {
          matrix[x][y] = 'W';
        }
        break;
      case 3:
        x--;
        if (x == -1) {
          x = 0;
          bcopyx = true;
        } else {
          matrix[x][y] = 'W';
        }
        break;
      default:
        break;
    }
    COROUTINE_YIELD();
  }
}

//Coroutine moving all matrix down one row
COROUTINE(copy_back_x) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bcopyx);
    for (int i = 69; i >= 0; i--) {
      for (int h = 0; h < 70; h++) {
        if (i != 0) {
          matrix[i][h] = matrix[i - 1][h];
        } else {
          matrix[i][h] = ' ';
        }
      }
    }
    matrix[x][y] = 'W';
    bcopyx = false;
    COROUTINE_YIELD();
  }
}

//Coroutine moving the entire matrix of a column to the right
COROUTINE(copy_back_y) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bcopyy);
    for (int i = 0; i < 70; i++) {
      for (int h = 69; h >= 0; h--) {
        if (h != 0) {
          matrix[i][h] = matrix[i][h - 1];
        } else {
          matrix[i][h] = ' ';
        }
      }
    }
    matrix[x][y] = 'W';
    bcopyy = false;
    COROUTINE_YIELD();
  }
}

COROUTINE(forward) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bforward);
    bforwardr = false;
    bforwardl = false;
    //Code to turn the wheels and make the device go forward
    //----------------------------------
    digitalWrite(ml1, LOW);
    digitalWrite(mr1, LOW);
    digitalWrite(ml2, LOW);
    digitalWrite(mr2, LOW);
    digitalWrite(ml3, LOW);
    digitalWrite(mr3, LOW);
    digitalWrite(ml4, HIGH);
    digitalWrite(mr4, HIGH);
    delay(motordelay);
    digitalWrite(ml1, LOW);
    digitalWrite(mr1, LOW);
    digitalWrite(ml2, LOW);
    digitalWrite(mr2, LOW);
    digitalWrite(ml3, HIGH);
    digitalWrite(mr3, HIGH);
    digitalWrite(ml4, LOW);
    digitalWrite(mr4, LOW);
    delay(motordelay);
    digitalWrite(ml1, LOW);
    digitalWrite(mr1, LOW);
    digitalWrite(ml2, HIGH);
    digitalWrite(mr2, HIGH);
    digitalWrite(ml3, LOW);
    digitalWrite(mr3, LOW);
    digitalWrite(ml4, LOW);
    digitalWrite(mr4, LOW);
    delay(motordelay);
    digitalWrite(ml1, HIGH);
    digitalWrite(mr1, HIGH);
    digitalWrite(ml2, LOW);
    digitalWrite(mr2, LOW);
    digitalWrite(ml3, LOW);
    digitalWrite(mr3, LOW);
    digitalWrite(ml4, LOW);
    digitalWrite(mr4, LOW);
    delay(motordelay);
    COROUTINE_YIELD();
    //----------------------------------
  }
}

//Coroutines that allows to correct the direction of the robot when it is away from the wall
COROUTINE(forward_left) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bforwardl);
    bforward = false;
    bforwardr = false;
    digitalWrite(mr1, LOW);
    digitalWrite(mr2, LOW);
    digitalWrite(mr3, LOW);
    digitalWrite(mr4, HIGH);
    delay(motordelay);
    digitalWrite(mr1, LOW);
    digitalWrite(mr2, LOW);
    digitalWrite(mr3, HIGH);
    digitalWrite(mr4, LOW);
    delay(motordelay);
    digitalWrite(mr1, LOW);
    digitalWrite(mr2, HIGH);
    digitalWrite(mr3, LOW);
    digitalWrite(mr4, LOW);
    delay(motordelay);
    digitalWrite(mr1, HIGH);
    digitalWrite(mr2, LOW);
    digitalWrite(mr3, LOW);
    digitalWrite(mr4, LOW);
    delay(motordelay);
    COROUTINE_YIELD();
  }
}

//Coroutines that allows to correct the direction of the robot when it is too close to the wall
COROUTINE(forward_right) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bforwardr);
    bforward = false;
    bforwardl = false;
    digitalWrite(ml1, LOW);
    digitalWrite(ml2, LOW);
    digitalWrite(ml3, LOW);
    digitalWrite(ml4, HIGH);
    delay(motordelay);
    digitalWrite(ml1, LOW);
    digitalWrite(ml2, LOW);
    digitalWrite(ml3, HIGH);
    digitalWrite(ml4, LOW);
    delay(motordelay);
    digitalWrite(ml1, LOW);
    digitalWrite(ml2, HIGH);
    digitalWrite(ml3, LOW);
    digitalWrite(ml4, LOW);
    delay(motordelay);
    digitalWrite(ml1, HIGH);
    digitalWrite(ml2, LOW);
    digitalWrite(ml3, LOW);
    digitalWrite(ml4, LOW);
    delay(motordelay);
    COROUTINE_YIELD();
  }
}

COROUTINE(right) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bright);
    bcontroller = false;
    bforwardr = false;
    bforwardl = false;
    bwall = false;
    bshock = false;
    bforward = false;
    for (int i = 0; i < 680; i++) {
      //Code to turn the wheels and make the device go right
      //----------------------------------
      digitalWrite(mr1, HIGH);
      digitalWrite(ml1, LOW);
      digitalWrite(mr2, LOW);
      digitalWrite(ml2, LOW);
      digitalWrite(mr3, LOW);
      digitalWrite(ml3, LOW);
      digitalWrite(mr4, LOW);
      digitalWrite(ml4, HIGH);
      delay(motordelay);
      digitalWrite(mr1, LOW);
      digitalWrite(ml1, LOW);
      digitalWrite(mr2, HIGH);
      digitalWrite(ml2, LOW);
      digitalWrite(mr3, LOW);
      digitalWrite(ml3, HIGH);
      digitalWrite(mr4, LOW);
      digitalWrite(ml4, LOW);
      delay(motordelay);
      digitalWrite(mr1, LOW);
      digitalWrite(ml1, LOW);
      digitalWrite(mr2, LOW);
      digitalWrite(ml2, HIGH);
      digitalWrite(mr3, HIGH);
      digitalWrite(ml3, LOW);
      digitalWrite(mr4, LOW);
      digitalWrite(ml4, LOW);
      delay(motordelay);
      digitalWrite(mr1, LOW);
      digitalWrite(ml1, HIGH);
      digitalWrite(mr2, LOW);
      digitalWrite(ml2, LOW);
      digitalWrite(mr3, LOW);
      digitalWrite(ml3, LOW);
      digitalWrite(mr4, HIGH);
      digitalWrite(ml4, LOW);
      delay(motordelay);
      //----------------------------------
    }
    if (counter != 2) {
      bcontroller = true;
      z++;
      if (z == 4) {
        z = 0;
      }
    } else {
      counter = 0;
    }
    bright = false;
    bwall = true;
    bshock = true;
    COROUTINE_YIELD();
  }
}

COROUTINE(left) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bleft);
    bforward = false;
    bcontroller = false;
    bwall = false;
    bshock = false;
    bforwardr = false;
    bforwardl = false;
    for (int i = 0; i < 680; i++) {
      //Code to turn the wheels and make the device go left
      //----------------------------------
      digitalWrite(ml1, HIGH);
      digitalWrite(mr1, LOW);
      digitalWrite(ml2, LOW);
      digitalWrite(mr2, LOW);
      digitalWrite(ml3, LOW);
      digitalWrite(mr3, LOW);
      digitalWrite(ml4, LOW);
      digitalWrite(mr4, HIGH);
      delay(motordelay);
      digitalWrite(ml1, LOW);
      digitalWrite(mr1, LOW);
      digitalWrite(ml2, HIGH);
      digitalWrite(mr2, LOW);
      digitalWrite(ml3, LOW);
      digitalWrite(mr3, HIGH);
      digitalWrite(ml4, LOW);
      digitalWrite(mr4, LOW);
      delay(motordelay);
      digitalWrite(ml1, LOW);
      digitalWrite(mr1, LOW);
      digitalWrite(ml2, LOW);
      digitalWrite(mr2, HIGH);
      digitalWrite(ml3, HIGH);
      digitalWrite(mr3, LOW);
      digitalWrite(ml4, LOW);
      digitalWrite(mr4, LOW);
      delay(motordelay);
      digitalWrite(ml1, LOW);
      digitalWrite(mr1, HIGH);
      digitalWrite(ml2, LOW);
      digitalWrite(mr2, LOW);
      digitalWrite(ml3, LOW);
      digitalWrite(mr3, LOW);
      digitalWrite(ml4, HIGH);
      digitalWrite(mr4, LOW);
      delay(motordelay);
      //----------------------------------
    }
    if (counter == 1) {
      z--;
      if (z == -1) {
        z = 3;
      }
      bcontroller = true;
    }
    bleft = false;
    bforward = true;
    bwall = true;
    bshock = true;
    COROUTINE_YIELD();
  }
}

//Coroutine that allows you to save the map on sd card or to save the map and to show it on the terminal
COROUTINE(Sd_bluetooth) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bsd);
    if (BTserial.available()) {  //wait for data received
      Data = BTserial.read();
      if (Data == '0') {
        if (counter2 != 0) {
          SD.remove("Mappa.txt");
        }
        counter2 = 1;
        myMap = SD.open("Mappa.txt", FILE_WRITE);
        if (myMap) {
          for (int h = 0; h < 70; h++) {
            for (int i = 0; i < 70; i++) {
              myMap.print(matrix[h][i]);
            }
            myMap.println(" ");
          }
        }
        myMap.close();
        BTserial.println("Map completed");
      } else {
        if (Data == '1') {
          if (counter2 != 0) {
            SD.remove("Mappa.txt");
          }
          counter2 = 1;
          myMap = SD.open("Mappa.txt", FILE_WRITE);
          if (myMap) {
            for (int h = 0; h < 70; h++) {
              for (int i = 0; i < 70; i++) {
                myMap.print(matrix[h][i]);
              }
              myMap.println(" ");
            }
            myMap.close();
          }
          myMap = SD.open("Mappa.txt", FILE_READ);
          if (myMap) {
            while (myMap.available()) {
              convert = myMap.read();
              BTserial.print(convert);
            }
            myMap.close();
          }
        }
      }
    }
    COROUTINE_YIELD();
  }
}

//Coroutine that allows to avoid obstacles along the perimeter
COROUTINE(shock) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(bshock);
    COROUTINE_DELAY(200);
    //Reading the status of the buttons on the left
    int fbs = digitalRead(fbutton);
    //Reading the status of the buttons on the right
    int sbs = digitalRead(sbutton);
    if (fbs == 1 || sbs == 1) {
      bright = true;
    }
    COROUTINE_YIELD();
  }
}

void setup() {
  //SD card initialization
  if (!SD.begin()) {
    return;
  }

  //Matrix initialization
  for (int h = 0; h < 70; h++) {
    for (int i = 0; i < 70; i++) {
      matrix[h][i] = ' ';
    }
  }
  matrix[1][0] = 'H';
  matrix[0][0] = 'W';

  pinMode(isObstaclePin, INPUT);

  pinMode(ml1, OUTPUT);
  pinMode(ml2, OUTPUT);
  pinMode(ml3, OUTPUT);
  pinMode(ml4, OUTPUT);
  digitalWrite(ml1, LOW);
  digitalWrite(ml2, LOW);
  digitalWrite(ml3, LOW);
  digitalWrite(ml4, LOW);

  pinMode(mr1, OUTPUT);
  pinMode(mr2, OUTPUT);
  pinMode(mr3, OUTPUT);
  pinMode(mr4, OUTPUT);
  digitalWrite(mr1, LOW);
  digitalWrite(mr2, LOW);
  digitalWrite(mr3, LOW);
  digitalWrite(mr4, LOW);

  pinMode(fbutton, INPUT);
  pinMode(sbutton, INPUT);

  Serial.begin(9600);

  BTserial.begin(9600);

  CoroutineScheduler::setup();
}

void loop() {
  CoroutineScheduler::loop();
}
