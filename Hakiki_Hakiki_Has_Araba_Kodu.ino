#include <AFMotor.h>
#include <Servo.h>

// SERVO MOTOR // 
#define SERVO_PIN 9
//#define ENA 10

// MATKAP MOTORU //
#define IN1 2
#define IN2 13

// SU MOTORU //
#define IN3 A3
#define IN4 A4

// TOPRAK NEM SENSÖRü //
#define SENSOR_PIN A5


//  - initial motors pin //
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

Servo myservo;

// MOTOR HIZ DEĞİŞKENLERİ //
int t=200;
int t2 = 255;
String motorSpeed;

// SERİ HABERLEŞME DEĞİŞKENLERİ
char command;

// NEM SENSÖRÜ KALİBRASYONU //
int dry = 895;
int wet = 740;

unsigned long lastSensorReadTime = millis(); // Zamanlayıcı için değişken
unsigned long sensorReadInterval = 500;  // 500ms'lik bir aralık
int percentageHumidity = 0; // Nem Yüzdesi

//Timers //

int olcum_suresi = 10000;
int delme_suresi = 3000;
int sulama_suresi = 5000;

unsigned long previousMillis1 = 0; // Ölçüm
unsigned long previousMillis2 = 0; // Delme
unsigned long wateringTimer = 0; // Sulama

// DEBOUNCE DEĞİŞKENLERİ //
bool isDown = false;
bool canStart = false;
bool canPump = false;
bool canMeasure = false;
bool isWatering = false;
bool shutdown = false;


// ANA MAKİNE DEĞİŞKENLERİ //

enum machineState {IDLE, GOING_DOWN, DRILLING, MEASURING, GOING_UP, WATERING};
machineState machineState = IDLE;


void setup()
{
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

  myservo.attach(SERVO_PIN);
  myservo.write(0);

  pinMode (SENSOR_PIN, INPUT);
  Serial.begin(9600);  //Set the baud rate to your Bluetooth module.

  Serial.println("--- SYSTEM ACTIVATED! ---\n----------\n");
}



void loop(){

    //  motor1.setSpeed(t); // velocity
    //motor1.run(FORWARD); //rotate the motor clockwise
  serialCom(); // // Seri haberlşme fonksiyonu.  
  if (canStart == true) MACHINE();
  
}

// ACİL DURUMLARDA MÜDAHALE FAZI \\


void SHUTDOWN() {

// Tüm parametreleri varsayılan konuma al!
canMeasure = false;
canPump = false;
isDown = false;
canStart = false;
isWatering = false;
machineState = IDLE;

// Tüm motorları durdur!
Stop();
myservo.write(0);
drillDeactivate();
pumpDeactivate();

Serial.println("--- SYSTEM HAS BEEN SHUTDOWN! ---\n");

}

// ÖZEL OLUŞTURULAN SİSTEMİ BLOKE ETMEDEN BEKLEME FONKSİYONU \\ 

bool _WAIT(int interval, unsigned long &previousMillis) {
  unsigned long currentMillis = millis();
  //Serial.println(currentMillis - previousMillis);
  if (currentMillis - previousMillis >= interval) {
    // Bekleme süresi tamamlandı, previousMillis'i güncelle
    int elapsedTime = currentMillis - previousMillis;  // Geçen süreyi hesapla
    previousMillis = currentMillis;  // Zamanlayıcıyı sıfırlıyoruz

    Serial.print("Geçen süre: ");
    Serial.print(elapsedTime);
    Serial.println(" ms");

    return true;  // Bekleme Tamamlandı! :)
  }

  return false;  // Bekleme Tamamlanmadı! :(
}




// ### HABERLŞME KOMUTLARI ### \\

void serialCom() {

  if(Serial.available() > 0){

    command = Serial.read();

    motorSpeed += command;
 //   Serial.println("HERE IS COMMAND: " + motorSpeed);
    //Stop(); //initialize with motors stoped
    //Change pin mode only if new command is different from previous.
 //   Serial.print("Anlık Hız: ");
 //   Serial.println(t);

    if (motorSpeed.charAt(0) == 'm' ) {
    if (motorSpeed.charAt(1) >= '0' && motorSpeed.charAt(1) <= '9') {
    // "m" karakterinden sonrasını alıp sayıya dönüştürme
    int speedValue = motorSpeed.charAt(1) - '0';  // '0' ASCII değeri çıkarılınca sayı olur
    t2 = 25 * (speedValue + 1);  // m0 -> 25, m1 -> 50, vb.
    if (speedValue == 9) {t2 += 5;}
    motorSpeed = "";
    }
    }
    else {
    motorSpeed = "";
    }


    //  4 WD MOTORLARIN HIZ AYARI
    if (command >= '0' && command <= '9') {
    // "m" karakterinden sonrasını alıp sayıya dönüştürme
    int _speedValue = command - '0';  // '0' ASCII değeri çıkarılınca sayı olur
    t = 25 * (_speedValue + 1);  // m0 -> 25, m1 -> 50, vb.
    if (command == 9) {t += 5;}
    }


    //HABERLEŞMEDEN GELEN VERİLERİN DEĞERLENDİRİLMESİ

    switch(command){
      case 'F':
        forward();
        break;
      case 'B':
        back();
        break;
      case 'L':
        left();
        break;
      case 'R':
        right();
        break;
      case 'S':
        Stop();
        break;
      case 'i':
        canStart = true;
        break;
      case 'P':
        pumpPermission();
        break;
      case 'K':
        SHUTDOWN();
        break;
      }
  }


}




// ### TEMEL YÖN KOMUTLARI ### \\ 

void forward()
{
  if (isDown == false) {
    Serial.println("F");
    Serial.println(t);
    motor1.setSpeed(t); // velocity
    motor1.run(FORWARD); //rotate the motor clockwise
    motor2.setSpeed(t); // velocity
    motor2.run(FORWARD); //rotate the motor clockwise
    motor3.setSpeed(t); // velocity
    motor3.run(FORWARD); //rotate the motor clockwise
    motor4.setSpeed(t); // velocity
    motor4.run(FORWARD); //rotate the motor clockwise
  }
  
}

void back()
{
  if (isDown == false){
  Serial.println("B");
  Serial.println(t);
  motor1.setSpeed(t); // velocity
  motor1.run(BACKWARD); //rotate the motor anti-clockwise
  motor2.setSpeed(t); // velocity
  motor2.run(BACKWARD); //rotate the motor anti-clockwise
  motor3.setSpeed(t); // velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(t); // velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
  }
}

void left()
{
  if (isDown == false) {
    Serial.println("L");
    Serial.println(t);
    motor1.setSpeed(t); // velocity
    motor1.run(FORWARD);  //rotate the motor clockwise
    motor2.setSpeed(t); // velocity
    motor2.run(FORWARD);  //rotate the motor clockwise
    motor3.setSpeed(t); // velocity
    motor3.run(BACKWARD); //rotate the motor anti-clockwise
    motor4.setSpeed(t); // velocity
    motor4.run(BACKWARD); //rotate the motor anti-clockwise
  }
  
}

void right()
{
  if (isDown == false) {
    Serial.println("R");
    Serial.println(t);
    motor1.setSpeed(t); // velocity
    motor1.run(BACKWARD); //rotate the motor anti-clockwise
    motor2.setSpeed(t); // velocity
    motor2.run(BACKWARD); //rotate the motor anti-clockwise
    motor3.setSpeed(t); // velocity
    motor3.run(FORWARD);  //rotate the motor clockwise
    motor4.setSpeed(t); // velocity
    motor4.run(FORWARD);  //rotate the motor clockwise
  }

}


void Stop()
{
  Serial.println("S");
  Serial.println(t);
  motor1.setSpeed(0); // velocity
  motor1.run(RELEASE); //stop the motor when release the button
  motor2.setSpeed(0); //velocity
  motor2.run(RELEASE); //rotate the motor clockwise
  motor3.setSpeed(0); // velocity
  motor3.run(RELEASE); //stop the motor when release the button
  motor4.setSpeed(0); //velocity
  motor4.run(RELEASE); //stop the motor when release the button
}



//  ### ÖZELLİK VE FONKSİYONEL KOMUTLAR ### \\



// TOPRAK KAZMA FONKSİYONU \\

void drillActivate() {
  digitalWrite(IN1, HIGH); // Set motor rotor direction to forward. 
  digitalWrite(IN2, LOW); // | yukarıdaki sebepten.

//  Serial.println("--- DRILL ACTIVATED! ---\n");
}

void drillDeactivate() {
  digitalWrite(IN1, LOW); // Set motor rotor direction to stop. 
  digitalWrite(IN2, LOW); // | yukarıdaki sebepten.

//  Serial.println("--- DRILL DEACTIVATED! ---\n");
}

// ANA MAKİNE FONKSİYONU \\

void MACHINE() {
  switch (machineState) {
    case IDLE:
 
//      Serial.println("--- IDLE STATE, ALL FUNCTIONS HAVE STOPPED! ---\n");

      Stop();
      machineState = GOING_DOWN;
      break;

    case GOING_DOWN:

//      Serial.println("--- ARM IS GOING DOWN! ---\n");

      for (int i = 0; i<= 180; i++) {
        myservo.write(i);
        if (i >= 90 && i <= 95) {
          drillActivate();
        }

        delay(17);
      }

//      Serial.println("--- ARM HAS REACHED DOWN! ---\n");

        isDown = true;
        previousMillis1 = millis(); // zamanı başlat
        machineState = DRILLING;
        break;

    case DRILLING:
      if (_WAIT(delme_suresi, previousMillis1)) {

        drillDeactivate();
        canMeasure = true;

        //canPump = true;
        previousMillis2 = millis(); // ikinci zamanlayıcı
        machineState = MEASURING;
//        Serial.println("--- SOIL MOISTURE IS BEING MEASURED! ---\n");
      }
      break;

    case MEASURING:
      measureSensor();
      if (_WAIT(olcum_suresi, previousMillis2)) {

 //       Serial.println("--- SOIL MOISTURE HAS BEEN MEASURED! ---\n");

        canMeasure = false;
        //canPump = false;
        machineState = GOING_UP;
      }
      break;

    case GOING_UP:

//      Serial.println("--- ARM IS GOING UP! ---\n");

      for (int i = 180; i > 0; i--) {
        myservo.write(i);
        delay(17);
      }
//      Serial.println("--- ARM HAS REACHED UP! ---\n");
      
      isDown = false;
      canPump = true;
      wateringTimer = millis();

      machineState = WATERING;
      break;

    case WATERING:
      
      if (canPump == true) pumpActivate();

      if (_WAIT(sulama_suresi,wateringTimer)) { 
        canStart = false;
        machineState = IDLE;
        canPump = false;
        pumpDeactivate();
      }

    break;

  }
}

// TOPRAKI NEMİ SENSÖR ÖLÇÜM \\ 


void measureSensor() {
   if (canMeasure == true) { // Eğer ölçüm izni verildiyse... 
  //ölçüme başla

     // if (isMeasuring == false) {
      //   lastSensorReadTime = millis();
      //  isMeasuring = true;
     // }
     

      if (_WAIT(sensorReadInterval,lastSensorReadTime)) { // Özel oluşturduğumuz _WAIT fonksiyonunu kullanıyoruz.
      int sensorVal = analogRead(SENSOR_PIN);
      percentageHumidity = map(sensorVal, wet, dry, 100, 0);
      //isMeasuring = false;
      // Veriyi ekrana yazdır
      Serial.print("Humidity: ");
      Serial.print(percentageHumidity);
      Serial.print("%");
      Serial.print(", ");
      Serial.println(" ");
    }
  }
}

// SULAMA FONKSİYONU \\

void pumpActivate() {
  digitalWrite(IN3, HIGH); // Set motor rotor direction to forward. // SULAMA BAŞLASIN! 
  digitalWrite(IN4, LOW); // | yukarıdaki sebepten.

 // Serial.println("--- PUMP ACTIVATED! ---\n");
}

void pumpDeactivate() {
  digitalWrite(IN3, LOW); // Set motor rotor direction to STOP. // SULAMA DURUYOR! 
  digitalWrite(IN4, LOW); // | yukarıdaki sebepten.

 // Serial.println("--- PUMP DEACTIVATED! ---\n");
}

void pumpPermission() {
  if (canPump == false) {
    canPump = true;
    pumpActivate();
  }
  else {
    canPump = false;
    pumpDeactivate();
  }
}


