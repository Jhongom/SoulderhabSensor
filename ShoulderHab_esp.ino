#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ShoulderHab"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(5,OUTPUT);

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

//VARIABLES GLOBALES
float angx, angy;
float accel_ang_x;
float accel_ang_y;
  //para el filtrado
const int numReadings  = 20;
int valx [numReadings];
int valy [numReadings];
int readIndex  = 0;
float totalx  = 0;
float totaly  = 0;
float max_x = 0;
float max_y = 0;

char dato='6';//se usa el 6 para inicializar la variable
float angMax = 0;
void loop() {
  
  if (SerialBT.available()) {
    dato = SerialBT.read();
    //Serial.write(SerialBT.read());//para escribir en el monitor serie
  }
  switch(dato){
    case'0'://se usara el case'0' para mandar el angulo max
      SerialBT.println(String(angMax));
      apaga();

      Serial.println("AngMax char");
      Serial.print(angMax);
      Serial.print('\t');
      Serial.println(String(angMax));
      Serial.println();
      delay(20);
      
      max_x = 0;
      max_y = 0;
      //angMax = 0;
      break;
    case'1':
      prende();
      angulos();
      filtro(accel_ang_x, accel_ang_y);
      angy = 70-angy;//<===== CORRECCION DE ANGULO
      anguloMax();
      imprime();
      angMax = max_y;//<======= SE MANDA EL ANGULO EN Y
      break;
    case'2':
      prende();
      angulos();
      filtro(accel_ang_x, accel_ang_y);
      angx=9-angx;//<===== CORRECCION DE ANGULO
      anguloMax();
      imprime();
      angMax = max_x;//<======= SE MANDA EL ANGULO EN x
      break;
    case'3':
      prende();
      angulos();
      filtro(accel_ang_x, accel_ang_y);
      angy = 70-angy;//<===== CORRECCION DE ANGULO
      anguloMax();
      imprime();
      angMax = max_y;//<======= SE MANDA EL ANGULO EN Y
      break;
    case'4':
      prende();
      angulos();
      filtro(accel_ang_x, accel_ang_y);
      angy = angy-55;//<===== CORRECCION DE ANGULO
      anguloMax();
      imprime();
      angMax = max_x;//<======= SE MANDA EL ANGULO EN X
      break;        
    case'5':
      prende();
      angulos();
      filtro(accel_ang_x, accel_ang_y);
      angx = 45-angx;//<===== CORRECCION DE ANGULO
      anguloMax();
      imprime();
      angMax = max_x;//<======= SE MANDA EL ANGULO EN X
      break;
     case'6':
      apaga();
      max_x = 0;
      max_y = 0;
      break;
  }     
    
  
}

void angulos(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  
  accel_ang_x=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  accel_ang_y=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  
}

void filtro(float valorx, float valory) {
  ////Perform average on sensor readings
  float averagex, averagey;
  // subtract the last reading:
  totalx = totalx - valx[readIndex];
  totaly = totaly - valy[readIndex];
  // read the sensor:
  valx[readIndex] = valorx;//valor de angulo que se quiere promediar
  valy[readIndex] = valory;//valor de angulo que se quiere promediar
  // add value to total:
  totalx = totalx + valx[readIndex];
  totaly = totaly + valy[readIndex];
  // handle index
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  // calculate the average:
  averagex = totalx / numReadings;
  averagey = totaly / numReadings;

  angx = averagex;
  angy = averagey;

}

void anguloMax(){
   if(angx > max_x)
        max_x = angx;
   if(angy > max_y)
        max_y = angy; 
}

void imprime(){
  Serial.println("AngX AngY Max_X Max_Y");
  Serial.print(angx); 
  Serial.print('\t');
  Serial.print(angy);
  Serial.print('\t');
  Serial.print(max_x);
  Serial.print('\t');
  Serial.println(max_y);
  Serial.println();
  delay(20);
}

void imprimeAng(){
  Serial.print(angy); 
  Serial.println();
  delay(20);
}

void prende(){
  digitalWrite(5,LOW);
}

void apaga(){
  digitalWrite(5,HIGH);
}
