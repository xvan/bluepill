#include <Wire_slave.h>
#include <RTClock.h>
#include <SD.h>

//objetos
RTClock rtc (RTCSEL_LSE);
uint32 tt;
tm_t mtt;
File myFile;


//variables
int estado = 1;
int k = 0;
float c[6];
const char * weekdays[] = {"Lun", "Mar", "Mier", "Jue", "Vie", "Sab", "Dom"};
const char * months[] = {"Dummy", "Ene", "Feb", "Mar", "Abr", "May", "Jun", "Jul", "Ago", "Sep", "Oct", "Nov", "Dic" };
int seg[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59};
float I;
float V1;
float V2;
float soc1;
float soc2;
float mmu_id;
bool EQ1 = false;
bool EQ2 = false;
byte ecu = 0;
int m; // para recorrer las mmu
const int TIMEOUT_MS = 1000; // para comunicacion i2c con los esclavos
bool alarma=false;

void Interrupcion_RTC() {
  k++;
  if (k>3){ //para que el pedido sea cada 4 segundos
  estado = 2; //solicitar
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));// blink the led
  k=0;
  }
}

void InterrupcionAlarma() {
  alarma=true;
  digitalWrite(PC15,alarma);  // se activa la llave de corte
  
}


void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
  delay(50);
 
  pinMode(LED_BUILTIN, OUTPUT); //led de la placa
  pinMode(PC15, OUTPUT); //llave de corte de zona segura
  digitalWrite(PC15,LOW);  // inicializa apagado 

//Cabecera de DATALOGG en la transmisión serial// 

  Serial.println("Inicializacion exitosa!");
  Serial.println();
  Serial.println(" id \t V1 \t V2 \t I \t SoC1 \t SoC2 \t Fecha ");
  

//Inicializacion de SD.
  int c = 0;
  while ((!SD.begin()) and (c < 6)) { // espera un tiempo a que responda la sd
    c++;
    delay(100);
    Serial.print(".");
  }
  myFile = SD.open("prueba.txt", FILE_WRITE);  // Crea el archivo
  delay(100);
  if (myFile) {
    myFile.println("id \t V1 \t V2 \t I \t SoC1 \t SoC2 \t Fecha ");
    myFile.close();    // Cierra el archivo
    Serial.println("SD ok");
  } else {
    Serial.println("SD no conectada");
  }


//---------------- Configuracion de la interrupcion  //
  rtc.attachSecondsInterrupt(Interrupcion_RTC);
  attachInterrupt(digitalPinToInterrupt(PA0),InterrupcionAlarma, RISING);
}                                           
  
void loop()
{
  switch (estado) {
    case 1:
      estar(); //
      break;
    case 2:
      solicitar(); //
      break;
    default:
      //Standby
    break;
  }
} 

 void estar() {
    estado=1;
    }

    
//####-------- estado=2 : solicitar --------####//
   
 void solicitar() {
  byte data[sizeof(byte) + sizeof(float) * 5];
  byte id;
  
  for(int n=1; n<3; n++){
  m=10*n;
  
  Wire.requestFrom(m, 24);    // request  bytes from slave device #id
  
  unsigned long startTime = millis();    // Restablece el tiempo de inicio para esperar respuesta de la mmu
  while (millis() - startTime <= TIMEOUT_MS && !Wire.available()) {
      // Espera hasta que haya datos o se alcance el tiempo de espera
    }
 if (Wire.available()){    
  while (Wire.available() < sizeof(data)) {
    // Espera hasta que se reciban suficientes bytes
 }
// Si se llega aquí, se recibieron los datos antes del tiempo de espera
  
// Lee el arreglo de datos

  for (int i = 0; i < sizeof(data); i++) {
    data[i] = Wire.read();
  }
   
    // Extrae las variables del arreglo de datos
  memcpy(&id, &data[0], sizeof(byte));
  memcpy(&V1, &data[sizeof(byte)], sizeof(float));
  memcpy(&V2, &data[sizeof(byte) + sizeof(float)], sizeof(float));
  memcpy(&I, &data[sizeof(byte) + sizeof(float) * 2], sizeof(float));
  memcpy(&soc1, &data[sizeof(byte) + sizeof(float) * 3], sizeof(float));
  memcpy(&soc2, &data[sizeof(byte) + sizeof(float) * 4], sizeof(float));

// el byte id contiene 3 bits con informacion (bit0, bit1 y bit2)
/* */
EQ1 = (id & 0b00000001) > 0; // Máscara para extraer el bit 0 (EQ1)
EQ2 = ((id >> 1) & 0b00000001) > 0;// Máscara para extraer el bit 1 (EQ2)
bool d = ((id >> 2) & 0b00000001) > 0;// Máscara para extraer el bit 2 (mmu_id)

mmu_id=20; // 
if (d){mmu_id=10;}; // Tener en cuenta que: si d=0 => mmu=20 // si d=1 => mmu=10  

// Analisis de diferencia de tension p/ecualizacion     

    if ((EQ1==0) && (EQ2==0) && (V1-V2 > 0.02))  // hay que ecualizar V1
      {   
       EQ1=true;
       //EQ2=false;    
      }
  
    else if ((EQ1==0) && (EQ2==0) && (V2-V1 > 0.02)) // hay que ecualizar V2
      {   
       //EQ1=false;
       EQ2=true;     
      }
      
/*    else if ((V1-V2 < 0.05) || (V2-V1 < 0.05) ) //  hay que dejar de ecualizar
      {             
       EQ1=false;
       EQ2=false; 
      } */
 delay(200); 
//++++++ Envia orden de ecualizacion +++++

// Combina las variables booleanas en un solo byte llamado 'ecu'

ecu=B00000000;
ecu = (EQ2 << 1) | EQ1;

  Wire.beginTransmission(m); // transmit to device #id
  Wire.write(ecu);              // sends one byte
  Wire.endTransmission();    // stop transmitting


 //####--------  datalog --------####//

rtc.breakTime(rtc.now(), mtt);

/* datos a la sd*/

  myFile = SD.open("Prueba.txt", FILE_WRITE);  // Abre el archivo
  if (myFile) {
    
     myFile.print(mmu_id);//id
  myFile.print("\t");
  myFile.print(V1,4);//V1
  myFile.print("\t");
  myFile.print(V2,4);//V2
  myFile.print("\t");
  myFile.print(I,4);//I
  myFile.print("\t");
  myFile.print(soc1,4);//soc1
  myFile.print("\t");
  myFile.print(soc2,4);//soc2
  myFile.print("\t");
   myFile.print(EQ1);//soc1
  myFile.print("\t");
  myFile.print(EQ2);//soc2
  myFile.print("\t");
    myFile.print(months[mtt.month]);
    myFile.print(" ");
    myFile.print(weekdays[mtt.weekday]);
    myFile.print(" ");
    myFile.print(mtt.year);
    myFile.print(" ");
    myFile.print(mtt.hour);     
    myFile.print(":");
    myFile.print(mtt.minute);
    myFile.print(":");   
    myFile.println(mtt.second);
    myFile.close();    // Cierra el archivo
  }

/* datos al puerto serial para visualizar con la PC*/
  Serial.print(mmu_id);//id
  Serial.print("\t");
  Serial.print(V1,4);//V1
  Serial.print("\t");
  Serial.print(V2,4);//V2
  Serial.print("\t");
  Serial.print(I,4);//I
  Serial.print("\t");
  Serial.print(soc1,4);//soc1
  Serial.print("\t");
  Serial.print(soc2,4);//soc2
  Serial.print("\t");
  Serial.print(EQ1);//soc1
  Serial.print("\t");
  Serial.print(EQ2);//soc2
    Serial.print("\t");
    Serial.print(months[mtt.month]);
    Serial.print(" ");
    Serial.print(weekdays[mtt.weekday]);
    Serial.print(" ");
    Serial.print(mtt.year);
    Serial.print(" ");
    Serial.print(mtt.hour);     
    Serial.print(":");
    Serial.print(mtt.minute);
    Serial.print(":");   
    Serial.println(mtt.second); 
    
      
  } /* aca termina el if en el que se ingresa si la mmu respondió*/  
 
 else {
 /* aca se ingresa si se superó el tiempo de espera y la mmu no respondió*/
 /*entonces se logea un mensaje */ 
rtc.breakTime(rtc.now(), mtt);
 myFile = SD.open("Prueba.txt", FILE_WRITE);  // Abre el archivo
 if (myFile) {
  myFile.print(m);//id de la q no respondio
  myFile.print("\t");
  myFile.print("No hay respuesta");   
  myFile.print("\t");
    myFile.print(months[mtt.month]);
    myFile.print(" ");
    myFile.print(weekdays[mtt.weekday]);
    myFile.print(" ");
    myFile.print(mtt.year);
    myFile.print(" ");
    myFile.print(mtt.hour);     
    myFile.print(":");
    myFile.print(mtt.minute);
    myFile.print(":");   
    myFile.println(mtt.second); 
    myFile.close();    // Cierra el archivo
  }


/*mensaje en el puerto serial */  
 
  Serial.print(m);//id de la q no respondio
  Serial.print("\t");
  Serial.print("No hay respuesta");   
  Serial.print("\t");
    Serial.print(months[mtt.month]);
    Serial.print(" ");
    Serial.print(weekdays[mtt.weekday]);
    Serial.print(" ");
    Serial.print(mtt.year);
    Serial.print(" ");
    Serial.print(mtt.hour);     
    Serial.print(":");
    Serial.print(mtt.minute);
    Serial.print(":");   
    Serial.println(mtt.second); 
    }    
 

  } // fin del for que recorre las mmu  
  
  estado=1;
}
