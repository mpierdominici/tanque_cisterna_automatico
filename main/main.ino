//***********************************//
//Modulo temporizador para pileta    //
//                                   //
//Matias Pierdominici                //
//mpierdominici@itba.edu.ar          //
//***********************************//
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Fsm.h"
#include <CircularBuffer.h>

//#define DEBUGG
#define SEC_TO_MILISEC(x) ((x)*1000) 
//*********Pines***********
#define PIN_BOMBA D2
#define PIN_REPOSO D6
#define PIN_BOMBEANDO D5
#define PIN_NOAGUA D4
#define PIN_NONET D7

#define PIN_NIVEL A0
//*********end Pines*******

//****Eventos y estados de la fsm
typedef enum 
{ 
    REPOSO, 
    SIN_AGUA, 
    BOMBEANDO,
    NO_NET
} estados_t;

typedef enum 
{ 
    ENCENDER_BOMBA, 
    APAGAR_BOMBA, 
    TIME_OUT,
    NO_AGUA,
    NET_ERROR,
    RESET
} eventos_t;

CircularBuffer<eventos_t,20> eventList;
//******  Creo los estados y la maquina de estados
State stateReposo(&onEnterReposo, NULL,&onExitReposo);//(enter,during,exit)
State stateSinAgua(&onSinAgua, NULL,&onExitSinAgua);
State stateBombeando(&onBombeando, NULL,&onExitBombeando);
State stateNoNet(&onNoNet, NULL,&onExitNoNet);
Fsm fsm(&stateReposo);
estados_t currentState=REPOSO;
//***************************************************

void onEnterReposo(){
  currentState=REPOSO;
  digitalWrite(PIN_REPOSO,HIGH);
  debug_message("Estoi en reposo",true);
}
void onSinAgua(){
  currentState=SIN_AGUA;
  digitalWrite(PIN_NOAGUA,HIGH);
  debug_message("Estoi en sin agua",true);
}
void onBombeando(){
  currentState=BOMBEANDO;
  digitalWrite(PIN_BOMBEANDO,HIGH);
  debug_message("Estoi en bombeando",true);
}
void onNoNet(){
  currentState=NO_NET;
  digitalWrite(PIN_NONET,HIGH);
  debug_message("Estoi en no net",true);
}

void onExitReposo(){
  digitalWrite(PIN_REPOSO,LOW);
}
void onExitSinAgua(){
  digitalWrite(PIN_NOAGUA,LOW);
}
void onExitBombeando(){
  digitalWrite(PIN_BOMBEANDO,LOW);
}
void onExitNoNet(){
  digitalWrite(PIN_NONET,LOW);
}

void transicionEncenderBomba()
{
  debug_message("Prendo la bomba",true);
  digitalWrite(PIN_BOMBA,HIGH);
}
void transicionApagarBomba()
{
  debug_message("Apago la bomba",true);
  digitalWrite(PIN_BOMBA,LOW);
}

void transicionNoAgua()
{
  transicionApagarBomba();
  debug_message("Prendo led no agua",true);
}

void transicionNoAgua2Reposo()
{
  debug_message("Apago led no agua",true);
}
void transicionNoAgua2NoNet()
{
  debug_message("Apago led no agua",true);
  debug_message("Prendo led net error",true);
}
void transicionBombeando2NoNet()
{
  transicionApagarBomba();
  debug_message("Prendo led net error",true);
}
void transicionNoNet2Reposo()
{
  debug_message("Apago led net error",true);
}
void transicionReposo2NoNet()
{
  debug_message("Prendo led net error",true);
}

class myTimer
{
  public:
  myTimer(unsigned int seconds=0);
  bool timeOver(void);
  void setNewTime(unsigned long seconds_);
  void showInfo();
  
  unsigned long seconds;
  unsigned long startTime;
  void resetTimer(void);
    
};
myTimer watchDogTimer(90);//pongo el watch dog en 90 seugndos


class waterBomb
{
  public:
  waterBomb(unsigned int pin_);
  void onn(){digitalWrite(pin,HIGH);};
  void off(){digitalWrite(pin,LOW);};
  private:
  unsigned int pin;
  
};




char * ssid ="WIFI Pier";
char * pass ="pagle736pagle";
unsigned int mqttPort=1883;

const char MqttUser[]="cisternaBox";
const char MqttPassword[]="1234";
const char MqttClientID[]="cisterna";

IPAddress mqttServer(192,168,0,116);

WiFiClient wclient;
PubSubClient mqtt_client(wclient);





void callback(char* topic, byte* payload, unsigned int length);
void  debug_message (char * string, bool newLine)
{
#ifdef DEBUGG
  if(string !=NULL)
  {
    if (!newLine)
    {
      Serial.print(string);
    }else
    {
      Serial.println(string);
    }
  }
  #endif
}

void setUpWifi(char * ssid, char * pass)
{
  String ip;
  debug_message(" ",true);
  debug_message(" ",true);
  debug_message("Conectandose a: ",false);
  debug_message(ssid,true);

  WiFi.begin(ssid,pass);

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    debug_message(".",false);
  }
  debug_message(" ",true);
  debug_message("Coneccion realizada",true);
  debug_message("La ip es: ",false);
  ip=WiFi.localIP().toString();
  debug_message((char *)ip.c_str(),true);
}

void setUpMqtt(void)
{
  mqtt_client.setServer(mqttServer,mqttPort);
  mqtt_client.setCallback(callback);
}


void callback(char* topic, byte* payload, unsigned int length)
{
  int tiempo=0;
  payload[length]='\n';
  String message((char *)payload);
  debug_message("Llego un mensage, topic:",false);
  debug_message(topic,false);
  debug_message(", payload : ",false);
  debug_message((char *)payload,true);

  if(!strcmp(topic,"cisterna/watchdog"))
  {
   watchDogTimer.resetTimer();
    //debug_message("LLEGO MENSAJE PARA DETENER LA BOMBA",true);
   
  }else if(!strcmp(topic,"cisterna/bomba/on")){
    eventList.push(ENCENDER_BOMBA);
  }else if(!strcmp(topic,"cisterna/bomba/off")){
    eventList.push(APAGAR_BOMBA);
  }
 
}

void reconnect()
{
  while(!mqtt_client.connected())
  {
    debug_message("Intentando conectar al servidor MQTT",true);
    if (mqtt_client.connect(MqttClientID,MqttUser,MqttPassword))
      {
            debug_message("conectado",true);
  
  
            // ...suscrivirse a topicos
            mqtt_client.subscribe("cisterna/watchdog");
            mqtt_client.subscribe("cisterna/bomba/on");
            mqtt_client.subscribe("cisterna/bomba/off");
            mqtt_client.subscribe("cisterna/state");
           
            


      }
      else
      {
        debug_message("intentando conetarse al broker",true);
        delay(3000);
      }
  }
}

void setup() {
  Serial.begin(9600);
  setUpWifi(ssid,pass);
  setUpMqtt();
  pinMode(PIN_BOMBA,OUTPUT);
  pinMode(PIN_REPOSO,OUTPUT);
  pinMode(PIN_BOMBEANDO,OUTPUT);
  pinMode(PIN_NOAGUA,OUTPUT);
  pinMode(PIN_NONET,OUTPUT);

//********Armado de las trancisiciones de la fsm *************************************************
  fsm.add_transition(&stateReposo, &stateBombeando,ENCENDER_BOMBA,&transicionEncenderBomba);
  fsm.add_transition(&stateBombeando, &stateReposo,TIME_OUT,&transicionApagarBomba);
  fsm.add_transition(&stateBombeando, &stateReposo,APAGAR_BOMBA,&transicionApagarBomba);
  fsm.add_transition(&stateBombeando, &stateSinAgua,NO_AGUA,&transicionNoAgua);
  fsm.add_transition(&stateSinAgua, &stateReposo,RESET,&transicionNoAgua2Reposo);
  
  fsm.add_transition(&stateSinAgua, &stateNoNet,NET_ERROR,&transicionNoAgua2NoNet);
  fsm.add_transition(&stateBombeando, &stateNoNet,NET_ERROR,&transicionBombeando2NoNet);
  fsm.add_transition(&stateNoNet,&stateReposo,RESET,&transicionNoNet2Reposo);
  fsm.add_transition(&stateReposo, &stateNoNet,NET_ERROR,&transicionReposo2NoNet);
//************************************************************************************************
  fsm.run_machine();
}



void loop() {
  if (!mqtt_client.connected()) 
  {
      fsm.trigger(NET_ERROR);
      fsm.run_machine();
      eventList.clear();
      reconnect();
      
 }else if(currentState==NO_NET){
  fsm.trigger(RESET);
  fsm.run_machine();
 }
 mqtt_client.loop(); 
 if(currentState==BOMBEANDO){
 if(watchDogTimer.timeOver()){
  eventList.push(TIME_OUT);
  
 }
 }
 if(!eventList.isEmpty()){
  debug_message("hay evento",true);
  fsm.trigger(eventList.pop());
  fsm.run_machine();
  
 }
 

}





//***********************TIMER**********************************



myTimer::myTimer(unsigned int seconds)
{
  setNewTime(seconds);
}

//timeOver
//devuelve true si ya paso el tiempo seteado,
//caso contrario devuelve false
//
bool myTimer::timeOver(void)
{
  if((millis())>startTime)
  {
    //resetTimer();//time over reseteal el timer se lo sque
    return true;
  }
  else
  {
    return false;
  }
}

void myTimer::resetTimer(void)
{
  unsigned long temp=seconds+millis();
 
  startTime=temp;
  //Serial.print("se llamo a rest timer con: ");
  //Serial.println(startTime);
}

void  myTimer::setNewTime(unsigned long seconds_)
{
  unsigned long temp=1000*seconds_;
  //Serial.println(temp);
  seconds=temp;
 
  //Serial.print("s seteo un timer cada: ");
  //Serial.print(seconds_);
  //Serial.print(" se registro un tirmpo de: ");
  //Serial.println(seconds/1000);
  resetTimer();

}

void myTimer::showInfo()
{
  //Serial.println(startTime);
  unsigned long dif=startTime-millis();
  //Serial.print("Remaining time (seconds):");
  //Serial.println(dif/1000);
  //Serial.println(startTime);
  //Serial.println(millis());
  //Serial.println(seconds/1000);
}

//*********************BOMBA DE AGUA********************

waterBomb::waterBomb(unsigned int pin_)
{
  pin=pin_;
  pinMode(pin,OUTPUT);
  off();
}
