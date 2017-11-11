#define MaCaco_DEBUG_INSKETCH
#define MaCaco_DEBUG                1

#define VNET_DEBUG_INSKETCH
#define VNET_DEBUG                    1

#define VNET_DEBUG_INSKETCH
#define VNET_DEBUG                    1
#include <phi_interfaces.h>  
#include "SoulissFramework.h"

#include <ESP8266WiFi.h>
#include <EEPROM.h>
//#include <ESP8266mDNS.h>
//#include <WiFiUdp.h>
#include <string.h>


#include "bconf/MCU_ESP8266.h"              // Load the code directly on the ESP8266
#include "conf/Gateway.h"                   // The main node is the Gateway, we have just one node
#include "conf/IPBroadcast.h"
#include "conf/DynamicAddressing.h"


#define WIFICONF_INSKETCH

#define WiFi_SSID               "MikroTik"
#define WiFi_Password           "blablabla"    
 

// This identify the number of the SLOT logic
#define LEDPWM      0     //THIS TYPICAL USES TWO SLOTS, SO THE NEXT FREE SLOT IS 2.

//PWM pin
#define LEDPWMP     15

//#include <EEPROM.h>
#include "Souliss.h"

#define Encoder1ChnA 5
#define Encoder1ChnB 4
#define EncoderDetent 20
#define EncoderPush 0

#define ZeroCross  14
#define POut 2 
#define MaxVal PWMRANGE
#define Channels 3
#define Step MaxVal/EncoderDetent

#define ssInput(slot)            memory_map[IN+slot]
#define ssOutput(slot)            memory_map[OUT+slot]




//WiFiUDP Udp;


char mapping1[]={'U','D'}; // This is a rotary encoder so it returns U for up and D for down on the dial.
phi_rotary_encoders my_encoder1(mapping1, Encoder1ChnA, Encoder1ChnB, EncoderDetent);
multiple_button_input* dial1=&my_encoder1;

char mapping[]={'P'}; // This is a list of names for each button.
byte pins[]={EncoderPush}; // The digital pins connected to the 6 buttons.
phi_button_groups my_btns(mapping, pins, 1);
multiple_button_input* pad1=&my_btns;

int channel=0;
long dimmervalue=0;
int val[]={0,0,0};
//char[32] outmapping["","192.168.8.92:10000/vol/1","192.168.8.92:10000/col/1"];
//char[32] outmapping["","192.168.8.222:10000/1w/0","192.168.8.92:10000/col/1"];

int curchan=0;
int imp=0;


void zero()
{

//#define TIM_DIV1   0 //80MHz (80 ticks/us - 104857.588 us max)
//#define TIM_DIV16 1 //5MHz (5 ticks/us - 1677721.4 us max)
//#define TIM_DIV265  3 //312.5Khz (1 tick = 3.2us - 26843542.4 us max)
  if (dimmervalue==1) digitalWrite(POut,0); 
  else
  {
  digitalWrite(POut,1);
  imp=0;
  if (dimmervalue>0) timer1_write(dimmervalue);
  }
}

void handler()
{
  if (imp==0) 
     {
      digitalWrite(POut,0);
      timer1_write(2000);
      imp=1;
     }
  else   
  {   
  digitalWrite(POut,1);
  imp=0;
  }
}

//OTA_Setup();
void setup()
{
  Serial.begin(115200);
 
  pinMode(POut,OUTPUT);
  pinMode(ZeroCross,INPUT_PULLUP);

 
  Serial.println("Connecting Wifi...");
  
    
//  Udp.begin(inPort);       


attachInterrupt(ZeroCross,zero,FALLING);
timer1_isr_init();
timer1_attachInterrupt(handler);
timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);


  //analogWriteFreq(500); 
  //analogWriteRange(255); 

    Initialize();  
    GetIPAddress();   
    SetAsGateway(myvNet_dhcp);
    SetAddress(0xD001, 0xFF00, 0x0000);
    SetAddressingServer();
    // Connect to the WiFi network and get an address from DHCP
       
   
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(myvNet_dhcp);
    
    // This is the vNet address for this node, used to communicate with other
  // nodes in your Souliss network
    //
    //Example of Peer Definition
    //SetAsPeerNode(0xAB02, 1);
    
    // This node will serve all the others in the network providing an address
   
    Set_DimmableLight(LEDPWM);
    pinMode(LEDPWMP, OUTPUT);

//     OTA_Init();    

}





void rotary_loop()
{
  char temp;
  U8 data_changed;
  //int t;
  temp=dial1->getKey(); // Use the phi_interfaces to access the same keypad
  switch (temp) {
  case 'U': val[channel]+=Step;
  //mInput(0) = 
//   if(memory_map)  memory_map[MaCaco_IN_s + 0] = Souliss_T1n_BrightUp ;
    ssInput(0)=Souliss_T1n_BrightUp;
    Souliss_Logic_T19(memory_map, 0, &data_changed);
  //DigKeepHold(32,Souliss_T1n_ToggleCmd, Souliss_T1n_BrightUp, 0);  
  break;
  case 'D': val[channel]-=Step;
  //mInput(0) 
   if(memory_map)  memory_map[MaCaco_IN_s + 0] = Souliss_T1n_BrightDown ;
   ssInput(0)=Souliss_T1n_BrightDown;
   Souliss_Logic_T19(memory_map, 0, &data_changed);
  //DigKeepHold(32,Souliss_T1n_ToggleCmd, Souliss_T1n_BrightDown, 0);  
  
  }
  
  
  if (temp!=NO_KEY) 
  {  
   // Logic_DimmableLight(LEDPWM);
    Serial.print("Key: ");  Serial.println(temp);
     if (val[channel]>MaxVal) val[channel]=MaxVal;
     
     if (val[channel]<0) 
            {
            val[channel]=0;
  //          mInput(0) =  Souliss_T1n_OffFeedback;
            }
 //       else  mInput(0) =  Souliss_T1n_OnFeedback;   
//     mInput(LEDPWM+1,val[channel]);
     Serial.println(val[channel] );
  ////ssInput(LIGHT2_NODE1)=Souliss_T1n_BrightDown;  
  }
  temp=my_btns.getKey(); // Use phi_button_groups object to access the group of buttons

  
  
  if (temp=='P') 
      {
        switch (pad1->get_status()) {
        case buttons_pressed:
           curchan=channel;
           if ((val[0] <0)||(Channels==1)) val[0]=-val[0];
              else 
                  channel++;
        break;
        case buttons_held:
        channel=curchan;
        if (val[channel]>0) val[channel]=-val[channel];
        }
        
      if (channel>=Channels) channel=0;
     // Serial.println(t);
      Serial.print("Channel: ");
      Serial.println(channel);
        
      Serial.print("Value: ");
      Serial.println(val[channel]);
      }
//if (val[0]>0) analogWrite(POut,val[0]); else analogWrite(POut,0);
if (val[0]>0) dimmervalue=map(val[0],0,MaxVal,45000,1); else dimmervalue=0;
//Serial.println(dimmervalue); 
 

}

void loop()
{ 
     rotary_loop();
     
    EXECUTEFAST() {                     
        UPDATEFAST();   
        
        FAST_50ms() {   // We process the logic and relevant input and output every 50 milliseconds
            Logic_DimmableLight(LEDPWM);                        
            analogWrite(LEDPWMP, mOutput(LEDPWM+1)*4);
           
        }
        // Here we handle here the communication with Android
        //ProcessCommunication();
        FAST_GatewayComms();                                        
    }
    EXECUTESLOW() {
 UPDATESLOW();


            SLOW_10s() {  // Read temperature and humidity from DHT every 110 seconds  
                Timer_DimmableLight(LEDPWM);              
            }            

 
    }
  //   OTA_Process();
}   
