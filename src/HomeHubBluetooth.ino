/*
 * Project          multicolorlamp
 * Description:     A neopixel powered desk lamp
 * Author:          Matthew Panizza
 * Date:            7/21/20
 */

/////////////////////////////////
/////// LIBRARIES INCLUDED //////
/////////////////////////////////

#include <neopixel.h>
#include "tplink.h"
#include "math.h"

/////////////////////////////////
//// NEOPIXEL CONFIGURATION /////
/////////////////////////////////

    //// INITIALLY START WITH WIFI OFF
    SYSTEM_MODE(SEMI_AUTOMATIC);

    //// SELECT ANTENNA - NORMALLY USING EXTERNAL ANTENNA
    STARTUP(WiFi.selectAntenna(ANT_INTERNAL));

    //// LED HARDWARE
    #define PIXEL_TYPE WS2812B

    //// MAIN MATRIX
    #define PIXEL_PINA A3
    #define PIXEL_COUNTA 12
    Adafruit_NeoPixel stripa = Adafruit_NeoPixel(PIXEL_COUNTA, PIXEL_PINA, PIXEL_TYPE);

    //// EXTERNAL STRIP
    #define PIXEL_PINB A5
    #define PIXEL_COUNTB 64
    Adafruit_NeoPixel stripb = Adafruit_NeoPixel(PIXEL_COUNTB, PIXEL_PINB, PIXEL_TYPE);
    
    //// EXTERNAL RING
    #define PIXEL_PINC D6
    #define PIXEL_COUNTC 12
    Adafruit_NeoPixel stripc = Adafruit_NeoPixel(PIXEL_COUNTC, PIXEL_PINC, PIXEL_TYPE);

/////////////////////////////////
///// NEOPIXEL COLOR CONFIG /////
/////////////////////////////////

    //// Lamp Matrix Warm Color
    #define rscale 5.2  
    #define gscale 14.167   
    #define bscale 75     

    //// Lamp Matrix Cool Color
    #define rscale3 5.2
    #define gscale3 10.0
    #define bscale3 21.667
    
    //// Strip Warm Color
    #define rscale2 7.8 //5.2
    #define gscale2 25 //16.667
    #define bscale2 150

    //// Strip Cool Color
    #define rscale4 9 //5.2
    #define gscale4 25 //14.167
    #define bscale4 80  //46.667

/////////////////////////////////
////   LIGHT CONTROL MACROS  ////
/////////////////////////////////

    #define NLIGHT      11,4,1      //Night light pixel color

    #define NLIGHT_PXNUM    3       //Pixel number of night-light

/////////////////////////////////
//// SOFTWARE CONTROL MACROS ////
/////////////////////////////////

    #define MAXMODE         3           //Maximum number of control modes
    
    #define ONBRT           0.15        //Default starting brightness for auto
    #define ONTEMP          35          //Default starting temperature for auto
    #define ENDTEMP         25          //Default ending temperature for auto
    #define HRENDFADE       23          //Hour to reach ending auto-temperature
    #define FADETIME_MIN    30          //Amount of time to fade to full brightness

/////////////////////////////////
///// EEPROM CONFIGURATION //////
/////////////////////////////////

    #define BT_MAXDEVICES       8           //Max number of storeable devices
    #define EEP_DEVCOUNT_REG    16          //Device eeprom max location, should be 2x BT_MAXDEVICES
    #define EEP_MODE_REG        17          //EEPROM location to hold last used mode
    #define EEP_BRT_REG         18          //EEPROM location to hold last used brightness value
    #define EEP_COL_REG         19          //EEPROM location to hold last used color temperature

/////////////////////////////////
//// BLUETOOTH CONFIGURATION ////
/////////////////////////////////   

    #define BT_BOUND            -65         //dBi of received signal to turn on lights
    #define BT_TIMEOUT_MS       180000      //Number of seconds before turning lights off
    #define BT_AUTO_RE_EN_MS    600000      //Number of milliseconds before re-enabling scanner after manual shutoff
    #define BT_BRT_AUTO_ON      300         //Minimum photoresistor value for automatic turn on of lights   

/////////////////////////////////
///// HARDWARE CONFIGURATION ////
/////////////////////////////////

    #define TOGGLE D0   // Single hardware push button

    #define GRNLT D1    // Green indicator on joystick

    #define BLULT D2    // Blue indicator on joystick

    #define HSNS A0     // Horizontal stick sensor
    
    #define VSNS A1     // Vertical stick sensor

    #define BRTSNS A2   // Photoresistor brightness sensing

    #define VAXIS_OFFSET    2500

/////////////////////////////////
/////// TP-Link Plug IPs ////////
/////////////////////////////////

/*uint8_t plugIP1[4] = {192,168,1,78};
uint8_t plugIP2[4] = {192,168,1,82};
uint8_t plugIP3[4] = {192,168,1,80};   //Kitchen lights
uint8_t plugIP4[4] = {192,168,1,81};   //TV lights

uint8_t bulbIP1[4] = {192,168,1,76};   //KL-130
uint8_t bulbIP2[4] = {192,168,1,75};   //KL-130
uint8_t bulbIP3[4] = {192,168,1,83};   //KL-135*/

uint8_t plugIP1[4] = {192,168,1,2};
uint8_t plugIP2[4] = {192,168,1,4};
uint8_t plugIP3[4] = {192,168,1,9};   //Kitchen lights
uint8_t plugIP4[4] = {192,168,1,23};   //TV lights

uint8_t bulbIP1[4] = {192,168,1,65};   //KL-130
uint8_t bulbIP2[4] = {192,168,1,70};   //KL-130
uint8_t bulbIP3[4] = {192,168,1,7};   //KL-135

/////////////////////////////////
//// GLOBAL VARIABLE DEFS ///////
/////////////////////////////////
SerialLogHandler logHandler(LOG_LEVEL_INFO);

#define modecount 2

int mode;               //current operating mode of lamp
bool update;             //var to update sensor values

int i;                  //loop vars
int j;

int Vaxis;              //variable to hold positions of joystick
int Haxis;
int photo;              //Variable to hold value of photoresistor sensor

int colorId;
uint16_t minsElapsed;
uint8_t startFadeHr;
float curColorTemperature;
float targetColorTemperature;
double curStripBrightness;
int color_table_r[14];  //red values for color modes
int color_table_g[14];  //green values for color modes
int color_table_b[14];  //blue values for color modes

//Bluetooth Variables
uint8_t numStoredDevices;               //Counter for number of saved devices
uint8_t storedKeys[BT_MAXDEVICES<<1];   //Array to hold byte keys for devices
uint8_t devRSSI[BT_MAXDEVICES];         //Array to hold RSSI for each device          
uint32_t lastHeard;                     //Time in milliseconds when a known device was heard
bool AutoCTL;                           //Boolean to disable automatic shutoff when no bluetooth device is present
bool reEnOnDiscover;                    //Boolean to tell scanner to re-enable if device is found after a manual turnoff
bool scanEn;                            //Do bluetooth scanning


uint32_t timeAutoOn;


//RGBColor does the color calculations for the neopixels to implement the color temperature and brighness
class RGBColor{
public:
    int red, green, blue;
    void setColorTemperature(float kelvinTemp){
        //Red Calculation
        if(kelvinTemp <= 66){
	        red = 255;
        }
        else{
            red = kelvinTemp - 60;
            red = 329.698727446*pow(red,-0.1332047592);
            if(red < 0) red = 0;
            if(red > 255) red = 255; 
        }

        //Green Calculation
        if(kelvinTemp <= 66){
            green = 92*log(kelvinTemp) - 161.1195681661;
            if(green < 0) green = 0;
            if(green > 255) green = 255;
        }
        else{
            green = kelvinTemp-60;
            green = 247*pow(green,-0.0755148492);
            if(green < 0) green = 0;
            if(green > 255) green = 255;
        }

        //Blue Calculation
        if(kelvinTemp >= 66){
            blue = 255;
        }
        else{
            if(kelvinTemp <= 19){
                blue = 0;
            }
            else{
                blue = kelvinTemp - 10;
                blue = 125 * log(blue) - 305.0447927307;
                if(blue < 0) blue = 0;
                if(blue > 255) blue = 255;
            }
        }
    }
};

RGBColor currentColor;  //Declare a color object
double dimPCT;          //Global to hold the dimming percentage from the slider

//FillStrip will fill all system neopixels if the booleans stra, strb,strc are set high, R1,G1,etc are the colors to set the pixels to.
//Time delay creates a delay between updates which creates an animation. brtpct sets the dimming percent of the color
void fillStrip(bool stra, bool strb, bool strc, int numPixels, int R1, int G1, int B1, int R2, int G2, int B2, uint32_t timeDelay, float brtpct){
    if(timeDelay > 0){
        for(float dim = 0.00; dim <= brtpct; dim += 0.05){
            for(i = 0; i < numPixels; i++){
                if(stra && i < PIXEL_COUNTA){
                    stripa.setPixelColor(i,R1*dim,G1*dim,B1*dim);
                }
                if(strb && i < PIXEL_COUNTB){
                    stripb.setPixelColor(i,R2*dim,G2*dim,B2*dim);
                }
                if(strc && i < PIXEL_COUNTC){
                    stripc.setPixelColor(i,R1*dim,G1*dim,B1*dim);
                }
            }
            stripa.show();
            stripb.show();
            stripc.show();
            delay(timeDelay);
        }
    }
    else{
        for(i = 0; i < numPixels; i++){
            if(stra && i < PIXEL_COUNTA){
                stripa.setPixelColor(i,R1*brtpct,G1*brtpct,B1*brtpct);
            }
            if(strb && i < PIXEL_COUNTB){
                stripb.setPixelColor(i,R2*brtpct,G2*brtpct,B2*brtpct);
            }
            if(strc && i < PIXEL_COUNTC){
                stripc.setPixelColor(i,R1*brtpct,G1*brtpct,B1*brtpct);
            }
        }
        stripa.show();
        stripb.show();
        stripc.show();
    }
    
}


void strcontrol(){
    update = false;
    if(mode == 0){
        for(i=0;i<64;i++){
            TPLink_Bulb(bulbIP1,1,100-i*1.5625,curColorTemperature*100);
            TPLink_Bulb(bulbIP2,1,100-i*1.5625,curColorTemperature*100);
            TPLink_Bulb(bulbIP3,1,100-i*1.5625,curColorTemperature*100);
            stripa.setPixelColor(i,0,0,0);
            stripa.show();
            delay(10);
            stripb.setPixelColor(i,0,0,0);
            stripb.show();
            delay(10);
            stripc.setPixelColor(i,0,0,0);
            stripc.show();
            delay(10);
            //if(digitalRead(TOGGLE) == HIGH) return;
        }
        TPLink_Bulb(bulbIP1,0,0,curColorTemperature*100);
        TPLink_Bulb(bulbIP2,0,0,curColorTemperature*100);
        TPLink_Bulb(bulbIP3,0,0,curColorTemperature*100);
    }
    else if(mode == 1){ 
        fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 0, VAxisToPct(Vaxis));
        TPLink_Bulb(bulbIP1,1,(int)(VAxisToPct(Vaxis)*100),curColorTemperature*100);
        TPLink_Bulb(bulbIP2,1,(int)(VAxisToPct(Vaxis)*100),curColorTemperature*100);
        TPLink_Bulb(bulbIP3,1,(int)(VAxisToPct(Vaxis)*100),curColorTemperature*100);
        EEPROM.write(EEP_COL_REG,(uint8_t)(VAxisToPct(Vaxis)*100));
        delay(3);
    }
    else if(mode == 2){
        curColorTemperature = (VAxisToPct(Vaxis)*100)+10;
        curStripBrightness = (EEPROM.read(EEP_COL_REG))/100.0;
        TPLink_Bulb(bulbIP1,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP2,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP3,1,curStripBrightness*100,curColorTemperature*100);
        EEPROM.write(EEP_BRT_REG,curColorTemperature);
        currentColor.setColorTemperature(curColorTemperature);
        fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 0, curStripBrightness);
        delay(3);
    }
    else if(mode == 3){
        currentColor.setColorTemperature(curColorTemperature);
        fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 0, curStripBrightness);
        TPLink_Bulb(bulbIP1,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP2,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP3,1,curStripBrightness*100,curColorTemperature*100);
        delay(5);
    }
}

void setup() {
    //Particle.subscribe("btnpress999", webHandler, ALL_DEVICES);
    //Particle.subscribe("brightness999", brtHandler, ALL_DEVICES);
    

    WiFi.setCredentials("p0cK3T-r0uT3R", "NC$U3ngineering");

    BLE.selectAntenna(BleAntennaType::INTERNAL);
    BLE.setScanTimeout(50);


    Particle.variable("V-Axis", Vaxis);
    Particle.variable("H-Axis", Haxis);
    Particle.variable("PhotoResistor", photo);
    Particle.variable("mode", mode);
    //Particle.variable("Dim Percent", dimPCT);
    Particle.variable("Brightness",(double)curStripBrightness);
    Particle.variable("Temperature",(double)curColorTemperature);
    Particle.variable("update", update);

    Particle.function("lightControl", brewCoffee);
    Particle.function("SetLightColor", setBulbColor);
    
    pinMode(TOGGLE, INPUT_PULLDOWN);
    pinMode(BLULT, OUTPUT);
    pinMode(GRNLT, OUTPUT);

    //EEPROM.write(EEP_MODE_REG,(uint8_t)1);
    //EEPROM.write(EEP_BRT_REG,(uint8_t)25);
    //EEPROM.write(EEP_COL_REG,(uint8_t)100);

    //for(int k = 0; k < (BT_MAXDEVICES>>1); k++) EEPROM.write(i,0);
    //EEPROM.write(EEP_DEVCOUNT_REG,0);

    EEPROM.write(0,7);
    EEPROM.write(1,31);
    EEPROM.write(2,5);
    EEPROM.write(3,152);

    

    EEPROM.get(EEP_DEVCOUNT_REG, numStoredDevices);          //Populate device count from EEPROM
    for(int k = 0; k < (numStoredDevices << 1); k++){     //Populate data keys from EEPROM
        EEPROM.get(k,storedKeys[k]);

    }
    for(int k = 0; k < numStoredDevices; k++){          //Set device RSSI's
        devRSSI[k] = 65;
    }
    devRSSI[1] = 100;
    scanEn = true;

    lastHeard = 0;
    AutoCTL = true;
    reEnOnDiscover = false;

    if(analogRead(BRTSNS) > 220){
        if(EEPROM.read(EEP_MODE_REG) == 2){
            mode = 2;
        }
        else{
            mode = 1;
        }
    }
    else{
        mode = 0;
    }
    update = true;
    
    digitalWrite(BLULT, HIGH);
    digitalWrite(GRNLT, HIGH);

    Vaxis = analogRead(VSNS)-VAXIS_OFFSET;
    Haxis = analogRead(HSNS);
    dimPCT = VAxisToPct(Vaxis);

    stripa.begin();
    stripa.show();
    delay(5);

    stripa.setPixelColor(NLIGHT_PXNUM,NLIGHT);
    stripa.show();
    delay(5);

    stripb.begin();
    stripb.show();
    delay(5);

    stripc.begin();
    stripc.show();
    delay(10);

    if(digitalRead(TOGGLE)==LOW){
        Particle.connect();
            while(!Particle.connected()) delay(5);
        BLE.on();
    }

    curColorTemperature = EEPROM.read(EEP_BRT_REG);
    curStripBrightness = EEPROM.read(EEP_COL_REG)/100.0;
    currentColor.setColorTemperature(curColorTemperature);

    if(analogRead(BRTSNS) > 220)
    {
        digitalWrite(BLULT, LOW);
        if(mode == 1){
            fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 30, VAxisToPct(Vaxis));
        }
        else if(mode == 2){
            fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 30, curStripBrightness);
        }
    }
    
    digitalWrite(BLULT, LOW);
    digitalWrite(GRNLT, LOW);
    RGB.control(true);
    RGB.color(0,0,0);
}

void loop() {
    /////////////////////////////////////////////////
    //////////    Bluetooth Scan Call     ///////////
    /////////////////////////////////////////////////

    if(scanEn){
        BLE.scan(scanResultCallback, NULL); //Scan for devices
        if(mode == 0 && AutoCTL){      //If lamp is currently off
            if(millis()-lastHeard < BT_TIMEOUT_MS && analogRead(BRTSNS) < BT_BRT_AUTO_ON){   //Check that the brightness sensor is dark so doesn't turn on during the day
                Log.info("Automatic-ON");
                TPLink_Bulb(bulbIP1,1,curStripBrightness*100,curColorTemperature*100);
                TPLink_Bulb(bulbIP2,1,curStripBrightness*100,curColorTemperature*100);
                TPLink_Bulb(bulbIP3,1,curStripBrightness*100,curColorTemperature*100);
                mode = 1;
                update = true;
                fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 30, VAxisToPct(Vaxis));
            }
            //else{
            //    Log.info("Time Diff: %d",millis()-lastHeard);
            //    delay(50);
            //}
        }
        else if(AutoCTL){ // If lamp is on, turn of iff timed out
            if(millis()-lastHeard > BT_TIMEOUT_MS){
                Log.info("Automatic-OFF");
                TPLink_Bulb(bulbIP1,0,0,2700);
                TPLink_Bulb(bulbIP2,0,0,2700);
                TPLink_Bulb(bulbIP3,0,0,2700);
                //EEPROM.write(EEP_MODE_REG,(mode%2));
                mode = 0;
                update = true;
                strcontrol();
            }
            //else{
            //    Log.info("Time Diff: %d",millis()-lastHeard);
            //    delay(50);
            //}
        }
        else if(!reEnOnDiscover){
            if(millis()-lastHeard > BT_AUTO_RE_EN_MS){  //Re-enable scanner if device has disappeared for a longer period of time
                Log.info("Manually on, looking for devices...");
                reEnOnDiscover = true;
            }
        }
    }
    
    /////////////////////////////////////////////////
    ///////////  Lamp Update Control  ///////////////
    /////////////////////////////////////////////////
    
    if(Vaxis-(analogRead(VSNS)-VAXIS_OFFSET) > 25 || Vaxis-(analogRead(VSNS)-VAXIS_OFFSET) < -25){
        update = true;
        delay(5);
    }
    else{
        delay(10);
    }

    /////////////////////////////////////////////////
    //////////  Update Sensor Variables  ////////////
    /////////////////////////////////////////////////
    Vaxis = analogRead(VSNS)-VAXIS_OFFSET;
    Haxis = analogRead(HSNS);
    photo = analogRead(BRTSNS);
    dimPCT = VAxisToPct(Vaxis);

    if(digitalRead(TOGGLE) == LOW && analogRead(BRTSNS) >= 220 && update == true){
        strcontrol();
    }

    ///////////////////////////////////////////////////
    //  Automatic Brightness and Temperature Control //
    ///////////////////////////////////////////////////
    if(mode == 3 && millis() > timeAutoOn){
        if(minsElapsed < FADETIME_MIN*2){
            if(curStripBrightness < 1){
                curStripBrightness += (double)((1.0-ONBRT)/(FADETIME_MIN*2));
                if(curStripBrightness > 1) curStripBrightness = 1;
            }
            if(curColorTemperature > targetColorTemperature){
                curColorTemperature  -= (double)((ONTEMP-targetColorTemperature)/(FADETIME_MIN*2.0));
                if(curColorTemperature < targetColorTemperature) curColorTemperature = targetColorTemperature;
            }
        }
        else if(Time.hour() > startFadeHr && Time.hour() < HRENDFADE){
            if(curColorTemperature > ENDTEMP){
                curColorTemperature -= (double)(targetColorTemperature-ENDTEMP)/(120.0*(HRENDFADE-startFadeHr));
                if(curColorTemperature < ENDTEMP) curColorTemperature = ENDTEMP;
            }
        }
        strcontrol();
        minsElapsed++;
        timeAutoOn += 30000;
    }

    /////////////////////////////////////////////////
    ////////////  Main Button Press  ////////////////
    /////////////////////////////////////////////////
    else if(digitalRead(TOGGLE) == HIGH){
        AutoCTL = false;
        if(mode == 0){
            update = 1;
            //TPLink_Plug(plugIP1, 1);
            //TPLink_Plug(plugIP2, 1);
            TPLink_Bulb(bulbIP1,1,curStripBrightness*100,curColorTemperature*100);
            TPLink_Bulb(bulbIP2,1,curStripBrightness*100,curColorTemperature*100);
            TPLink_Bulb(bulbIP3,1,curStripBrightness*100,curColorTemperature*100);
            if(EEPROM.read(EEP_MODE_REG) == 2){
                mode = 2;
                fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 30, curStripBrightness);
            }
            else{
                mode = 1;
                fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 30, VAxisToPct(Vaxis));
            }
        }
        else
        {
            update = 1;
            //TPLink_Plug(plugIP1, 0);
            //TPLink_Plug(plugIP2, 0);
            //TPLink_Bulb(bulbIP1,0,0,2700);
            //TPLink_Bulb(bulbIP2,0,0,2700);
            //TPLink_Bulb(bulbIP3,0,0,2700);
            EEPROM.write(EEP_MODE_REG,(mode%2));
            mode = 0;
            strcontrol();
        }
        while(digitalRead(TOGGLE) == HIGH) delay(10);
    }

    /////////////////////////////////////////////////
    ///////////  Photoresistor Check  ///////////////
    /////////////////////////////////////////////////
    else if(analogRead(BRTSNS) < 220 && mode == 0)                //Reads Photoresistor
    {
        stripa.setPixelColor(NLIGHT_PXNUM,NLIGHT);
        stripa.show();
        delay(10);
    }

    /////////////////////////////////////////////////
    //////////  Joystick Left Gesture  //////////////
    /////////////////////////////////////////////////
    if(Haxis > 3400 && Haxis < 4000)
    {
        AutoCTL = false;
        if(mode > 0){
            mode--;
            strcontrol();
        }
        else{
            mode = MAXMODE;
            strcontrol();
        }
        EEPROM.write(EEP_MODE_REG,mode);
        digitalWrite(GRNLT, HIGH);
        while(analogRead(A0) > 3400 && analogRead(A0) < 4000)  delay(5);
        delay(500);
        digitalWrite(GRNLT, LOW);
    }

    /////////////////////////////////////////////////
    //////////  Joystick Right Gesture  /////////////
    /////////////////////////////////////////////////
    if(Haxis > 2700 && Haxis < 3000)
    {
        AutoCTL = false;
        if(mode < MAXMODE){
            mode++;
            strcontrol();
        }
        else{
            mode = 0;
            strcontrol();
        }
        EEPROM.write(EEP_MODE_REG,mode);
        digitalWrite(GRNLT, HIGH);
        while(analogRead(A0) > 2700 && analogRead(A0) < 3000)  delay(5);
        delay(500);
        digitalWrite(GRNLT, LOW);
    }
    delay(10);
}

int brewCoffee(const char *command)
{
    if(strcmp(command,"toggle")==0){    
        AutoCTL = false;
        if(mode == 0){
            if(EEPROM.read(EEP_MODE_REG) == 2){
                mode = 2;
                fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 30, curStripBrightness);
            }
            else{
                mode = 1;
                fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 30, VAxisToPct(Vaxis));
            }
        }
        else{
            AutoCTL = false;
            //TPLink_Bulb(bulbIP1,0,0,2700);
            //TPLink_Bulb(bulbIP2,0,0,2700);
            //TPLink_Bulb(bulbIP3,0,0,2700);
            //TPLink_Plug(plugIP1, 0);
            //TPLink_Plug(plugIP2, 0);
            //TPLink_Plug(plugIP3, 0);
            //TPLink_Plug(plugIP4, 0);
            EEPROM.write(EEP_MODE_REG,(mode%2));
            mode = 0;
            update = true;
            strcontrol();
        }
    }
    else if(strcmp(command,"evoff")==0){
        AutoCTL = false;
        //TPLink_Bulb(bulbIP1,0,0,2700);
        //TPLink_Bulb(bulbIP2,0,0,2700);
        //TPLink_Bulb(bulbIP3,0,0,2700);
        //TPLink_Plug(plugIP1, 0);
        //TPLink_Plug(plugIP2, 0);
        //TPLink_Plug(plugIP3, 0);
        //TPLink_Plug(plugIP4, 0);
        EEPROM.write(EEP_MODE_REG,(mode%2));
        mode = 0;
        update = true;
        strcontrol();            
    }
    else if(strcmp(command,"evon")==0){
        AutoCTL = false;
        TPLink_Bulb(bulbIP1,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP2,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP3,1,curStripBrightness*100,curColorTemperature*100);
        //TPLink_Plug(plugIP1, 1);
        //TPLink_Plug(plugIP2, 1);
        //TPLink_Plug(plugIP3, 1);
        //TPLink_Plug(plugIP4, 1);
        if(mode == 0){
            if(EEPROM.read(EEP_MODE_REG) == 2){
                mode = 2;
                fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 30, curStripBrightness);
            }
            else{
                mode = 1;
                fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 30, VAxisToPct(Vaxis));
            }
        }
    }
    else if(strcmp(command,"autoon")==0){
        AutoCTL = false;
        if(mode == 0){
            mode = 3;
            minsElapsed = 0;
            startFadeHr = Time.hour()+1;
            timeAutoOn = millis() + 30000;
            targetColorTemperature = curColorTemperature;
            curStripBrightness = ONBRT;
            curColorTemperature = ONTEMP;
            currentColor.setColorTemperature(curColorTemperature);
            fillStrip(true, true, true, 64, currentColor.red,currentColor.green,currentColor.blue, currentColor.green,currentColor.red,currentColor.blue, 0, curStripBrightness);
            TPLink_Bulb(bulbIP1,1,curStripBrightness*100,curColorTemperature*100);
            TPLink_Bulb(bulbIP2,1,curStripBrightness*100,curColorTemperature*100);
            TPLink_Bulb(bulbIP3,1,curStripBrightness*100,curColorTemperature*100);
            update = false;
        }
    }
    else if(strcmp(command,"bton")==0){
        scanEn = true;
    }
    else if(strcmp(command,"btoff")==0){
        scanEn = false;
    }
    else if(strcmp(command,"comon")==0){
        TPLink_Plug(plugIP3, 1);
        TPLink_Plug(plugIP4, 1);
    }
    else if(strcmp(command,"comoff")==0){
        TPLink_Plug(plugIP3, 0);
        TPLink_Plug(plugIP4, 0);
    }
    else if(strcmp(command,"plg1on")==0){
        TPLink_Plug(plugIP1, 1);
    }
    else if(strcmp(command,"plg2on")==0){
        TPLink_Plug(plugIP2, 1);
    }
    else if(strcmp(command,"plg3on")==0){
        TPLink_Plug(plugIP3, 1);
    }
    else if(strcmp(command,"plg4on")==0){
        TPLink_Plug(plugIP4, 1);
    }
    else if(strcmp(command,"plg1off")==0){
        TPLink_Plug(plugIP1, 0);
    }
    else if(strcmp(command,"plg2off")==0){
        TPLink_Plug(plugIP2, 0);
    }
    else if(strcmp(command,"plg3off")==0){
        TPLink_Plug(plugIP3, 0);
    }
    else if(strcmp(command,"plg4off")==0){
        TPLink_Plug(plugIP4, 0);
    }
    else if(strcmp(command,"pct25")==0){
        curStripBrightness = 0.25;
        TPLink_Bulb(bulbIP1,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP2,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP3,1,curStripBrightness*100,curColorTemperature*100);
    }
    else if(strcmp(command,"pct50")==0){
        curStripBrightness = 0.50;
        TPLink_Bulb(bulbIP1,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP2,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP3,1,curStripBrightness*100,curColorTemperature*100);
    }
    else if(strcmp(command,"pct75")==0){
        curStripBrightness = 0.75;
        TPLink_Bulb(bulbIP1,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP2,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP3,1,curStripBrightness*100,curColorTemperature*100);
    }
    else if(strcmp(command,"pct100")==0){
        curStripBrightness = 1;
        TPLink_Bulb(bulbIP1,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP2,1,curStripBrightness*100,curColorTemperature*100);
        TPLink_Bulb(bulbIP3,1,curStripBrightness*100,curColorTemperature*100);
    }
    return 1;
}

void scanResultCallback(const BleScanResult *scanResult, void *context) {
    //uint8_t buf[BLE_MAX_ADV_DATA_LEN];
    uint8_t bufName[BLE_MAX_ADV_DATA_LEN];
    BleAddress addr = scanResult->address;
    BleAdvertisingData advData = scanResult->advertisingData;
    for(int i = 0; i < 10; i++) bufName[i] = 255;
    scanResult->advertisingData.customData(bufName, BLE_MAX_ADV_DATA_LEN);
    char dataStr [10];
    sprintf(dataStr,"%x",advData);

    //Check if button is pressed and device is nearby and if an Apple device - then store device id if there is space
    /*if(digitalRead(TOGGLE) == HIGH && scanResult->rssi >= -28 && strcmp("21a0102", dataStr) == 0){
        if((numStoredDevices) < BT_MAXDEVICES){
            Log.info("PAIRED DEVICE IDs: %d %d", bufName[3], bufName[5]);
            EEPROM.write(numStoredDevices << 1, bufName[3]);
            EEPROM.write((numStoredDevices << 1) + 1, bufName[5]);
            storedKeys[numStoredDevices << 1] = bufName[3];
            storedKeys[(numStoredDevices << 1)+1] = bufName[5];
            numStoredDevices++;
            EEPROM.put(EEP_DEVCOUNT_REG, numStoredDevices);
            RGB.color(0, 0, 250);
        }
        else{
            Log.info("ERROR, ID MEMORY FULL");
            RGB.color(250, 30, 0);
        }
        while(digitalRead(TOGGLE) == HIGH) delay(5);
        RGB.color(0, 0, 0);
        return;
    }*/
    //if(scanResult->rssi >= -28) Log.info("FOUND CLOSE DEVICE %d %d", bufName[3], bufName[5]);
    //if(scanResult->rssi >= -28) Log.info("FOUND CLOSE DEVICE %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", bufName[0], bufName[1], bufName[2], bufName[3], bufName[4], bufName[5], bufName[6], bufName[7], bufName[8], bufName[9], bufName[10],bufName[11],bufName[12],bufName[13], bufName[14], bufName[15], bufName[16], bufName[17], bufName[18], bufName[19], bufName[20], bufName[21], bufName[22], bufName[23], bufName[24],bufName[25],bufName[26],bufName[27]);
    if(strcmp("21a0102", dataStr) == 0){
        uint8_t deviceNumber;
        for(deviceNumber = 0; deviceNumber < numStoredDevices; deviceNumber++){
            if(bufName[3] == storedKeys[deviceNumber << 1] && bufName[5] == storedKeys[(deviceNumber << 1)+1] && scanResult->rssi > (0-devRSSI[deviceNumber])){
                //RGB.color(0, 250, 250);
                Log.info("Found Device IDs: %d %d, RSSI: %d", storedKeys[deviceNumber << 1], storedKeys[(deviceNumber << 1)+1], scanResult->rssi);
                lastHeard = millis();
                if(reEnOnDiscover){
                    AutoCTL = true;
                    reEnOnDiscover = false;
                    Log.info("Found device, re-enabling automatic control");
                }
                
            }
        }
    }    
}

int setBulbColor(const char *command){
    int receivedVal;
    receivedVal = atoi(command);
    int RedVal = receivedVal%1000;
    int GreenVal = (receivedVal/1000)%1000;
    int BlueVal = (receivedVal/1000000)%1000;
    TPLink_RGB_Bulb(bulbIP2, 1, RedVal, GreenVal, BlueVal);
    return 1;
}

double VAxisToPct(int sliderVal){
    double tempVal = (double)Vaxis;
    tempVal = (tempVal-300.0)/1000.0;
    if(tempVal > 1.0) return 1.0;
    if(tempVal < 0.0) return 0.0;
    return tempVal;
}