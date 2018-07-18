// Large Hadron Collider Simulator
//
// Press S1 to drop in a clockwise proton (blue)
// Press S2 to drop in an anticlockwise proton (red)
// Press both buttons to trigger the magnet

#include <Adafruit_NeoPixel.h>
#include <stdlib.h> //rand
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define S1   PB0  // S1 Tact switch
#define S2   PB2  // S2 Tact switch
#define PIXEL_PIN PB1  // PB1 is connected to Neopixels

uint32_t magenta = 0x1f001fUL; //2031647UL;
uint32_t cyan = 0x001f1fUL; //7967UL;
uint32_t yellow = 0x1f1f00UL; //2031647UL;
uint32_t white = 0x1f1f1fUL; //2039583UL;

#define PIXELS 16

uint32_t colours[10] = {magenta, yellow, yellow, yellow, cyan, cyan, yellow, yellow, white, white};

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool oS1 = HIGH;
bool oS2 = HIGH;
bool magnets = LOW;
bool lmagnets = LOW;
bool capture = LOW;
//uint32_t layer0[PIXELS]; // Our combination of all layers.
uint32_t layer1[PIXELS]; // 32-bit colour on each spot, anti-clockwise
uint32_t layer2[PIXELS]; // 32-bit colour on each spot, clockwise
uint32_t layer3[PIXELS]; // 32-bit colour on each spot, magnets
uint32_t wait = 360;

void setup() {
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  initRand();
  clearLayers();
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Get current button state.
  bool nS1 = digitalRead(S1);
  bool nS2 = digitalRead(S2);
  uint32_t colour = 0;

  // Check if state changed from high to low (S1 press).
  if (magnets == LOW){
    if ((nS1 == LOW && oS1 == HIGH)||(nS2 == LOW && oS2 == HIGH)) {
      // Short delay to debounce button.
      delay(20);
      // Check if button is still low after debounce.
      nS1 = digitalRead(S1);
      nS2 = digitalRead(S2);
      colour = getColour();
      //layer1[0] = 0;
      if (nS1 == LOW) {
        layer2[0] = 0x00001FUL;
      }
      if (nS2 == LOW) {
        layer1[0] = 0x1F0000UL;
      }
  
      if (wait > 30) wait = wait - 10;
    }
  }

  // Prime magnets
  if (nS1 == LOW && nS2 == LOW){
    //layer3[1] = 0x001F00UL;
    //layer3[2] = 0x001F00UL;
    if (magnets == LOW){
      lmagnets = LOW;
    }
    magnets = HIGH;
  } else {
    //layer3[1] = 0x000000UL;
    //layer3[2] = 0x000000UL;
    if (magnets == HIGH){
      lmagnets = HIGH;
    }
    magnets = LOW;
  }
  // Set the last button state to the old state.
  oS1 = nS1;
  oS2 = nS2;

  drawDisplay();
  if (magnets == LOW && lmagnets == HIGH){
    capture = HIGH;
    lmagnets = LOW;
    fin2();
  }
  rotateRing();
}

void clearLayers(){
  for (uint8_t i = 0; i < PIXELS; i++){
    layer1[i] = 0;
    layer2[i] = 0;
    layer3[i] = 0;
  }
}

void drawDisplay(){
  for (uint8_t i=0; i< PIXELS; i++){
    uint32_t c = layer1[i] + layer2[i] + layer3[i];
    uint8_t r = splitColor(c,'r');
    uint8_t g = splitColor(c,'g');
    uint8_t b = splitColor(c,'b');
    strip.setPixelColor(i,r,g,b);

    /*if (c > 0){
      strip.setPixelColor(i,r,g,b);
    } else {
      strip.setPixelColor(i,0,0,0);
    }*/
  }
  if (magnets == HIGH){
    strip.setPixelColor(1,0,0x1f,0);
    strip.setPixelColor(2,0,0x1f,0);
    strip.setPixelColor(5,0,0x1f,0);
    strip.setPixelColor(6,0,0x1f,0);
    strip.setPixelColor(9,0,0x1f,0);
    strip.setPixelColor(10,0,0x1f,0);
    strip.setPixelColor(13,0,0x1f,0);
    strip.setPixelColor(14,0,0x1f,0);
  }
  strip.show();
  delay(wait);
}

void rotateRing(){
  uint32_t tmpbuf = 0; // holds the value of layer1
  // Clear segments in magnets if magnets are down
  if (magnets == HIGH){
    annihilate(1,0);
    annihilate(2,3);
    annihilate(5,4);
    annihilate(6,7);
    annihilate(9,8);
    annihilate(10,11);
    annihilate(13,12);
    annihilate(14,15);
  }
  
  // Start with anti-clockwise layers
  tmpbuf = layer1[PIXELS - 1];
  for (uint8_t i=PIXELS - 1; i>0; i--){
      layer1[i] = layer1[i - 1];
  }
  layer1[0] = tmpbuf;
  // Clockwise layers
  tmpbuf = layer2[0];
  for (uint8_t i=0; i < PIXELS - 1; i++){
      layer2[i] = layer2[i + 1];
  }
  layer2[PIXELS -1] = tmpbuf;
}

uint8_t splitColor ( uint32_t c, char value )
{
  switch ( value ) {
    case 'r': return (uint8_t)(c >> 16);
    case 'g': return (uint8_t)(c >>  8);
    case 'b': return (uint8_t)(c >>  0);
    default:  return 0;
  }
}

/*uint32_t getColour(){
  uint32_t r = (rand() / (RAND_MAX / 0x1FUL + 1)) << 16;//(uint8_t)( 63UL << 16);
  uint16_t g = (rand() / (RAND_MAX / 0x1F + 1)) << 8;
  uint8_t b = (rand() / (RAND_MAX / 0x1F + 1)) << 0;
  //uint32_t red   = (uint32_t)(63UL << 16);//4128768; //63 << 16;
  //uint16_t green = (uint8_t)(63 <<  8);
  //uint8_t  blue  = (uint8_t)(63 <<  0);
  return r + g + b;//+green+blue;
}*/

uint32_t getColour(){
  uint8_t i = rand() / (RAND_MAX / 10 );
  uint32_t c = colours[i];
  //uint32_t r = (rand() / (RAND_MAX / 0x1FUL + 1)) << 16;//(uint8_t)( 63UL << 16);
  //uint16_t g = (rand() / (RAND_MAX / 0x1F + 1)) << 8;
  //uint8_t b = (rand() / (RAND_MAX / 0x1F + 1)) << 0;
  //uint32_t red   = (uint32_t)(63UL << 16);//4128768; //63 << 16;
  //uint16_t green = (uint8_t)(63 <<  8);
  //uint8_t  blue  = (uint8_t)(63 <<  0);
  return c;//+green+blue;
}

void initRand(void)
{
        uint32_t state;
        static uint32_t EEMEM sstate;

        state = eeprom_read_dword(&sstate);

        // Check if it's unwritten EEPROM (first time).
        if (state == 0xffffffUL)
                state = 0xC0D3CAFE;
        srand(state);
        eeprom_write_dword(&sstate, rand());
} 

void annihilate(uint8_t loc, uint32_t flashloc){
  if (layer1[loc] != 0 && layer2[loc] !=0){
      layer3[flashloc] = getColour();
      layer1[loc] = 0;
      layer2[loc] = 0;
      if (wait < 360) wait = wait + 20;
    } //else {
      //layer3[flashloc] = 0;
    //}
}

uint32_t fade(uint32_t c){
    uint8_t r = splitColor(c,'r');
    uint8_t g = splitColor(c,'g');
    uint8_t b = splitColor(c,'b');
    if (r > 0) r--;
    if (g > 0) g--;
    if (b > 0) b--;
    uint32_t r2 = r;
    r2 = r2 << 16;
    c = r2 + (uint16_t)(g <<  8) + (uint8_t)(b <<  0);
    return c;
}

void fin(){
  uint32_t c = 1;
  while(c > 0){
    c = 0;
    for (uint8_t i=0; i< PIXELS; i++){
      layer1[i] = 0; //fade(layer1[i]);
      layer2[i] = 0; //fade(layer2[i]);
      layer3[i] = fade(layer3[i]);
      c = c + layer1[i] + layer2[i] + layer3[i];
      if (c > 0) c = 1;
    }
    drawDisplay();
    delay(wait);
    wait = wait + 10;
  }

  capture = LOW;
  clearLayers();
  wait = 360;
}

void fin2(){
  oS1 = HIGH; 
  oS2 = HIGH;
  uint32_t c = 1;
  uint8_t t = 0;
  for (uint8_t i=0; i< PIXELS; i++){
    if (layer3[i] > 0) t++;
    layer1[i] = 0; //fade(layer1[i]);
    layer2[i] = 0; //fade(layer2[i]);
    
  }
  drawDisplay();
  while(c > 0){
    bool nS1 = digitalRead(S1);
    bool nS2 = digitalRead(S2);
    if (nS1 == LOW && nS2 == LOW){
      oS1 = LOW;
      oS2 = LOW;
    }
    if (nS1 == HIGH && nS2 == HIGH && oS1 == LOW && oS2 == LOW){
      c = 0;
      clearLayers();
      wait = 360;      
    }
    if (t < 2){
      c = 0;
      clearLayers();
      wait = 360; 
    }
  }

}

