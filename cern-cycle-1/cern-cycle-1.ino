// This is a demonstration on how to use an input device to trigger changes on your neo pixels.
// You should wire a momentary push button to connect from ground to a digital IO pin.  When you
// press the button it will change to a new pixel animation.  Note that you need to press the
// button once to start the first animation!

#include <Adafruit_NeoPixel.h>

#define S1   PB0  // S1 Tact switch
#define S2   PB2  // S2 Tact switch
#define PIXEL_PIN PB1  // PB1 is connected to Neopixels

#define PIXELS 16

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool oS1 = HIGH;
bool oS2 = HIGH;
uint32_t position[PIXELS]; // 32-bit colour on each spot

void setup() {
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  for (uint8_t i = 0; i < PIXELS - 1; i++){
    position[i] = 0;
  }
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Get current button state.
  bool nS1 = digitalRead(S1);
  bool nS2 = digitalRead(S2);
  uint32_t colour = 0;

  // Check if state changed from high to low (S1 press).
  if ((nS1 == LOW && oS1 == HIGH)||(nS2 == LOW && oS2 == HIGH)) {
    // Short delay to debounce button.
    delay(20);
    // Check if button is still low after debounce.
    nS1 = digitalRead(S1);
    nS2 = digitalRead(S2);
    //position[0] = 0;
    if (nS1 == LOW) {
      colour = 16; // Add blue, (#00003F)
    }
    if (nS2 == LOW) {
      colour = 1048576; // Add red, (#3F0000)
    }
    if (nS1 == LOW && nS2 == LOW && oS1 == HIGH && oS2 == HIGH){
      colour = 4096; // Add green (#003F00)
    }
    position[0] = position[0] + colour;
    if (position[0] > 3158064) {
      position[0] = 0;
    }
  }

  // Set the last button state to the old state.
  oS1 = nS1;
  oS2 = nS2;
  drawDisplay();
  rotateRing();
}

void drawDisplay(){
  for (uint8_t i=0; i< PIXELS; i++){
    uint8_t r = splitColor(position[i],'r');
    uint8_t g = splitColor(position[i],'g');
    uint8_t b = splitColor(position[i],'b');
    if (position[i] > 0){
      strip.setPixelColor(i,r,g,b);
    } else {
      strip.setPixelColor(i,0,0,0);
    }
  }
  strip.show();
  delay(30);
}

void rotateRing(){
  uint32_t tmpbuf = 0; // holds the value of position 11
  uint8_t dir = 0; // holds pixel direction
  tmpbuf = position[PIXELS - 1];
  for (uint8_t i=PIXELS - 1; i>0; i--){
    //dir = (uint8_t)(position[i] >> 24);
    //if(dir > 0){
      position[i] = position[i - 1];
    //}
  }
  position[0] = tmpbuf;
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
