// 3 Phase PWM sine
// (c) 2018 Norawit Nangsue
// Fixed comments from
// (c) 2016 C. Masenas
// Modified from original DDS from: 
// KHM 2009 /  Martin Nawrath

// table of 256 sine values / one sine period / stored in flash memory
PROGMEM const unsigned short SINE_X[]  = { 0,25,50,75,100,125,150,175,200,224,249,273,297,321,345,369,392,415,438,460,483,505,526,548,569,590,610,630,650,669,688,706,724,741,759,775,791,807,822,837,851,865,878,891,903,914,925,936,946,955,964,972,979,986,993,998,1004,1008,1012,1015,1018,1020,1022,1023,1023,1023,1022,1020,1018,1015,1012,1008,1003,998,992,985,978,971,963,954,944,934,924,913,901,889,876,863,849,835,820,805,789,773,756,739,722,704,685,666,647,627,607,587,566,545,524,502,480,458,435,412,389,366,342,318,294,270,246,221,197,172,147,122,97,72,47,22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
PROGMEM const unsigned short SINE_Y[]  = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,25,50,75,100,125,150,175,200,224,249,273,297,321,345,369,392,415,438,460,483,505,526,548,569,590,610,630,650,669,688,706,724,741,759,775,791,807,822,837,851,865,878,891,903,914,925,936,946,955,964,972,979,986,993,998,1004,1008,1012,1015,1018,1020,1022,1023,1023,1023,1022,1020,1018,1015,1012,1008,1003,998,992,985,978,971,963,954,944,934,924,913,901,889,876,863,849,835,820,805,789,773,756,739,722,704,685,666,647,627,607,587,566,545,524,502,480,458,435,412,389,366,342,318,294,270,246,221,197,172,147,122,97,72,47,22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
PROGMEM const unsigned short SINE_Z[]  = {889,876,863,849,835,820,805,789,773,756,739,722,704,685,666,647,627,607,587,566,545,524,502,480,458,435,412,389,366,342,318,294,270,246,221,197,172,147,122,97,72,47,22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,25,50,75,100,125,150,175,200,224,249,273,297,321,345,369,392,415,438,460,483,505,526,548,569,590,610,630,650,669,688,706,724,741,759,775,791,807,822,837,851,865,878,891,903,914,925,936,946,955,964,972,979,986,993,998,1004,1008,1012,1015,1018,1020,1022,1023,1023,1023,1022,1020,1018,1015,1012,1008,1003,998,992,985,978,971,963,954,944,934,924,913,901};


#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

//Digital Pin(Output)
#define X_PIN             5
#define IX_PIN            2
#define Y_PIN             3
#define IY_PIN            6
#define Z_PIN             7
#define IZ_PIN            8

//Using Timer5 as Spare pins
#define SPARE_PIN1        44
#define SPARE_PIN2        45
#define SPARE_PIN3        46

volatile float scalingFactorX = 1;
volatile float scalingFactorY = 1;
volatile float scalingFactorZ = 1;

volatile uint16_t X;
volatile uint16_t IX;
volatile uint16_t Y;
volatile uint16_t IY;
volatile uint16_t Z;
volatile uint16_t IZ;

float currentRead = 0;

volatile float freq=1;
const float refclk = 30.547  ;     //16 MHz/1023/2/256


//Variables used inside interrupt service declared as voilatile
volatile unsigned long sigma;  // Phase Accumulator
volatile unsigned long delta;  // Phase Increment
volatile int WerteA0;
volatile int WertA0;
volatile byte Anzahl;
byte phase, phaseI;


void setup()
{
  Serial.begin(115200);
  Serial.println("DDS Test");

  //PWM Pin
  pinMode(X_PIN, OUTPUT);
  pinMode(IX_PIN, OUTPUT);
  pinMode(Y_PIN, OUTPUT);
  pinMode(IY_PIN, OUTPUT);
  pinMode(Z_PIN, OUTPUT);
  pinMode(IZ_PIN, OUTPUT);
  

  GTCCR = (1<<TSM)|(1<<PSRSYNC); //Halt Syncronous Timers

  Setup_Timer5();
  Setup_Timer3();
  Setup_Timer4();
  TCNT5H = 0;
  TCNT5L = 0;
  TCNT3H = 0;
  TCNT3L = 0;
  TCNT4H = 0;
  TCNT4L = 0;

  GTCCR = 0; //Release All Timers
}

// the waveform index is the highest 8 bits of sigma
// choose refclk as freq to increment the lsb of the 8 highest bits
// for every call to the ISR of timer2 overflow
// the lsb of the 8 highest bits is 1<<24 (1LL<<24 for long integer literal)
  
void loop(){
  //Frequency
  changeFreq(50);
  if(WertA0 < 440 && scalingFactorY < 1.3) scalingFactorX += 0.005;
  if(WertA0 > 445 && scalingFactorY > 0.8) scalingFactorX -= 0.005;
  delay(50);
}


void changeFreq(float _freq){
  cbi (TIMSK5,TOIE5);           // Disable Timer5 Overflow Detect
  freq = _freq;
  delta=(1LL<<24)*freq/refclk;  // Update Phase Increment
  sbi (TIMSK5,TOIE5);           // Enable Timer5 Overflow Detect
} 


// Timer5 setup
// Set prescaler to 1,  Phase Correct PWM
void Setup_Timer5() {
  //Timer5 Clock Prescaler to : 1
  sbi (TCCR5B, CS50);
  cbi (TCCR5B, CS51);
  cbi (TCCR5B, CS52);

  sbi (TCCR5A, COM5A0);  //Set OC5A on Compare Match, PWM pin 44
  sbi (TCCR5A, COM5A1);
  sbi (TCCR5A, COM5B0);  //Set OC5B on Compare Match, PWM pin 45
  sbi (TCCR5A, COM5B1);
  sbi (TCCR5A, COM5C0);  //Set OC5C on Compare Match, PWM pin 46
  sbi (TCCR5A, COM5C1);
  
  //Mode 3, 10 Bit Phase Correct PWM
  sbi (TCCR5A, WGM50);   
  sbi (TCCR5A, WGM51);
  cbi (TCCR5B, WGM52);
  cbi (TCCR5B, WGM53);

  sbi (TIMSK5,TOIE5);    // Enable Overflow Detect
}

// Timer3 Setup (Set pin 5, 2 and 3)
// Set prescaler to 1, PWM mode to phase correct PWM,  16000000/2046 = 7820.14 Hz clock
void Setup_Timer3() {
// Timer3 Clock Prescaler to : 1
  sbi (TCCR3B, CS30);
  cbi (TCCR3B, CS31);
  cbi (TCCR3B, CS32);

  sbi (TCCR3A, COM3A0);  //Set OC3A on Compare Match, PWM pin 5
  sbi (TCCR3A, COM3A1);
  sbi (TCCR3A, COM3B0);  //Set OC3B on Compare Match, PWM pin 2
  sbi (TCCR3A, COM3B1);
  sbi (TCCR3A, COM3C0);  //Set OC3C on Compare Match, PWM pin 3
  sbi (TCCR3A, COM3C1);

  // Mode 3: 10 Bit Phase correct PWM
  sbi (TCCR3A, WGM30);  
  sbi (TCCR3A, WGM31);
  cbi (TCCR3B, WGM32);
  cbi (TCCR3B, WGM33);
}

void Setup_Timer4() {
  //Timer4 Clock Prescaler to : 1
  sbi (TCCR4B, CS40);
  cbi (TCCR4B, CS41);
  cbi (TCCR4B, CS42);

  sbi (TCCR4A, COM4A0);  //Set OC4A on Compare Match, PWM pin 6
  sbi (TCCR4A, COM4A1);
  sbi (TCCR4A, COM4B0);  //Set OC4B on Compare Match, PWM pin 7
  sbi (TCCR4A, COM4B1);
  sbi (TCCR4A, COM4C0);  //Set OC4C on Compare Match, PWM pin 8
  sbi (TCCR4A, COM4C1);

  //Mode 3: 10 Bit Phase correct PWM
  sbi (TCCR4A, WGM40);  
  sbi (TCCR4A, WGM41);
  cbi (TCCR4B, WGM42);
  cbi (TCCR4B, WGM43);
}


// Timer5 Interrupt Service at 7820 Hz
ISR(TIMER5_OVF_vect) {
  //use global variable with interrupt -> use volatile variable
  //float _scalingFactor = scalingFactor; //For optimization purpose because volatile variable would be load every time
  /*
  Spare Parts: Timer 5
  OCR5A: Pin 44
  OCR5B: Pin 45
  OCR5C: Pin 46

  USE Pin 45 for Hotfix
  */

  sigma = sigma + delta; // soft DDS, phase accu with 32 bits

  //Check whether it's 0 or not before multiplication which could reduce the code computing complexity. 
  phase = sigma >> 24;
  phaseI = phase + 128;
  
  X = pgm_read_word_near(SINE_X + phase);
  if(X != 0)
  {
    if(X == 75)
    {
      WerteA0 += analogRead(0);
      Anzahl ++;
      if(Anzahl > 10)
      {
        WertA0 = WerteA0 / 10;
        Anzahl = 0;
        WerteA0 = 0;
      }
    }
    X = X * scalingFactorX;
    IX = 0;
  }else{
    IX = scalingFactorX * pgm_read_word_near(SINE_X + phaseI);
  }
  
  Y = pgm_read_word_near(SINE_Y + phase);
  if(Y != 0)
  {
    Y = Y * scalingFactorY;
    IY = 0;
  }else{//Y == 0
    IY = scalingFactorY * pgm_read_word_near(SINE_Y + phaseI);
  }
  
  Z = pgm_read_word_near(SINE_Z + phase);
  if(Z != 0)
  {
    Z = Z * scalingFactorZ;
    IZ = 0;
  }else{//Z == 0
    IZ = scalingFactorZ * pgm_read_word_near(SINE_Z + phaseI);
  }
  
  OCR3A=1023 - IX;  // pwm pin 5
  OCR3B=1023 - X;   // pwm pin 2
  OCR3C=1023 - Y;   // pwm pin 3
  OCR4A=1023 - IY;  // pwm pin 6
  OCR4B=1023 - Z;   // pwm pin 7
  OCR4C=1023 - IZ;  // pwm pin 8
}
