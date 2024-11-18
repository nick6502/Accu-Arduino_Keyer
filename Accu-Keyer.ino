/*morseSendString

Accu-Keyer on the Arduino by WA5BDU
Started December 18, 2023
Working January 11, 2024
V1.4 complete November 18, 2024

NOTE: THE ACCU-KEYER EMULATION CODE IS VERY DIFFICULT TO TROUBLESHOOT
The additional features code is OK to revise as desired.

V1.0 working with speed pot included. No ACS yet.
V1.1 adds ACS (auto character space) and manual or hand key input
V1.2 1/30/2024 - I want to add battery monitoring & LED controlled by MCU 
I saved V1.2 as a BAK file because it's working fine and I want to pursue
why ACS seems to be always on. 5/2/2024

V1.3 changes intended to fix issue with ACS being always on. Fixed! 5/4/2024

V1.4 fix the source code so KeyOut_i is always maintained as the inverse
     of KeyOut, which was my intent. Starting 11/13/2024 Also added
     announcement of revision # to startup message option.
     

The WB4VVF Accu-Keyer was published in the August 1973 QST and
designed by James Garrett WB4VVF. It appeared in some issues of
the Handbook, including 1976.

K7MEM has some good re-drawn schematics and other info on his website
There are a few errors or discrepancies though.

Simulating the accu-keyer with logic statements. WA5BDU 12/18/2023

Logic in C:
&& is AND
|| is OR
! is NOT

D flip-flop, as in 7474:

Data on the D input goes to Q output when clock C goes LOW to HIGH
CLR and SET are normally HIGH and can't both go LOW at same time

If CLR goes low (false), Q goes low
If SET goes low, Q goes high
If you use CLR or SET to make Q low or high and then you take CLR or SET
back to high, does Q stay where you just put it?  Yes!

Clock circuit:

It is free-running, producing positive going pulses across R6 of varying
duration depending on the speed setting. I call that voltage CLOCK. The
duty cycle is not symmetrical. The time between leading edges matters.

A positive (HIGH) voltage into CR1 stops the clock. I call that signal RST
So a LOW RST will start the clock.

The clock output is LOW in its idle state. When triggered, it stays LOW
for the timing period, then makes a fast LOW to HIGH and back to LOW 
transition. Actually, the full timing period includes the HIGH and LOW times,
but the LOW time is about 99% of the total period. 

Note that the article and K7MEM timing diagrams show the idle clock being
high. It doens't really matter much, but my LTspice model shows it's low.

The circuit consists only of NAND gates, plus D flip-flops, plus the clock
and output drivers. Outputs of the NAND gate sections are referred to simply
by the gate number. So U1A refers to pin 3 of U1. And when a flip-flop
has Q and -Q outputs, I'm referring to the Q output. So 'U5B' refers to the
Q output of U5B.
There are also paddle inputs, which will be actual input pins of the Arduino.

In Morse, one dit is 60 ms long at 20 WPM. My dit time is called ditTime

My intent is to just ripple through all definitions repeatedly and see what
comes out. Things that happen externally are the paddle states and the clock
transitions. For clock transitions, only up pulses matter. If the clock is
enabled by RST = false, it runs and is checked after each loop. When DIT-TIME is
reached, clocked logic starts with the U4A and U5A gates and propagates through.
A new counting period is started unless RST goes high (true).

Analog Pot speed control:

I'm using input A1. I wanted to automatically detect no pot connected, but
found that leaving A1 disconnected results in random readings. I'm reading
422. So my next solution will be to require grounding A1 for no pot, and
when I read '0', I'll default to the hard coded value of WPM.

As an alternate, a flag called UsePot will be true when one is used and
false when it is to be ignored.

ACS Switch: The state in this circuit is the opposite of that in the hardware
keyer. Meaning, I could have chosen either state to signal ACS ON, and I chose
the closed state, grounding ACSPin (5) to turn ACS ON. But in the hardware
version, the switch is open when ACS if ON.

Every 100th pass through the main loop, the speed pot is checked, unless 
keying is taking place. The state of the ACS switch is also checked.

After a defined period of idleness, a beeping tone is given to remind the user
that they keyer is ON - to prevent running down the battery. If the keyer
has been idle for > 1 minute, the battery voltage will be checked and if it
is below the low setpoint BATT_LO_ALM, an alert sound will be sounded, 
followed by Morse 'B'.

V1.3 Rules for U5B. I'm going put all the non-clocked parts in the main loop
and the one event where U5B gets clocked inside the clock() function.

1) ACS OFF, then U5B = U6A
2) ACS ON, then IF U6A = H, U5B = H
           If U6A = L and U4A goes high (clocking), U5B = U6A = H
So for V1.3, I remove all U5B = statements and replace with the above


*/

#define Dash_In 3
#define Dot_In 2
#define KeyOut 4 // HIGH when key down, then opto-isolator makes LOW
#define KeyOut_i 6 // inverse key out - HIGH when key up
#define ACSPin 5 // Ground for ACS ON
#define HandKey 7 // Ground for manual keying or TUNE
#define TestPin 8 // I/O pin used for toubleshooting / development V1.3
#define LEDPIN 13 // This is an on-board LED plus external panel mtd one
#define spkrpin 9
#define pitch 550 // USERS: define your desired sidetone pitch here
#define WPM 20 // USERS: default speed for when no pot is used
#define MSG_WPM 20 // USERS: Speed for messages from keyer
#define POT_ADC A1	
#define BATT_ADC A0 // V1.2
#define ADC_REF 5.05 // V1.2 USERS needs to experimentally verify = Vcc
#define BATT_LO_ALM 3.0 // low battery alarm V1.3 USERS set as per your battery
#define SPD_LIMIT_LO 10.0 // USERS - set as desired, about 8 WPM is low limit
#define SPD_LIMIT_HI 40.0 // USERS - set as desired
#define UsePot true // USERS edit to 'false' if no speed pot will be used
#define IDLE_MS 300000 // USERS - this is # of milliseconds idle before alert beep 

// The intent is to save the battery by alterting the user to turn the keyer off
// 300000 ms = 300 s = 5 minutes
// USER can also ground pin A1 if no speed pot, then default speed is WPM
// as defined above

bool U1A, U1B, U1C, U1D, U2A, U2B, U2C, U2D;
bool U6A, U6B, U6C, U6D, U7A, U7B;
bool U3A;
bool U3B;
bool U4A;
bool U4B;
bool U5A;
bool U5B;
bool U1C_clear_flag = false; // for delay on clear U3A
//bool RST; reset is !U5B
bool clock_active = false;
bool keyDown = false; // V1.2 - keep state so don't have to hammer on pins
bool ACS = false; // auto-char space. True if switch open, false if closed - in original TTL circuit
bool latch = false;
bool dashLatch = false;
uint8_t _5count = 0;
bool battmon = true; // USER make true if battery monitor is used, else false

////bool Dot_In_S = true;  // Vx.xx 1-2-24 for testing
bool Dash_In_S = true;  // Vx.xx 1-2-24 for testing
bool DoTransmit = true; // "true" keys TX on keydown, "false" tone only V1.2
bool DoTransmitMSG = false; // V1.2 - keyer messages don't close key line
bool SideTone = true; // true means DO generate sidetone V1.2 USERS
bool idle_alert = true; // USERS - make false if you don't want idle time beep
bool manual = false; // false - normal keyer, true - paddle acts as straight key

unsigned long timing_start; // Initial reading of millis() for clock
unsigned long timing_test; // Vx.xx 1-2-24 for testing
unsigned long CycleCount = 0; // V1.1a for finding speed of main loop
unsigned long timeIdle; // V1.2 see how long keyer has been idle

float speed_x; 
uint8_t speed;
uint8_t ditTime = 121; // 120 is 60 ms per dit at 20 WPM
uint8_t ditTimeMsg = 80; // for keyer messages, calc from MSG_WPM
float ditTimeF; // float version, for calculation
char SpeedString[] = "00 WPM"; // text equivalent of speed
char VersionString[] = " V1R4"; // V1.4 - add to info text NOTE: CAPS ONLY!!
uint16_t clockCount = 0;
uint16_t clockCountOld = 0;
uint8_t PrintTimes = 0;
uint16_t loopCount; // # times through main loop
uint8_t pot_count = 0; // main loop trips before doing ADC read
uint16_t oldADC;
uint16_t newADC;
float batteryvolts;
uint8_t keyDNstate = HIGH; // V1.2 HIGH on key down to drive opto
uint8_t keyUPstate = LOW; // V1.2 V1.4 low turns off opto (just comment change)
//uint16_t v_pitch = pitch; // v1.1a need a variable pitch so I can 'vary' it
//uint16_t delta_p = 0; // V1.1a, going to vary pitch to tell me if ACS is on

//    Function Declarations

void defaultSpeed();
void readSpeedPot();
void start_clock();
//void printStuff();
//void printAll();
void clock();
void readBatt();
void send_code(int pointtocode); // V1.2 adds char to Morse sender
                                
void alertSound();
                                 

void dit();
void dah();
void space();
void char_space();
void word_space();
void do_report();
void morseSendString(char mstring[]);
bool batteryCheck();
void do_manual();


// V1.2: USER: The greeting text below is sent when the keyer starts up. You
// can change it to what you want. It must end with the asterisk. Suggest 
// short message such as TU, HI, OK, QRV, R, EE (dit-dit) or maybe your call.

char greeting[] =  "EE*"; // keyer says this on start-up '*' marks end USERS
//char send_now = 'X';
uint8_t greet_ptr;



// ********************** S E T U P *****************************************

void setup() {
  pinMode(Dash_In, INPUT_PULLUP);
  pinMode(Dot_In, INPUT_PULLUP);
  pinMode(ACSPin, INPUT_PULLUP); // V1.1
  pinMode(LEDPIN, OUTPUT);
  pinMode(KeyOut, OUTPUT);
  pinMode(KeyOut_i, OUTPUT); // V1.1a
  pinMode(spkrpin, OUTPUT);
  pinMode(HandKey, INPUT_PULLUP); // V1.1a
  // pinMode(TestPin, OUTPUT); // V1.3, allow program to signal a condition V1.4 out

  digitalWrite(LEDPIN, HIGH); // LED will normally function as "power on" ind
 // digitalWrite(TestPin, LOW); // V1.3 V1.4 - out

   //Serial.begin(9600); // V1.1a REMOVE WHEN NOT BEING USED FOR TESTING
  // Serial.println("Running Accu-Arduino");

  ditTimeMsg = 1200/MSG_WPM; // dit time for keyer messages

  // if(oldADC != 0); // V1.4 this has no function so I commented it out
  //Serial.println();
 // Serial.println("   START");
 // Serial.println();

  ditTimeF = 20.0/WPM * 60.0; // F for floating point version
  ditTime = (uint8_t)ditTimeF;

  // From figure 3 initial conditions

  U7B = false;
  U6B = false;
  U6D = false;
  U3A = true;
  U3B = true;
  U4A = false;
  U5A = false;
  U4B = false;
  U5B = false;
  U1A = false; // Vx.x 1-7-23
  U2C = false; // ""
  U1B = true; // ""
  U2D = true; // ""


  oldADC = analogRead(POT_ADC);
 // Serial.print("Initial ADC: ");
 // Serial.println(oldADC);
  if(UsePot && (!(oldADC < 5))) // V1.4 instead of 0, use < 5 for no pot
    {
      if(oldADC > 15) // V1.4 making fake movement a litlte more certain
      {oldADC -= 15;}
      else {oldADC += 15;} // Here I fake a pot movement of > 10 for readSpeedPot()
   // Serial.print("NewOldADC: ");
   // Serial.println(oldADC);

    readSpeedPot();
   // Serial.print("ditTime first: ");
   // Serial.println(ditTime);
    }
    else defaultSpeed();
   // Serial.print("ditTime changed? ");
   // Serial.println(ditTime);

    if(digitalRead(ACSPin) == LOW) 
    {ACS = true; }
    // digitalWrite(TestPin, HIGH); } // V1.4 out

  // V1.2 check for both paddles closed
  if(!digitalRead(Dot_In) && !digitalRead(Dash_In))
  {
    while(!digitalRead(Dot_In) && !digitalRead(Dash_In)); // V1.2 wait for them to be released
    do_report();
  }
  else if (!digitalRead(Dash_In)) // if Dash paddle alone is held at start-up
   {
    while(!digitalRead(Dash_In)); // V1.2 wait for paddle to be released
    SideTone = false; // turn off sidetone, except for messages
  }
  else if(!digitalRead(Dot_In))
  {
    while(!digitalRead(Dot_In));
    manual = true; // V1.2 paddle acts as stright key
  }


 // do_report(); // for testing
  timing_test = millis(); // V1.1a test

  batteryCheck(); // V1.2 read battery and alarm if low
 

  delay(500);
  greet_ptr = 0;
  while(greeting[greet_ptr] != '*')
  {

 // send_now = greeting[greet_ptr];
  send_code(greeting[greet_ptr]);
  greet_ptr++;
  }

}

/* **************************************************************************

                       M A I N   L O O P

  ***************************************************************************
  */

void loop() 
{

  if(manual) do_manual(); // V1.2 hand key mode

  loopCount++; // Will be cleared to 0 every time the clock runs

// Below - every 65000 passes through loop, check idle time and if it is
// > than IDLE_MS, give the alert beep
// Note: Added a battery voltage check after 65100 times through the loop
// It also uses timeIdle and will reset it when voltage is low, so 
// battery low volts will preempt idle time, which is OK because we still
// get a sound every minute.

  if((loopCount == 65000) && idle_alert)
  {
    if(millis() - timeIdle > IDLE_MS) 
    {
      alertSound();
      timeIdle = millis();
    }
  }

  if((loopCount == 65100) && battmon)
  {
     if(millis() - timeIdle > 60000) // if idle > 1 minute, check battery volts
    {
       digitalWrite(LEDPIN, HIGH); // Also, LED goes back to power on indication
      if(batteryCheck()) timeIdle = millis();

    }   
  }
  /*
  CycleCount++;

  if(millis() - timing_test >= 100 && !latch)
  {
    Serial.print("Loops in 100ms = ");
    Serial.println(CycleCount);
    latch = true;
    // Note: Returned 2081 in 100ms or 20,810 per second or 48 us per loop
  }
*/

// I'll check the pot every 100th pass if the keyer clock is not active
// V1.1, I'll also check the ACS pin at that time

pot_count++;
if((pot_count > 100) && !clock_active)
{
  if(UsePot) readSpeedPot(); // V1.1a - move UsePot to affect this line only
  pot_count = 0;
  if(digitalRead(ACSPin) == LOW) 
  {ACS = true;}
  // digitalWrite(TestPin, HIGH); } // V1.3 v1.4 - out
 
  else {
    ACS = false;}
   // digitalWrite(TestPin, LOW);} // V1.1b V1.4 out
   
}

if(clock_active)
{
  if((millis() - timing_start) >= ditTime)
  {
    // Here, clock time has expired and a low-to high event occurred
    // so the routine for that event is called and the timer is 
    // reset.

    clockCount++;

    timing_start = millis(); // start a new period if clock running
    // if the clock timed out, a + spike was produced and sent to U4A 
    // & U5A clock terminals.

    clock();

  }
}


// Below is true regardless of ACS: If U6A is HIGH (U6C LOW), then U5B is HIGH

  U6C = !U6A; // V12-30-2
 

  if(!ACS) U5B = U6A; // V1.3 these three lines
  if(ACS & U6A) U5B = true;
  // if(U5B) start_clock(); // covered below ...

  // The intent below is to clear U3A on the second time through the loop, 
  // to emulate the 150 ohm resistor and 0.001 uF capacitor delay

    if(!U1C) // U1C clears U3A when low - tricky with R/C lag
 
    {
      if(U1C_clear_flag)
      {
      U3A = false;
      U1C_clear_flag = false;
      }
     else U1C_clear_flag = true;
    }
    else U1C_clear_flag = false; // Where U1C was NOT LOW (false)



  if(!U5B) // stop clock? Vx.x 1-11-24 also, U5A clear which clears U4A, U4B
  {
    clock_active = false;
    U5A = false; // cleared by !U5B 1-11-24 these three lines finally fixed it!
    U4A = false; // cleared by !U5A
    U4B = false; // cleared by !U5A
  }
  else // else, we have a run clock signal
  {
    start_clock();
  }



  U1A = !(digitalRead(Dash_In) && U1B);

  if(U1A) dashLatch = true; // Vx.xx 1-8-24
  U1B = !(U1A && U3A);
  U2C = !(digitalRead(Dot_In) && U2D);
  if(U2C) clockCount = 0; // Vx.xx 1-8-24 - don't start counting until dash paddle
  U2D = !(U2C && U3B); // 1-2-24 MOVED UP



  U1D = !(U1A && U2D);
  U1C = !(U1A && !U5B);
 
  U2B = !(U2C && U1B);
  U2A = !(U2C && !U5B);
  if(!U2A) U3B = false; // clear pin low on U3B

  U6B = !(!U4B && !U3A);
  U6D = !(!U5A && !U3B);

  U7A = !(U6B && U6D && U6A);

  U6A = !(U3A && U3B);
  U7B = !(U6B && U6D && digitalRead(HandKey)); // V1.1a
  // keyDown = U7B; vx.xx
  U6C = !U6A;

// They keyer's output is in the state of gate U7B. When HIGH, it's key down
// When it's LOW,it's key up

  if(U7B)
  {
    if(!keyDown) digitalWrite(LEDPIN, HIGH); // V1.2x
    if(SideTone) tone(spkrpin, pitch); // V1.2 add 'no sidetone' option
    digitalWrite(KeyOut, keyDNstate); // V1.1a V1.2 HIGH on keydown to drive opto V1.4
    digitalWrite(KeyOut_i, keyUPstate); // V1.4 - was HIGH
    keyDown = true; 
  }
  else
  {
    if(keyDown) digitalWrite(LEDPIN, LOW); // V1.2x
    noTone(spkrpin);
    digitalWrite(KeyOut_i, keyDNstate ); // V1.1a V1.4
    digitalWrite(KeyOut, keyUPstate); // V1.2 HIGH on keydown to drive opto V1.4 
    keyDown = false;
  }




if(U5B)
{
  start_clock();
}
else clock_active = false; // Vx.xx

} 


// ********** END OF MAIN LOOP ************************



// clock() function executes at the end of every clock period

  void clock()
  {
    // clock low to high transition clocks U4A and U5B
    if(idle_alert) timeIdle = millis(); // V1.2 reset timeIdle count to 'zero'
    if(U5A) // clears of U4A and U4B not held low
    {
      U4A = !U4A;
      // Here, U4A did change state so possible clock of U4A
      if(!U4A) // if -Q went high ...
      {
        U4B = !U4B; // already know clear was not held low
      }
      // It appears that if the ACS switch is OFF (closed), U5B can never be
      // clocked since SET or RESET will always be low. So it will alternate
      // with U6A (-BIT_READY)

      U6C = !U6A; // inverts U6A and goes to U5B -SET terminal Vx.xx
      if(U4A) // if Q went high, clock U5B
      {

        if(ACS && !U6A) U5B = false;

        if(U5B) start_clock(); // -Q of U5B starts clock when low
        else clock_active = false;
      }
    }
    if(U5B) // clear of U5A not held low
    {
      U5A = U7A; // if this makes U5B low, clear U4A & U4B
      if(!U5A)
      {
        U4A = false;
        U4B = false;
      }
      if(!U5A) // if U5A -Q went high, PMEM_CLK clocks U3A & B  Vx.xx chg to !U5A from U5A
      {
        if(U1C) // U3A clear not held low
        {
          U3A = U1D; // D goes to Q
        }
        if(U2A) // U3B clear not held low
        {
          U3B = U2B; // D goes to Q
        }
      }
    }

  }

// ************ START CLOCK **************************************************
// The clock runs when -Q of U5B is low. This can happen on clocking or when
// the clear terminal goes low. If the clock is already running, make no
// change. If it was not running, flag that it is running and reset the timer

  void start_clock()
  {
    if(!clock_active)
    {
      clock_active = true;
      timing_start = millis();

    }
    // Otherwise, clock was already running so no change
  }

// defaultSpeed calculates the hard coded speed using constant WPM

  void defaultSpeed()
  {
  ditTimeF = 20.0/WPM * 60.0; // NOTE 'WPM' is the default speed
  ditTime = (uint8_t)ditTimeF;
  }

// V1.2 - add routine to check battery voltage and report if it is low

  void readBatt()
  {

    batteryvolts = (float) analogRead(BATT_ADC) * ADC_REF/1023.0;

  }


  bool batteryCheck() // V1.2 check battery voltage, alarm if low
  {
    bool didalarm = false;
  readBatt(); 
  //Serial.print("Battery voltage = ");
  //Serial.println(batteryvolts);
  if(batteryvolts < BATT_LO_ALM) 
  {
   alertSound(); 
   send_code('B');
   didalarm = true;
  }
  return didalarm;
  }


  void do_report()
  {
    char voltsascii[4]; // example string "3.7" plus null
    uint8_t volt_ptr = 0;

    if(battmon == true)
    {
    readBatt();
    dtostrf(batteryvolts, 3, 1, voltsascii);
    while(voltsascii[volt_ptr] != 0)
  {
    send_code(voltsascii[volt_ptr]);
    volt_ptr++;

  }
      send_code(' '); // V1.4 - I want some space between # and V
      send_code('V');
    delay(700);
    }
    SpeedToString(1200/ditTime);
    morseSendString(SpeedString);
    delay(500); // V1.4 
    send_code('='); // V1.4
    morseSendString(VersionString); // V1.4
 // Serial.print("ditTime: "); // These lines for troubleshooting
 // Serial.println(ditTime);
//  Serial.print("SpeedString: ");
 // Serial.println(SpeedString);


  }

// This routine reads the pot and calculates the length of a ditTime, if the 
// reading has changed by 10 or more. This gives about 1% steps

	void readSpeedPot()
	{
    //uint8_t speed;
    //float speed_x;
    uint16_t difference;
    newADC = analogRead(POT_ADC);
    if(newADC >= oldADC) difference = newADC - oldADC;
    else difference = oldADC - newADC;

    if (difference >= 10)
    {
     // Serial.print("difference = ");
    //  Serial.println(difference);
      oldADC = newADC;
		// speed = SPD_LIMIT_LO; 
		speed_x = SPD_LIMIT_LO + (SPD_LIMIT_HI - SPD_LIMIT_LO) * (float) newADC/1023.0;
		speed = (uint8_t)  (speed_x + 0.5); // casting just truncates
    if(speed > SPD_LIMIT_HI) speed = SPD_LIMIT_HI;
    ditTime = 1200/speed;
    //Serial.print("speed: ");
   // Serial.println(speed);
    }
	}

// routine to do paddle hand key mode
// Either paddle gives key down. Both paddles exits this mode

  void do_manual()
  {
    bool dot_closed, dash_closed;
    while(manual)
    {
      dot_closed = !digitalRead(Dot_In);
      dash_closed = !digitalRead(Dash_In);

      if((dot_closed && !dash_closed) || (dash_closed && !dot_closed)) // either paddle closed
      {
		  digitalWrite(KeyOut, keyDNstate); // close keyed line
		  digitalWrite(KeyOut_i, keyUPstate); // V1.4
      digitalWrite(LEDPIN, HIGH); // V1.4 - add LED for key down in manual mode
      keyDown = true; // V1.4
		  tone(spkrpin, pitch);
      }
      else if (!dot_closed && !dash_closed)
      {
      // now neither paddle is closed
      
		  noTone(spkrpin);
		  digitalWrite(KeyOut, keyUPstate); // open keyed line
		  digitalWrite(KeyOut_i, keyDNstate); // V1.4
      digitalWrite(LEDPIN, LOW); // V1.4
      keyDown = false; // V1.4
      }
      // Now check for both paddles closed:
      else if(dot_closed && dash_closed) 
      {
      manual = false; //exit manual mode
      noTone(spkrpin); // and make sure we don't leave in key down state
		  digitalWrite(KeyOut, keyUPstate); 
		  digitalWrite(KeyOut_i, keyDNstate); // V1.4
      digitalWrite(LEDPIN, LOW); // V1.4 LED of when exiting manual; loop counter will turn back
      keyDown = false; // V1.4
      }
    }
  }
  

/* V1.1a - comment out printing

  void printStuff()
  {

    PrintTimes++;

    Serial.println();
    Serial.println("***");
    Serial.println();

    Serial.print("Out U7B: ");
    Serial.println(U7B);
    Serial.print("PRS Dot -U3A: ");
    Serial.println(!U3A);
    Serial.print("PRS Dash -U3B: ");
    Serial.println(!U3B);
    Serial.print("CNTR -U5A: ");
    Serial.println(!U5A);
    Serial.print("CNTR -U4A: ");
    Serial.println(!U4A);
    Serial.print("CNTR -U4B: ");
    Serial.println(!U4B);
    Serial.print("MISS BIT -U5B: ");
    Serial.println(!U5B);   
    Serial.print("DOT PDL: ");
    Serial.println(digitalRead(Dot_In));  
    Serial.print("DASH PDL: ");
    Serial.println(digitalRead(Dash_In)); 
    Serial.print("Clock: ");
    Serial.println(clock_active);
    Serial.print("IAM GATE U1D: ");
    Serial.println(U1D);
    Serial.print("IAM GATE U2B: ");
    Serial.println(U2B); 
    Serial.print("NEXT DASH U1A: ");
    Serial.println(U1A);
    Serial.print("NEXT DOT U2C: ");
    Serial.println(U2C);     
    Serial.print("clockCount = ");
    Serial.println(clockCount);
   // Serial.print("Dot Paddle: ");
   // Serial.println(digitalRead(Dot_In));
   // Serial.print("Dash Paddle: ");
   // Serial.println(digitalRead(Dash_In));      
  }

  void printAll()
  {
    Serial.println("\nU1A B C D U2C D A B U3A B U4A B U5A B U6A B C D U7A B");
    Serial.print("  ");
    Serial.print(U1A);
    Serial.print(" ");
    Serial.print(U1B);
    Serial.print(" ");
    Serial.print(U1C);
    Serial.print(" ");
    Serial.print(U1D);
    Serial.print("   ");
    Serial.print(U2C);
    Serial.print(" ");
    Serial.print(U2D);
    Serial.print(" ");
    Serial.print(U2A);
    Serial.print(" ");
    Serial.print(U2B);
    //Serial.print("\nU3A B U4A B U5A B");
    Serial.print("   ");
    Serial.print(U3A);
    Serial.print(" ");
    Serial.print(U3B);
    Serial.print("   ");
    Serial.print(U4A);
    Serial.print(" ");
    Serial.print(U4B);
    Serial.print("   ");
    Serial.print(U5A);
    Serial.print(" ");
    Serial.print(U5B);
    Serial.print("   ");
    Serial.print(U6A);
    Serial.print(" ");
    Serial.print(U6B);
    Serial.print(" ");
    Serial.print(U6C);
    Serial.print(" ");
    Serial.print(U6D);
    Serial.print("   ");
    Serial.print(U7A);
    Serial.print(" ");
    Serial.print(U7B);
    Serial.print("   ");
    Serial.println(clockCount);
  }

  */

  // **************  MORSE SENDING UNIT *************************************

  // Imnporting from Keyer_WA5BDu.ino

// Morse characters are stored like this:  The high bit represents the first
// element of a character.  0 is a dot and 1 is a dash.  The final 1 flags
// the end of this character and is not sent.  Therefore 'B' being _... is 
// stored as 0b10001000
// Sending happens by checking the high bit and sending dot or dash as
// appropriate, then left shifting the data by one bit.  Before sending though,
// if the data = 0b10000000, then stop.  Sending of this character is finished.

// There are some gaps in ASCII codes that have characters I don't need and
// so waste some RAM, but not enough to do special remapping.
// From 44 through 90 are 47 bytes and I figure 41 are moderately necessary.

// Notice below how it's possible to break lines and even have comments
// interspersed

byte morse[47] = {0b11001110, 0b10000110, 0b01010110,// ','  '-'  '.' 44-46
					0b10010100, 0b11111100, 0b01111100,// /, 0, 1  47-49
					0b00111100, 0b00011100, 0b00001100, // 2, 3, 4 50-52
					0b00000100, 0b10000100, 0b11000100, // 5, 6, 7 53-55
					0b11100100, 0b11110100, 0b10110100, // 8, 9, : (KN) 56-58
					0b10101010, 0b01010100, 0b10001100, // ; (KR), < (AR), = (BT) 59-61
					0b00010110, 0b00110010, 0b01101010, // > (SK), ?, @ (AC) 62-64
					0b01100000, 0b10001000, 0b10101000, // ABC starts at 65 DEC
					0b10010000, 0b01000000, 0b00101000, // DEF
					0b11010000, 0b00001000, 0b00100000, // GHI
					0b01111000, 0b10110000, 0b01001000, // JKL
					0b11100000, 0b10100000, 0b11110000, // MNO
					0b01101000, 0b11011000, 0b01010000, // PQR
					0b00010000, 0b11000000, 0b00110000, // STU
					0b00011000, 0b01110000, 0b10011000, // VWX
					0b10111000, 0b11001000};             // YZ ends at 90 DEC


void dit()
	{
		if(DoTransmitMSG)
		{
			digitalWrite(KeyOut, keyDNstate); // close keyed line
			digitalWrite(KeyOut_i, keyUPstate); // V1.4
      keyDown = true; // v1.4
		}
		tone(spkrpin, pitch);
		space();
		 noTone(spkrpin);
		digitalWrite(KeyOut, keyUPstate); // open keyed line
		digitalWrite(KeyOut_i, keyDNstate); // V1.4
    keyDown = false; // v1.4
		space();
	}

void dah()
	{
		if(DoTransmitMSG)
		{
			digitalWrite(KeyOut, keyDNstate); // close keyed line
			digitalWrite(KeyOut_i, keyUPstate); // V1.4
      keyDown = true; // v1.4
		}		
		tone(spkrpin, pitch);
		space();
		space();
		space();
		noTone(spkrpin);
		digitalWrite(KeyOut, keyUPstate); // open keyed line
		digitalWrite(KeyOut_i, keyDNstate); // V1.4
    keyDown = false; // v1.4
		space();
	}

void space()
	{
	   delay(ditTime); // delays ditTime milliseconds
	}

void char_space()
	{
		delay(2*ditTime);
	}
void word_space()
{
	  delay(4*ditTime);
}

// Send a null-terminated string in Morse

void morseSendString(char mstring[])

    {
      for(int x=0; mstring[x]; x++)
        {
            send_code(mstring[x]);
        }
        
    }

// ******  SEND CHARACTER IN MORSE *********************************

void send_code(int pointtocode)
{
  uint8_t saveDitTime;
  saveDitTime = ditTime;
  ditTime = ditTimeMsg; // V1.2 - use message timing in send_code
	byte pattern;
	if (pointtocode == ' ')
	{
	   word_space();
	}
	else
	{
	pointtocode -= 44; // shift ASCII pointer to begin at 0 for comma
	pattern = morse[pointtocode];
	
	while (pattern != 128)
	{
		if (pattern & 0b10000000)
		{
			dah();
		}
		else
		{
		   dit();
		} 
		
		pattern <<= 1;
		
	}
	char_space();
	}
  ditTime = saveDitTime; // V1.2 restore normal time
}		
 // borrowed from WA5BDU keyer

	void SpeedToString(uint8_t oldspeed)
	{
		SpeedString[0] = oldspeed/10 + '0'; // tens digit
		SpeedString[1] = oldspeed%10 + '0'; // ones digit
		
	}

// V1.2 - produce an alert sound and LED flashes

void alertSound()
{
  digitalWrite(LEDPIN, LOW); 
  delay(100);
  tone(spkrpin, 523); // C
  digitalWrite(LEDPIN, HIGH);
  delay(100);
  noTone(spkrpin);
  digitalWrite(LEDPIN, LOW);
   delay(100);
  tone(spkrpin, 659); // E
  digitalWrite(LEDPIN, HIGH);
  delay(100);
  noTone(spkrpin);
  digitalWrite(LEDPIN, LOW);
   delay(100);
  tone(spkrpin, 784); // G
  digitalWrite(LEDPIN, HIGH);
  delay(100);
  noTone(spkrpin);
  digitalWrite(LEDPIN, LOW);
  delay(200);
  digitalWrite(LEDPIN, HIGH);    
   }

