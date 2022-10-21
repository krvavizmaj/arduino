/**
 * Arduino infrared switch with manual decoding of the remote signal.
 * Works with samsung tv remove. The IR code is decoded and compared to a predefined
 * 'SWITCH_CODE'. When 'EXPECTED_REPEATS' of the code happen repeatedly, the action is executed.
*/

// #define DEBUG
// #define TRACE

#define IRpin_PIN                       PIND
#define IRpin                           2
#define LEDpin                          4

#define SWITCH_CODE                     0x97680707
#define EXPECTED_REPEATS                10

// Timing variables
#define MICROS_PER_TICK                 50
#define GAP_MICROS                      5000
#define GAP_TICKS                       (GAP_MICROS / MICROS_PER_TICK)

// Samsung protocol variables
#define SAMSUNG_UNIT                    560

#define SAMSUNG_HEADER_MARK             (8 * SAMSUNG_UNIT)
#define SAMSUNG_HEADER_MARK_TICKS_LOW   ((SAMSUNG_HEADER_MARK / MICROS_PER_TICK) * 0.75)
#define SAMSUNG_HEADER_MARK_TICKS_HIGH  ((SAMSUNG_HEADER_MARK / MICROS_PER_TICK) * 1.25)
#define SAMSUNG_HEADER_SPACE            (8 * SAMSUNG_UNIT)
#define SAMSUNG_HEADER_SPACE_TICKS_LOW  ((SAMSUNG_HEADER_SPACE / MICROS_PER_TICK) * 0.75)
#define SAMSUNG_HEADER_SPACE_TICKS_HIGH ((SAMSUNG_HEADER_SPACE / MICROS_PER_TICK) * 1.25)

#define SAMSUNG_BIT_MARK                SAMSUNG_UNIT
#define SAMSUNG_BIT_MARK_TICKS          (SAMSUNG_BIT_MARK / MICROS_PER_TICK)
#define SAMSUNG_BIT_MARK_TICKS_LOW      (SAMSUNG_BIT_MARK_TICKS * 0.75)
#define SAMSUNG_BIT_MARK_TICKS_HIGH     (SAMSUNG_BIT_MARK_TICKS * 1.25)

#define SAMSUNG_ONE_SPACE               (3 * SAMSUNG_UNIT)
#define SAMSUNG_ONE_SPACE_TICKS         (SAMSUNG_ONE_SPACE / MICROS_PER_TICK)
#define SAMSUNG_ONE_SPACE_TICKS_LOW     (SAMSUNG_ONE_SPACE_TICKS * 0.75)
#define SAMSUNG_ONE_SPACE_TICKS_HIGH    (SAMSUNG_ONE_SPACE_TICKS * 1.25)

#define SAMSUNG_ZERO_SPACE              SAMSUNG_UNIT
#define SAMSUNG_ZERO_SPACE_TICKS        (SAMSUNG_ZERO_SPACE / MICROS_PER_TICK)
#define SAMSUNG_ZERO_SPACE_TICKS_LOW    (SAMSUNG_ZERO_SPACE_TICKS * 0.75)
#define SAMSUNG_ZERO_SPACE_TICKS_HIGH   (SAMSUNG_ZERO_SPACE_TICKS * 1.25)

#define SAMSUNG_MARK_LEVEL              0

#define IR_STATE_IDLE                   0
#define IR_STATE_MARK                   1
#define IR_STATE_SPACE                  2
#define IR_STATE_STOP                   4

uint8_t rawLength = 0;                  // index for pulses we're storing

uint8_t currentState = IR_STATE_IDLE;
uint8_t tickCount = 0;
uint8_t currentIRLevel = 0;

uint32_t decodedResult = 0;
uint8_t switchCodeRepetitions = 0;
uint16_t switchCodeGapCount = 0;

uint32_t mask = 1UL;

bool receivingHeader = false;
bool headerMarkValid = true;
bool headerSpaceValid = true;
bool codeValid = true;

void setup(void) {
  Serial.begin(115200);
#if defined(DEBUG)
  Serial.println("Ready to decode IR!");
#endif
}
 
void loop(void) {

  currentIRLevel = IRpin_PIN & (1 << IRpin);
  if (tickCount < UINT16_MAX) {
      tickCount++;
  }
  if (switchCodeGapCount < UINT16_MAX) {
      switchCodeGapCount++;
  }
    
  if (currentState == IR_STATE_IDLE) {
    if (currentIRLevel == SAMSUNG_MARK_LEVEL) {
      if (tickCount > GAP_TICKS) {
        // start of SIGNAL
        currentState = IR_STATE_MARK;
        receivingHeader = true;
        codeValid = true;
        rawLength = 0;
        decodedResult = 0;
        mask = 1UL;
      }
      tickCount = 0;
    }
  } else if (currentState == IR_STATE_MARK) {
    if (currentIRLevel != SAMSUNG_MARK_LEVEL) {
      if (receivingHeader) {
        // end of header mark, validate it
        headerMarkValid = tickCount >= SAMSUNG_HEADER_MARK_TICKS_LOW && 
                          tickCount <= SAMSUNG_HEADER_MARK_TICKS_HIGH;
      } else {
        // end of mark, validate mark
        if (tickCount < SAMSUNG_BIT_MARK_TICKS_LOW || tickCount > SAMSUNG_BIT_MARK_TICKS_HIGH) {
#if defined(TRACE)
          Serial.print("mark invalid ");
          Serial.print(rawLength);
          Serial.print(" ");
          Serial.println(tickCount);
#endif
          codeValid = false;
        }
      }

      currentState = IR_STATE_SPACE;
      rawLength++;
      tickCount = 0;
    }
  } else if (currentState == IR_STATE_SPACE) {
    if (currentIRLevel == SAMSUNG_MARK_LEVEL) {
      if (rawLength >= 100) {
        // overflow
        currentState = IR_STATE_STOP;
      } else {
        if (receivingHeader) {
          // end of header space, validate it
          headerSpaceValid = tickCount >= SAMSUNG_HEADER_SPACE_TICKS_LOW && 
                             tickCount <= SAMSUNG_HEADER_SPACE_TICKS_HIGH;
          receivingHeader = false;
        } else {
          // end of bit space, validate space
          if (tickCount >= SAMSUNG_ZERO_SPACE_TICKS_LOW && tickCount <= SAMSUNG_ZERO_SPACE_TICKS_HIGH) {
            // zero bit, do nothing
          } else  if (tickCount >= SAMSUNG_ONE_SPACE_TICKS_LOW && tickCount <= SAMSUNG_ONE_SPACE_TICKS_HIGH) {
            // one bit, set it to result
            decodedResult |= mask;
          } else {
#if defined(TRACE)
            Serial.print("space invalid ");
            Serial.print(rawLength);
            Serial.print(" ");
            Serial.println(tickCount);
#endif
            codeValid = false;
          }
          mask <<= 1;
        }
        currentState = IR_STATE_MARK;
        rawLength++;
        tickCount = 0;
      }
    } else if (tickCount > GAP_TICKS) {
      currentState = IR_STATE_STOP;
    }
  } else if (currentState == IR_STATE_STOP) {
    // validate and decode signal
#if defined(TRACE)
      Serial.print("raw length: ");
      Serial.println(rawLength);
      Serial.print("header valid: ");
      Serial.println(headerMarkValid && headerSpaceValid);
      Serial.print("code valid: ");
      Serial.println(codeValid);
#endif
    // signal length: header 2 bits + data: 2*32 bits + one stop bit
    if (rawLength == 67 && headerMarkValid && headerSpaceValid && codeValid) {
#if defined(DEBUG)
      Serial.print("Decoded signal: ");
      Serial.println(decodedResult, HEX);
      Serial.println();
#endif
      if (decodedResult == SWITCH_CODE) {
        if (switchCodeGapCount * MICROS_PER_TICK > 35000) {
          switchCodeRepetitions = 0;
        } 
        switchCodeGapCount = 0;
      
        switchCodeRepetitions++;
        if (switchCodeRepetitions == EXPECTED_REPEATS) {
          switchCodeRepetitions = 0;
          switchCodeGapCount = 0;
          doAction();
        }
      }
    }

    // reset variables and continue
    currentState = IR_STATE_IDLE;
    rawLength = 0;
    tickCount = 0;
  }

  delayMicroseconds(MICROS_PER_TICK);
}

void doAction() {
  PORTD = PORTD ^ (1 << LEDpin);
}
