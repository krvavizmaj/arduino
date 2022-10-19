/**
 * Arduino infrared switch with manual decoding of the remote signal.
 * Works with samsung tv remove. The IR code is decoded and compared to a predefined
 * 'SWITCH_CODE'. When 'EXPECTED_REPEATS' of the code happen repeatedly, the action is executed.
*/

// #define DEBUG
// #define TRACE

#define IRpin_PIN               PIND
#define IRpin                   2
#define LEDpin                  4

#define SWITCH_CODE             0x97680707
#define EXPECTED_REPEATS        10

// Timing variables
#define MICROS_PER_TICK         50
#define GAP_MICROS              5000
#define GAP_TICKS               (GAP_MICROS / MICROS_PER_TICK)

// Samsung protocol variables
#define SAMSUNG_UNIT            560
#define SAMSUNG_HEADER_MARK     (8 * SAMSUNG_UNIT)
#define SAMSUNG_HEADER_SPACE    (8 * SAMSUNG_UNIT)
#define SAMSUNG_BIT_MARK        SAMSUNG_UNIT
#define SAMSUNG_ONE_SPACE       (3 * SAMSUNG_UNIT)
#define SAMSUNG_ZERO_SPACE      SAMSUNG_UNIT
#define SAMSUNG_MARK_LEVEL      0
#define SAMSUNG_HEADER_BITS     2
#define SAMSUNG_DATA_BITS       32
#define SAMSUNG_STOP_BITS       1
#define BIT_DURATION_THRESHOLD  20    // Threshold for each bit duration in microseconds

#define IR_STATE_IDLE           0
#define IR_STATE_MARK           1
#define IR_STATE_SPACE          2
#define IR_STATE_STOP           4

#define RAW_BUFFER_LENGTH 100
 
uint16_t rawData[RAW_BUFFER_LENGTH];    // tick count at a transition
uint8_t rawLength = 0;                  // index for pulses we're storing
uint8_t currentPulseIndex = 0;          // index for pulses we're storing

uint8_t currentState = IR_STATE_IDLE;
uint8_t tickCount = 0;
uint8_t currentIRLevel = 0;

uint32_t decodedResult = 0;
uint8_t switchCodeRepetitions = 0;
uint16_t switchCodeGapCount = 0;

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
        rawLength = 0;
      }
      tickCount = 0;
    }
  } else if (currentState == IR_STATE_MARK) {
    if (currentIRLevel != SAMSUNG_MARK_LEVEL) {
      // start of SPACE
      currentState = IR_STATE_SPACE;
      rawData[rawLength++] = tickCount;
      tickCount = 0;
    }
  } else if (currentState == IR_STATE_SPACE) {
    if (currentIRLevel == SAMSUNG_MARK_LEVEL) {
      if (rawLength >= RAW_BUFFER_LENGTH) {
        // overflow
        currentState = IR_STATE_STOP;
      } else {
        // start of MARK
        currentState = IR_STATE_MARK;
        rawData[rawLength++] = tickCount;
        tickCount = 0;
      }
    } else if (tickCount > GAP_TICKS) {
      currentState = IR_STATE_STOP;
    }
  } else if (currentState == IR_STATE_STOP) {
    // validate and decode signal
    if (rawLength == (SAMSUNG_HEADER_BITS + (2 * SAMSUNG_DATA_BITS) + SAMSUNG_STOP_BITS) && validateHeader()) {

      if (decode()) {
#if defined(DEBUG)
        Serial.println(decodedResult, HEX);
#endif
        if (decodedResult == SWITCH_CODE) {
#if defined(TRACE)
          for (int i = 0; i < rawLength; i++) {
            Serial.print(rawData[i] * 50);
            Serial.print(" ");
            Serial.println(rawData[++i] * 50);
          }
          Serial.println();
#endif
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
        } else {
#if defined(DEBUG)
          Serial.println("decode error");
#endif
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

/**
 * Decode bits LSB
*/
bool decode() {
  uint32_t mask = 1UL;
  decodedResult = 0;

  int index = 2;
  for (uint_fast8_t i = 0; i < SAMSUNG_DATA_BITS; i++) {
    if (!matchMark(index)) {
      return false;
    }

    index++;
    if (matchSpace(index, SAMSUNG_ONE_SPACE)) {
      // '1' bit
      decodedResult |= mask;
    } else if (matchSpace(index, SAMSUNG_ZERO_SPACE)) {
      // '0' bit, do nothing
    } else {
      // no match
      return false;
    }

    index++;
    mask <<= 1;
  }
  
  return true;
}

/**
 * Check if header of received signal matches
*/
bool validateHeader() {
  // check header mark
  // number of ticks should be more than 75% and less that 125% of the expected ticks
  if ((rawData[0] < (SAMSUNG_HEADER_MARK / MICROS_PER_TICK) * 0.75) ||
      (rawData[0] > (SAMSUNG_HEADER_MARK / MICROS_PER_TICK) * 1.25)) {
      return false;
  }

  // check header space
  // number of ticks should be more than 75% and less that 125% of the expected ticks
  if ((rawData[1] < (SAMSUNG_HEADER_SPACE / MICROS_PER_TICK) * 0.75) ||
      (rawData[1] > (SAMSUNG_HEADER_SPACE / MICROS_PER_TICK) * 1.25)) {
      return false;
  }

  return true;
}

bool matchMark(int atIndex) {
  // check signal mark
  // number of ticks should be more than 75% and less that 125% of the expected ticks
  if ((rawData[atIndex] < (SAMSUNG_BIT_MARK / MICROS_PER_TICK) * 0.75) ||
      (rawData[atIndex] > (SAMSUNG_BIT_MARK / MICROS_PER_TICK) * 1.25)) {
#if defined(TRACE)
      Serial.print("Mark expected at index ");
      Serial.print(atIndex);
      Serial.print(" but was ");
      Serial.println(rawData[atIndex]);
#endif
      return false;
  }
  return true;
}

bool matchSpace(int atIndex, int length) {
  // check signal space
  // number of ticks should be more than 75% and less that 125% of the expected ticks
  if ((rawData[atIndex] < (length / MICROS_PER_TICK) * 0.75) ||
      (rawData[atIndex] > (length / MICROS_PER_TICK) * 1.25)) {
#if defined(TRACE)
      Serial.print("Space expected at index ");
      Serial.print(atIndex);
      Serial.print(" but was ");
      Serial.println(rawData[atIndex]);
#endif
      return false;
  }
  return true;
}

void doAction() {
  PORTD = PORTD ^ (1 << LEDpin);
}