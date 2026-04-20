#include "FScanner.hpp"
// #if 0
void FScanner::setup() {
    
    // Initializing pins
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 4; j++) {
        pinMode(settingsArr[i].muxPins[j], OUTPUT);
        digitalWrite(settingsArr[i].muxPins[j], LOW);
      }
    }

    // Testing
    // digitalWrite(settingsArr[0].muxPins[3], HIGH);
  
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    pinMode(irqPin, INPUT_PULLUP);
  
    // pinMode(ptADCPins.miso, INPUT);
    // pinMode(ptADCPins.mosi, OUTPUT);
    
    // Initializing ADC
    adc.setSettings(SPISettingsDefault);
    adc.initialize();
    // Serial.println("0");
    // delay(100);
    // adc.writeRegisterDefaults(); // Called twice to ensure operation after power-cycling
    // Serial.println("1");
    // delay(100);
    // adc.writeRegisterDefaults();
    // Serial.println("2");
    adc.setGain(GainSettings::GAIN_1);
    adc.setMuxInputs(MuxSettings::CH0, MuxSettings::AGND);
    adc.setVREF(1.25f);
    adc.setBiasCurrent(BiasCurrentSettings::I_0);
    // adc.readAllRegisters();
    // adc.printRegisters();

    tSensor.initialize();



}

void FScanner::update() {

  boardTemp = tSensor.getTemp();

  switch(state) {
    case IDLE:
      // Select mux channel and ADC input channel, reset timer and proceed to WAIT_MUX state
      adc.setMuxInputs(settingsArr[index].muxChannel, MuxSettings::AGND);

      for (int i = 0; i < 4; i++) {
        digitalWrite(settingsArr[index].muxPins[i], (channel >> i) & 1);
      }

      timer = 0;
      state = WAIT_MUX;
      break;
    case WAIT_MUX:
      // wait for T_MUX_SETTLE_US, then start one-shot reading from ADC
      // reset timer, then proceed to WAIT_CONV
      if (timer >= T_MUX_SETTLE_US) {
        // Tell the ADC to perform one-shot
        adc.trigger();
        timer = 0;
        state = WAIT_CONV;
      }
      break;
    case WAIT_CONV:
      // wait for T_CONV_US, then read the ADC output into the respective samples index
      // Increment channel (If it's at 15, wrap back to zero, then we have a full sample array and we're good to process)
      // return to IDLE
      if (adc.getInterrupt() && !digitalRead(irqPin)) {
      // if (true) {
        // Read ADC output
        // Set samples[channel] to what the ADC outputs
        // float res = adc.getOutput();
        float res = adc.getOutput();
        // if (index == 0) {
        //   Serial.print("LCTC Channel: ");
          
        // }
        // else {Serial.print("PT Channel: ");}

        // Serial.print(channel + 1);
        // Serial.print(" | ");
        // Serial.print("Raw reading: ");
        // Serial.println(res);

        if (index == 0 && channel >= 6) { // Load cell/TC conversions
          uint8_t tcChannel = channel - NUM_TC_CHANNELS;
          Serial.print("TC Channel: ");
          Serial.print(tcChannel);
          float rawVoltage = res - tcOffsets[tcChannel];

          Serial.print(" | Raw voltage: ");
          Serial.print(res,5);

          Serial.print(" | Adjusted voltage: ");
          Serial.print(rawVoltage,5);

          float reading = rawVoltage * tcConstant + boardTemp;

          Serial.print(" | Temperature: ");
          Serial.println(reading);


          settingsArr[index].out[channel] = reading;

        }
        


        // Advancing channel and/or bank
        channel++;
        if (channel >= settingsArr[index].numChannels) {
          channel = 0;
          // index++;
          // if (index >= 2) {
          //   index = 0;
          // }
        }
        timer = 0;
        state = IDLE;
      }
      break;
    }
  }

  void FScanner::getLCTCOutput(float* out) {
    memcpy(out, lctcOutput, sizeof(float) * (NUM_LC_CHANNELS + NUM_TC_CHANNELS));
  }
  void FScanner::getPTOutput(float* out) {
    memcpy(out, ptOutput, sizeof(float) * NUM_PT_CHANNELS);
  }
// #endif