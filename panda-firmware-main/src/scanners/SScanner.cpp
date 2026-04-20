#include "SScanner.hpp"

void SScanner::setup() {


    // Initializing multiplexer and irq pins
    for (int i = 0; i < 4; i++) {
        pinMode(sMux[i], OUTPUT);
        digitalWrite(sMux[i], LOW);
    }

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    pinMode(irqPin, INPUT_PULLUP);

    // pinMode(sADCPins.miso, INPUT);
    // pinMode(sADCPins.mosi, OUTPUT);

    // Initializing ADC
    adc.setSettings(SPISettingsDefault);
    adc.initialize();
    adc.setGain(GainSettings::GAIN_1);
    adc.setMuxInputs(MuxSettings::CH0, MuxSettings::AGND);
    adc.setVREF(3.3f);
    adc.setBiasCurrent(BiasCurrentSettings::I_0);

}

void SScanner::update() {

    switch(state) {
      case IDLE:
        // Select mux channel and ADC input channel, reset timer and proceed to WAIT_MUX state
        adc.setMuxInputs(adcCh, MuxSettings::AGND);

        for (int i = 0; i < 4; i++) {
          digitalWrite(sMux[i], (channel >> i) & 1);
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
        // if (timer >= T_CONV_US) {
        if (adc.getInterrupt() && !digitalRead(irqPin)) {
          // Read ADC output
          // Set samples[channel] to what the ADC outputs
          float res = adc.getOutput();
          sData[channel] = res * sConstant;
          // Serial.print("Solenoid channel :");
          // Serial.print(channel + 1);
          // Serial.print(" | Raw reading: ");
          // Serial.println(res, 8);

          // if (bankPtr == 0) {
          //   Serial.print("S Channel: ");
          //   Serial.print(channel + 1);
          //   Serial.print(" | Raw: ");
          //   Serial.println(res, HEX);
          // }
          // else if (bankPtr == 1) {
          //   Serial.print("PT Channel: ");
          //   Serial.print(channel + 1);
          //   Serial.print(" | Raw: ");
          //   Serial.println(res, HEX);
          // }          // else if (bankPtr == 2) {
          //   Serial.print("LCTC Channel: ");
          //   Serial.print(channel + 1);
          //   Serial.print(" | Raw: ");
          //   Serial.println(res, HEX);
          // }

          // Advancing channel
          channel++;
          if (channel >= NUM_DC_CHANNELS) {
            channel = 0;
          }
          timer = 0;
          state = IDLE;
        }
        break;
    }
  }

  void SScanner::getSOutput(float* out) {
    for (int i = 0; i < NUM_DC_CHANNELS; i++) {out[i] = sData[i];}
  }

