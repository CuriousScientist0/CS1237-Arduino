/*RP2040-CS1237 minimal code for operating the 24-bit ADC with an Arduino-compatible MCU
* Blog: https://www.curiousscientist.tech
* GitHub: https://github.com/CuriousScientist0
* Donate: https://curiousscientist.tech/support-me
*/

//-------------------------------------------------------------------------------------------------------------
//This is valid for RP2040, make sure you adjust it for your own MCU based on its clock speed.  
#define DELAY_455_NS asm volatile ("nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t")
//11 NOP is 11x7.52 ns = 82.72 ns
//1 / 133 MHz = 7.52 ns (1 cycle time of the MCU)

void customDelay455ns() 
{
  // Adjust the number of cycles based on the calculated value
  // This may need to be fine-tuned based on the actual execution time
  for (int i = 0; i < 10; ++i) // 455 ns/ 82.72 ns = 5.5. Since 455 is a minimum req, I increased to 10. There's no max value...
  {
    DELAY_455_NS;
  }
  //So actually, this delay is more like 827.2 ns. But it seems to work well. It is stable.
  //However, always refer to the clock cycle of your chosen MCU!!!
}
//--------------------------------------------------------------------------------------------------

//Pin descriptions. Can be any GPIO pin, but if you use an Arduino Uno or Nano, avoid pin 0 and 1 because they are serial pins!.
//Also avoid pin 2 and 3 on Nano/Uno because they are valuable interrupt pins.
int DOUT_DRDY = 1; //DOUT/DRDY pin
int SCLK = 0; //SCLK pin

long ADCreading; //This variable stores the ADCreading - Can be omitted with all its relevant references!
float pga_divider = 1.0; //default value = 1. This value stores the gain value

void setup()
{
  Serial.begin(115200);//Start serial
  while (!Serial) {} //Wait for serial
  delay(3000); //Delay a bit, so the terminal can catch up with the next message:
  Serial.println("RP2040 Zero - CS1237 ADC - Curious Scientist, Custom code");

  pinMode(DOUT_DRDY, INPUT); //DRDY - Input
  digitalWrite(DOUT_DRDY, LOW); //Pull it LOW

  pinMode(SCLK, OUTPUT); //SCLK - OUTPUT
  digitalWrite(SCLK, LOW); //Pull it LOW

  //Make sure the chip is awake
  while (digitalRead(DOUT_DRDY) == 0) {} //Wait while DRDY is low
  while (digitalRead(DOUT_DRDY) == 1) {} //Wait while DRDY is high
  delay(1000);

  //Set all registers to a default value (my arbitrarily chosen default values)
  setRegister(0, 0); //CH 0 input
  delay(100);
  setRegister(1, 0); //PGA = 1
  delay(100);
  setRegister(2, 0); //DRATE = 10 Hz
  delay(100);
  setRegister(3, 1); //VREF = DISABLED
}


long readADC() //Data acquisition function - Returns a long variable
{
    while (digitalRead(DOUT_DRDY) == 1) //Wait for the DRDY/DOUT to fall LOW
    {
        //Wait until dout/drdy becomes 0
    }
    //DRDY was low, so we can proceed further

    long result = 0; //24-bit output data is stored in this variable

    delayMicroseconds(1); //t4 (could be zero, actually)

    for (int i = 0; i < 24; i++) //Read the 24-bits
    {        
        digitalWrite(SCLK, HIGH);
        customDelay455ns(); //t6

        result |= digitalRead(DOUT_DRDY) << (23 - i); //Read a bit and shift it up
        //i = 0; MSB @ bit 23
        //i = 1; MSB-1 @ bit 22
        //... i = 23; LSB @ bit 0 (not shifted, just OR'd together with the result)

        digitalWrite(SCLK, LOW);
        customDelay455ns(); //t6
    }

    //Shift bit 25-26-27 as well.
    for (uint8_t i = 0; i < 3; i++)
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns(); //t6
        digitalWrite(SCLK, LOW);
        customDelay455ns(); //t6
    }

    return result;
}

int getRegister() //Reads all registers. The actual values will be fetched with other, simple functions
{  
    byte registerValue; //Variable that stores the config register value

    //Shift out 27 (24+3) bits
    ADCreading = readADC(); //32-bit variable that stores the whole ADC reading
    
    pinMode(DOUT_DRDY, OUTPUT); //After the 27th SCLK pulse, set DOUT to OUTPUT

    for (uint8_t i = 0; i < 2; i++) //Emit 2 pulses (28-29)
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns(); //t6
        digitalWrite(SCLK, LOW);
        customDelay455ns(); //t6
    } 

    for (uint8_t i = 0; i < 7; i++) //SCLK 30-36, sending READ word
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns();
        digitalWrite(DOUT_DRDY, ((0x56 >> (6 - i)) & 0b00000001)); //0x56 - READ
        digitalWrite(SCLK, LOW);
        customDelay455ns();
    }    

    for (uint8_t i = 0; i < 1; i++) //Send the 37th SCLK pulse
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns(); //t6
        digitalWrite(SCLK, LOW);
        customDelay455ns(); //t6
    } 
    
    //After the 37th SCLK pulse switch the direction of DOUT. 
    pinMode(DOUT_DRDY, INPUT_PULLUP); //we read, so dout becomes INPUT

    registerValue = 0; //Because we are reading

    for (uint8_t i = 0; i < 8; i++) //38-45 SCLK pulses
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns();        
        registerValue |= digitalRead(DOUT_DRDY) << (7 - i); //read out and shift the values into the register_value variable        
        digitalWrite(SCLK, LOW);
        customDelay455ns();
    }

    // send 1 clock pulse, to set the Pins of the ADCs to output and pull high
    for (uint8_t i = 0; i < 1; i++)
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns(); //t6
        digitalWrite(SCLK, LOW);
        customDelay455ns(); //t6
    }

    // At the 46th SCLK, switch DRDY / DOUT to output and pull up DRDY / DOUT. 
    pinMode(DOUT_DRDY, INPUT_PULLUP); //Ready to receive the DRDY to perform a new acquisition

    return registerValue;
}

void setRegister(int registertowrite, int valuetowrite)
{
    //"Arbitrary" register numbers
    //0 - Channel
    //1 - PGA
    //2 - Speed
    //3 - REF

    //Config register structure
    //bit 0-1 : Channel. 00 - A, 01 - reserved, 10 - Temperature, 11 - internal short (maybe for offset calibration?)
    //bit 2-3 : PGA. 00 - 1, 01 - 2, 10 - 64, 11 - 128
    //bit 4-5 : speed. 00 - 10 Hz, 01 - 40 Hz, 10 - 640 Hz, 11 - 1280 Hz
    //bit 6   : Reference. Default is enabled which is 0.
    //bit 7   : reserved, don't touch
    //----------------------------------------------------------
    
    byte register_value = getRegister(); //Variable that stores the config register value. Fill it up with the current reg value

    int byteMask = 0b00000000; //Masking byte for writing only 1 register at a time

    switch (registertowrite)
    {
        case 0: //channel
        byteMask = 0b11111100; //when using & operator, we keep all, except channel bits
        register_value = register_value & byteMask; //Update the register_value with mask. This deletes the first two
        
            switch (valuetowrite)
            {
            case 0: // A // W0 0
                register_value = register_value | 0b00000000; //Basically keep everything as-is
                Serial.println("Channel = 0");
                break;
            case 1: // Reserved // W0 1
                //dont implement it!
                Serial.println("Channel = Reserved, invalid!");
                break;
            case 2: //Temperature // W0 2
                register_value = register_value | 0b00000010;
                Serial.println("Channel = Temp");
                break;
            case 3: //Internal short //W0 3
                register_value = register_value | 0b00000011;
                Serial.println("Channel = Short");
                break;
            }      
        break;
        //-------------------------------------------------------------------------------------------------------------

        case 1: //PGA
            byteMask = 0b11110011; //when using & operator, we keep all, except channel bits
            register_value = register_value & byteMask; //Update the register_value with mask. This deletes the first two

            switch (valuetowrite)
            {
            case 0: // PGA 1 //W1 0
                register_value = register_value | 0b00000000; //Basically keep everything as-is
                pga_divider = 1;
                Serial.println("PGA = 1");
                break;
            case 1: // PGA 2 //W1 1
                register_value = register_value | 0b00000100;
                pga_divider = 2;
                Serial.println("PGA = 2");
                break;
            case 2: //PGA 64 //W1 2
                register_value = register_value | 0b00001000;
                pga_divider = 64;
                Serial.println("PGA = 64");
                break;
            case 3: //PGA 128 //W1 3
                register_value = register_value | 0b00001100;
                pga_divider = 128;
                Serial.println("PGA = 128");
                break;
            }
            break;
            //-------------------------------------------------------------------------------------------------------------

        case 2: //DRATE
            byteMask = 0b11001111; //when using & operator, we keep all, except channel bits
            register_value = register_value & byteMask; //Update the register_value with mask. This deletes the first two

            switch (valuetowrite)
            {
            case 0: // 10 Hz //W2 0
                register_value = register_value | 0b00000000; //Basically keep everything as-is
                Serial.println("DRATE = 10 Hz");
                break;
            case 1: // 40 Hz //W2 1
                register_value = register_value | 0b00010000;
                Serial.println("DRATE = 40 Hz");
                break;
            case 2: //640 Hz //W2 2
                register_value = register_value | 0b00100000;
                Serial.println("DRATE = 640 Hz");
                break;
            case 3: //1280 Hz //W2 3
                register_value = register_value | 0b00110000;
                Serial.println("DRATE = 1280 Hz");
                break;
            }
            break;
            //-------------------------------------------------------------------------------------------------------------
        case 3: //VREF
            if (valuetowrite == 0) //W3 0
            {
                bitWrite(register_value, 6, 0); //Enable
                Serial.println("VREF ON");
            }
            else if (valuetowrite == 1) //W3 1
            {
                bitWrite(register_value, 6, 1); //Disable
                Serial.println("VREF OFF");
            }
            else {}//Other values wont trigger anything
            break;
    }

    //Shift out 27 (24+3) bits
    ADCreading = readADC(); //32-bit variable that stores the whole ADC reading

    pinMode(DOUT_DRDY, OUTPUT); //After the 27th SCLK pulse, set DOUT to OUTPUT

    for (uint8_t i = 0; i < 2; i++) //Emit 2 pulses (28-29)
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns(); //t6
        digitalWrite(SCLK, LOW);
        customDelay455ns(); //t6
    } 

    for (uint8_t i = 0; i < 7; i++) //SCLK 30-36, sending READ word
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns();
        digitalWrite(DOUT_DRDY, ((0x65 >> (6 - i)) & 0b00000001)); //0x65 - WRITE
        digitalWrite(SCLK, LOW);
        customDelay455ns();
    }

    for (uint8_t i = 0; i < 1; i++) //Send the 37th SCLK pulse
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns(); //t6
        digitalWrite(SCLK, LOW);
        customDelay455ns(); //t6
    } 

    for (uint8_t i = 0; i < 8; i++) //38-45 SCLK pulses
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns();
        digitalWrite(DOUT_DRDY, ((register_value >> (7 - i)) & 0b00000001)); //Write the register values        
        digitalWrite(SCLK, LOW);
        customDelay455ns();
    }

    // send 1 clock pulse, to set the Pins of the ADCs to output and pull high
    for (uint8_t i = 0; i < 1; i++)
    {
        digitalWrite(SCLK, HIGH);
        customDelay455ns(); //t6
        digitalWrite(SCLK, LOW);
        customDelay455ns(); //t6
    }

    // At the 46th SCLK, switch DRDY / DOUT to output and pull up DRDY / DOUT. 
    pinMode(DOUT_DRDY, INPUT_PULLUP);
}

void loop()
{
    if (Serial.available() > 0)
    {
        char commandCharacter = Serial.read(); //we use characters (letters) for controlling the switch-case
        switch (commandCharacter) //based on the command character, we decide what to do
        {
            case 'R': //Read register
            {
                byte regvalues = getRegister(); //Read the register value

                Serial.print("Register: 0b"); //Print the register value in a convenient 0bxxxxxxxx format

                for (int i = 7; i >= 0; i--) 
                {
                    Serial.print((regvalues >> i) & 1); //Print/shift the register bits until the whole byte is printed
                }
                Serial.println(); // Print a newline character
            }
            break;

            case 'A': //Read continuously
            {
                while (Serial.read() != 's')
                {
                    long adcValue = readADC();

                    //Treat negative readings
                    if (adcValue >> 23 == 1) //if the 24th bit (sign) is 1, the number is negative
                    {
                        adcValue = adcValue - 16777216;  //conversion for the negative sign
                        //"mirroring" around zero
                    }

                    float voltageValue = (1250.0 / pga_divider) * ((float)adcValue / (8388607.0));
                    //Formula from datasheet, see Section 2.6.3: (0.5 * VREF / GAIN) * LSB/(2^(23) - 1)
                    //NOTE: if you want the values in milliVolts, change 2.5 to 2500.
                    //I use milliVolts and I directly entered the 0.5*VREF, so 0.5x2500 mV = 1250 mV

                    Serial.print("RAW-ADC reading: ");
                    Serial.println(adcValue);

                    Serial.print("Voltage reading: ");
                    Serial.println(voltageValue, 10); //Print the voltage in voltage units and with 10 digits                   
                }
            }
            break;

            case 'W': //Write register
            {
                while (!Serial.available()); //wait for the serial
                int registerToWrite = Serial.parseInt(); //Parse the register ID
                delay(100);

                while (!Serial.available());
                int valueToWrite = Serial.parseInt(); //Parse the register value
                delay(100);

                setRegister(registerToWrite, valueToWrite); //Set the register value
            }
            break;
        }
    }
  
}
