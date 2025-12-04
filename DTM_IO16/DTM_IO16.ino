// Original works by https://github.com/tolunaygul
// Modified by Peter at PT Motorsport AU to suit the line of PT Motorsport products
// This code is for the v1.3 IO-MINI Board & v1.1 IO-DTM Board
// Revised for new Muxed CAN Protocol

#include <mcp_can.h>
#include <SPI.h>
#include <EEPROM.h>

long unsigned int rxId; 			 // storage for can data
unsigned char len = 0; 			 // storage for can data // FIX: Added initialization and semicolon
unsigned char rxBuf[8]; 		 // storage for can data

#define CAN0_INT 2 				 // Set INT to pin 2
MCP_CAN CAN0(4); 				 // set CS pin to 4

#define LED_PIN A5 				 // Set CAN LED pin to analog pin 5

// =========================================================================
// 1. NEW CAN PROTOCOL DEFINITIONS (Based on Base ID 0x6A8)
// =========================================================================

// --- Incoming Messages (RX) ---
// HBO Control (0x6A8) Mux ID 2 (Half Bridge) - Duty Cycle/Frequency control (HBO1, HBO2 only)
const long unsigned int HBO_CONTROL_ID = 0x6A8; 	 
// HBO Config (0x6A9) Mux ID 2 (HBO Config), 3 (SPI Config), and 4 (AVI Config)
const long unsigned int HBO_CONFIG_ID = 0x6A9;    // 0x6A8 + 1 

// --- Outgoing Messages (TX) ---
const long unsigned int HBO_STATUS_ID = 0x6AC;    // 0x6A8 + 4 - Mux ID 2 (Half Bridge) - HBO Status
const long unsigned int SPI_STATUS_ID = 0x6AB;    // 0x6A8 + 3 - Mux ID 3 (SPI) - DPI/SPI status
const long unsigned int AVI_STATUS_ID_1 = 0x330; // AVI 1-4 State/Voltage
const long unsigned int SPI_STATUS_ID_1 = 0x332; // SPI 1-4 State/Voltage
const long unsigned int SPI_STATUS_ID_2 = 0x333; // SPI 1-4 Frequency

// The base CAN addresses are used here. They are adjusted in setup() based on A7 input.
long unsigned int AVIsendCANAddress = AVI_STATUS_ID_1; // Used for AVI Status 1 (0x330)
long unsigned int SPIsendCANAddress1 = SPI_STATUS_ID; // Used for SPI Status (0x6AB)

// Placeholder for old addresses; only kept if used for base adjustment
long unsigned int KeepAliveCANAddress = 0x2C6; // Keep original keep-alive ID

// Mux ID definitions
const int MUX_ID_HBO = 2;
const int MUX_ID_SPI = 3;
const int MUX_ID_AVI = 4;

// =========================================================================
// 2. HBO CONTROL VARIABLES (DPO names retained, now Half Bridge)
// =========================================================================

// HBO Output (Duty Cycle 0-1000, Frequency 0-1000)
unsigned int DPO1duty = 0; // Now HBO1 duty cycle
unsigned int DPO2duty = 0; // Now HBO2 duty cycle
unsigned int DPO3duty = 0; // Now HBO3 duty cycle
unsigned int DPO4duty = 0; // Now HBO4 duty cycle

unsigned int DPO1freq = 0; // Now HBO1 frequency
unsigned int DPO2freq = 0; // Now HBO2 frequency
unsigned int DPO3freq = 0; // Now HBO3 frequency
unsigned int DPO4freq = 0; // Now HBO4 frequency

// =========================================================================
// 3. IO STATES AND MAPPINGS (Adjusted for new protocol)
// =========================================================================

// Define the digital pins for the HBO outputs
const int DPO1 = 5; // HBO1
const int DPO2 = 6; // HBO2
const int DPO3 = 9; // HBO3
const int DPO4 = 10; // HBO4

// Define Digital Output Active and Safe States
unsigned long lastCANMessageTime = 0; 
// Safe state of the HBOs (read from EEPROM/CAN 0x6A9)
bool safeStateDPO1, safeStateDPO2, safeStateDPO3, safeStateDPO4; 
// Active state of the HBOs (read from CAN 0x6A9)
bool activeStateDPO1, activeStateDPO2, activeStateDPO3, activeStateDPO4; 

//define the digital pins for the DPI inputs
const int DPI1input = 3; 			 // dpi 1 input (Interrupt Pin)
const int DPI2input = 7; 			 // dpi 2 input
const int DPI3input = 8; 			 // dpi 3 input
const int DPI4input = A4; 			 // dpi 4 input

bool DPI1in = 0; 					 // storage for digital input value as bool
bool DPI2in = 0; 					 // storage for digital input value as bool
bool DPI3in = 0; 					 // storage for digital input value as bool
bool DPI4in = 0; 					 // storage for digital input value as bool

// AVI Thresholds (not implemented as functional logic, but defined for configuration)
unsigned int AVI1_ON_TH = 0;
unsigned int AVI1_OFF_TH = 0;

// DPI/SPI Configuration (stored/read from EEPROM)
// Bit 7-6: Pin Mode (0=Reluctor, 1=Hall Effect, 2=Custom). Default Hall Effect (1)
byte SPI_MODE_DPI1 = 1;

volatile unsigned long lastTime = 0; // DI1 time storage (Freq read)
volatile unsigned long highTime = 0;
volatile unsigned long lowTime = 0;
volatile bool lastState = LOW;
volatile unsigned long lastInterruptTime = 0;
const unsigned long timeoutInterval = 2000000; // 2 seconds in microseconds

int scaledvalue1 = 0; 				 // storage for 12 bit analog value
int scaledvalue2 = 0; 				 // storage for 12 bit analog value
int scaledvalue3 = 0; 				 // storage for 12 bit analog value
int scaledvalue4 = 0; 				 // storage for 12 bit analog value

// Task Intervals (in milliseconds)
unsigned long task1Interval = 100; 	// 100ms (10hz) interval for keep alive frame
unsigned long task2Interval = 10; 	// 10ms (100hz) interval for AVI Status 0x330, 0x332, 0x333
unsigned long task3Interval = 20; 	// 20ms (50hz) interval for DPO safe state check
unsigned long task4Interval = 50; 	// 50ms (20hz) interval for SPI Status 0x6AB
unsigned long task5Interval = 200; 	// 200ms (5hz) interval for Blinking the LED
unsigned long task6Interval = 2000; // 2000ms interval for serial print debug
unsigned long task7Interval = 200; 	// 200ms (5hz) interval for HBO Status (0x6AC)
unsigned long task1Millis = 0; 		// storage for millis counter
unsigned long task2Millis = 0; 		// storage for millis counter
unsigned long task3Millis = 0; 		// storage for millis counter
unsigned long task4Millis = 0; 		// storage for millis counter
unsigned long task5Millis = 0; 		// storage for millis counter
unsigned long task6Millis = 0; 		// storage for millis counter
unsigned long task7Millis = 0; 		// storage for millis counter


// =========================================================================
// 4. HELPER FUNCTIONS
// =========================================================================

/**
 * @brief Extracts a 16-bit value from the buffer (MSB at index, LSB at index+1) 
 * @param buf The CAN receive buffer (rxBuf)
 * @param msbIndex The index of the Most Significant Byte (MSB)
 * @return The 16-bit unsigned integer value.
 */
unsigned int getUint16_MSBLSB(unsigned char buf[], int msbIndex) {
    // Note: This assumes Byte X is MSB and Byte X+1 is LSB.
    // For DPO duty cycle: Byte 1 is MSB, Byte 2 is LSB. msbIndex should be 1.
    // For DPO frequency: Byte 3 is MSB, Byte 4 is LSB. msbIndex should be 3.
    return ((unsigned int)buf[msbIndex] << 8) | (unsigned int)buf[msbIndex + 1];
}

// Function to extract 16-bit value from buffer (LSB at index, MSB at index+1)
// Used for existing AVI/analog value structure
unsigned int getUint16_LSBMSB(unsigned char buf[], int lsbIndex) {
    return (unsigned int)buf[lsbIndex] | ((unsigned int)buf[lsbIndex + 1] << 8);
}


void setup() {

	// start serial port an send a message with delay for starting
	Serial.begin(115200); 	
	Serial.println("IO Board Initializing (New Protocol)...");
	delay(200);

	// initialize canbus with 1000kbit and 8mhz xtal
	if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) 
	Serial.println("MCP2515 Initialized Successfully!");
	else Serial.println("Error Initializing MCP2515...");

	// Set operation mode to normal so the MCP2515 sends acks to received data.
	CAN0.setMode(MCP_NORMAL); 	

	pinMode(CAN0_INT, INPUT); 		// set INT pin to be an input
	digitalWrite(CAN0_INT, HIGH); 	// set INT pin high to enable interna pullup

	// Set the pin mode for HBOs
	pinMode(DPO1, OUTPUT); 
	pinMode(DPO2, OUTPUT); 
	pinMode(DPO3, OUTPUT); 
	pinMode(DPO4, OUTPUT); 

	// Read HBO safe state from EEPROM
	safeStateDPO1 = EEPROM.read(0);
	safeStateDPO2 = EEPROM.read(1);
	safeStateDPO3 = EEPROM.read(2);
	safeStateDPO4 = EEPROM.read(3);
    
    // Apply safe states immediately
    digitalWrite(DPO1, safeStateDPO1);
    digitalWrite(DPO2, safeStateDPO2);
    digitalWrite(DPO3, safeStateDPO3);
    digitalWrite(DPO4, safeStateDPO4);

	// Set the pin mode for DPIs
	pinMode(DPI1input, INPUT_PULLUP);
	pinMode(DPI2input, INPUT_PULLUP);
	pinMode(DPI3input, INPUT_PULLUP);
	pinMode(DPI4input, INPUT_PULLUP);

	// Set up the interrupt for DPI1 (pin D3) for frequency/duty cycle measurement
	attachInterrupt(digitalPinToInterrupt(DPI1input), handleDPI1Interrupt, CHANGE);

	// IO Box A or B Mode (A7 is analog only, hence the analog read)
	pinMode(A7, INPUT); // Set the analog pin 7 as input (needs an external pullup resisitor 10k or 20k)
	int analogValue = analogRead(A7); // Read the analog value from pin A7

	// The old protocol's +/- 1 CAN ID offset is no longer used for the new protocol IDs (0x6A8, 0x4A9, 0x330, etc.)
	// However, if the base ID (0x6A8) is intended to shift, we apply it here.
	// Assuming the new protocol uses fixed IDs (0x6A8, 0x4A9, 0x6A9, 0x6AB, 0x330, 0x332, 0x333) and does not shift.

	// If using the old shifting logic for compatibility with the hardware switch:
	if (analogValue < 512) { // Adjust the threshold as necessary
		// This block is left empty since the new protocol uses absolute IDs, 
		// but I am keeping the structure for future protocol versioning if needed.
	}
	
	pinMode(LED_PIN, OUTPUT);
	Serial.println("All OK"); 	// all ready to go !
}


void loop() {
	unsigned long currentMillis = millis(); 	// Get current time in milliseconds

	// Execute task 1: Send Keep Alive
	if (currentMillis - task1Millis >= task1Interval) {
		task1Millis = currentMillis;
		SendKeepAlive();
	}

	// Execute task 2: Send AVI/SPI Status (New 0x330, 0x332, 0x333)
	if (currentMillis - task2Millis >= task2Interval) {
		task2Millis = currentMillis;
		SendAnalogAndSPIStatus(); // Renamed and changed
	}

	// Execute task 3: HBO Safety Check 
	if (currentMillis - task3Millis >= task3Interval) {
		task3Millis = currentMillis;
		CheckDPOSafeSateTimeout(); // Function name retained for simplicity, now checks HBO timeout
	}

	// Execute task 4: Send DPI/SPI Input Status (New 0x6AB)
	if (currentMillis - task4Millis >= task4Interval) {
		task4Millis = currentMillis;
		SendDPIValues_NewProtocol(); // Renamed and changed
	}
    
    // Execute task 7: Send HBO Status (New 0x6AC)
	if (currentMillis - task7Millis >= task7Interval) {
		task7Millis = currentMillis;
		SendHalfBridgeStatus(); 
	}

	// Execute task 6: Debug Print
	if (currentMillis - task6Millis >= task6Interval) {
		task6Millis = currentMillis;
		//debugPrint();
	}

	// read can buffer when interrupted and jump to canRead for processing.
	if (!digitalRead(CAN0_INT)) 	// If CAN0_INT pin is low, read receive buffer
	{
		CAN0.readMsgBuf(&rxId, &len, rxBuf); 	// Read data: len = data length, buf = data byte(s)
		canRead();
	}
}

// Interrupt service routine for DPI1 (frequency/duty cycle measurement)
void handleDPI1Interrupt() {
	unsigned long currentTime = micros();
	bool currentState = digitalRead(DPI1input);

	if (currentState == HIGH && lastState == LOW) {
		// Rising edge
		lowTime = currentTime - lastTime;
		lastTime = currentTime;
	} else if (currentState == LOW && lastState == HIGH) {
		// Falling edge
		highTime = currentTime - lastTime;
		lastTime = currentTime;
	}

	lastState = currentState;
	lastInterruptTime = currentTime; // Update the last interrupt time
}

/**
 * @brief Processes incoming CAN messages based on their ID and Mux Index.
 * Handles HBO Control (0x6A8), HBO Config (0x6A9), and SPI/AVI Config (0x6A9).
 */
void canRead() {
	lastCANMessageTime = millis();
	
	// Byte 0 contains Mux ID (bits 7-5) and Mux Index (bits 3-0)
	int mux_id = (rxBuf[0] >> 5) & 0x07; // bits 7-5
	int mux_index = rxBuf[0] & 0x0F;     // bits 3-0

	switch (rxId) {
		case HBO_CONTROL_ID: // 0x6A8: HBO Duty Cycle and Frequency (HBO1, HBO2 only)
			if (mux_id == MUX_ID_HBO) { // Must be Mux ID 2 for Half Bridge
				
				// NEW MAPPING: Duty Cycle is 1:7 - 2:0 (16-bit)
				// Frequency is 3:7 - 4:0 (16-bit)
				unsigned int duty_cycle = getUint16_MSBLSB(rxBuf, 1); // MSB at Byte 1, LSB at Byte 2
				unsigned int frequency = getUint16_MSBLSB(rxBuf, 3);  // MSB at Byte 3, LSB at Byte 4
				
				switch (mux_index) {
					case 0: // HBO1
						DPO1duty = duty_cycle;
						DPO1freq = frequency;
						break;
					case 1: // HBO2
						DPO2duty = duty_cycle;
						DPO2freq = frequency;
						break;
					// HBO3 and HBO4 control must come from a different message ID if needed.
					// Since 0x6A8 only defines index 0 and 1, we ignore 2 and 3 here.
					default:
						// Ignore control for HBO3/HBO4 if it arrives on 0x6A8
						break;
				}
				DriveDigitalPin(); 
			}
			break;

		case HBO_CONFIG_ID: // 0x6A9: HBO, SPI, AVI Configuration
			if (mux_id == MUX_ID_HBO) { // Mux ID 2: HBO Configuration
				
				// Byte 1: Bits 2-1: Safe State, Bit 0: Active State
				byte safe_state_raw = (rxBuf[1] >> 1) & 0x03; // Bits 2-1
				bool active_state_raw = bitRead(rxBuf[1], 0);   // Bit 0 (0=low, 1=high)

				// Safe State: 0=inactive (LOW), 1=active (HIGH), 2=open circuit (Treat 1 and 2 as HIGH safe state)
				bool newSafeState = (safe_state_raw == 1 || safe_state_raw == 2);
				
				switch (mux_index) {
					case 0: // HBO1
						if (newSafeState != safeStateDPO1) {
							safeStateDPO1 = newSafeState;
							EEPROM.write(0, safeStateDPO1);
						}
						activeStateDPO1 = active_state_raw;
						break;
					case 1: // HBO2
						if (newSafeState != safeStateDPO2) {
							safeStateDPO2 = newSafeState;
							EEPROM.write(1, safeStateDPO2);
						}
						activeStateDPO2 = active_state_raw;
						break;
					case 2: // HBO3
						if (newSafeState != safeStateDPO3) {
							safeStateDPO3 = newSafeState;
							EEPROM.write(2, safeStateDPO3);
						}
						activeStateDPO3 = active_state_raw;
						break;
					case 3: // HBO4
						if (newSafeState != safeStateDPO4) {
							safeStateDPO4 = newSafeState;
							EEPROM.write(3, safeStateDPO4);
						}
						activeStateDPO4 = active_state_raw;
						break;
				}
				
				// We ignore Retries, Drive Type, Retry Delay, Control Method, and Fuse Current for this board,
				// but the state and safety flags are updated.

				// Re-drive pin if safety state changed while CAN is active
				DriveDigitalPin();
			} 
            else if (mux_id == MUX_ID_SPI) { // Mux ID 3: SPI Configuration
				// Logic to store SPI configuration (Pin Mode, Edge Select, Thresholds, etc.) goes here.
				if (mux_index == 0) { // SPI1
				    // Read Pin Mode (Bits 7-6 of Byte 1)
				    SPI_MODE_DPI1 = (rxBuf[1] >> 6) & 0x03;
				}
			} else if (mux_id == MUX_ID_AVI) { // Mux ID 4: AVI Configuration
				// Logic to store AVI configuration (Switch On/Off Thresholds) goes here.
				if (mux_index == 0) {
				    // Thresholds are 16-bit (mV, 5000 = 5.00V)
				    // Note: The hardware may scale 10V to 5V, but the protocol expects 5V thresholds.
				    AVI1_ON_TH = getUint16_MSBLSB(rxBuf, 2);  // 2:7 - 3:0
				    AVI1_OFF_TH = getUint16_MSBLSB(rxBuf, 4); // 4:7 - 5:0
				}
			}
			break;

		default:
			// Ignore other CAN IDs
			break;
	}
}

/**
 * @brief Checks if the CAN message timeout has occurred.
 * If no CAN message is received within 2 seconds, all HBOs revert to their saved safe state.
 */
void CheckDPOSafeSateTimeout() {
	if (millis() - lastCANMessageTime > 2000) {
		// CAN Bus Timeout: Revert to Safe State
		digitalWrite(DPO1, safeStateDPO1);
		digitalWrite(DPO2, safeStateDPO2);
		digitalWrite(DPO3, safeStateDPO3);
		digitalWrite(DPO4, safeStateDPO4);
		digitalWrite(LED_PIN, LOW); // Extinguish LED state
	} else {
		// CAN Bus is Active: Blink LED and ensure pins are being driven by the last received command
		unsigned long currentMillis = millis(); 

		if (currentMillis - task5Millis >= task5Interval) {
			task5Millis = currentMillis;
			digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the LED
			DriveDigitalPin(); // Ensures PWM is continuously running
		}
	}
}

/**
 * @brief Applies the stored HBO duty cycle (DPOx_duty) to the physical pin, 
 * respecting the configured active state (activeStateDPOx).
 */
void DriveDigitalPin() {
	// The protocol uses Duty Cycle 0-1000 (0.0% to 100.0%)
	// analogWrite uses 0-255

	// ================= HBO 1 Logic =================
	if (DPO1duty == 0) {
		// Duty is 0: Interpret as OFF
		digitalWrite(DPO1, activeStateDPO1 ? LOW : HIGH); // LOW if active HIGH, HIGH if active LOW
	} else { 
		// Duty is 1 to 1000: Interpret as PWM.
		// Map 0-1000 to 0-255. Note: DPO1duty=1000 maps to 255.
		int dutyCycle = map(DPO1duty, 0, 1000, 0, 255);
		
		// If LOW active, inversion is necessary so 100% commanded duty (255) results in 0 analogWrite 
		// (which is 100% HIGH output for an active LOW device).
		if (!activeStateDPO1) {
		    dutyCycle = 255 - dutyCycle;
		}
		analogWrite(DPO1, dutyCycle);
	}

	// ================= HBO 2 Logic =================
	if (DPO2duty == 0) {
		digitalWrite(DPO2, activeStateDPO2 ? LOW : HIGH);
	} else { 
		int dutyCycle = map(DPO2duty, 0, 1000, 0, 255);
		if (!activeStateDPO2) {
		    dutyCycle = 255 - dutyCycle;
		}
		analogWrite(DPO2, dutyCycle);
	}

	// ================= HBO 3 Logic =================
	if (DPO3duty == 0) {
		digitalWrite(DPO3, activeStateDPO3 ? LOW : HIGH);
	} else { 
		int dutyCycle = map(DPO3duty, 0, 1000, 0, 255);
		if (!activeStateDPO3) {
		    dutyCycle = 255 - dutyCycle;
		}
		analogWrite(DPO3, dutyCycle);
	}

	// ================= HBO 4 Logic =================
	if (DPO4duty == 0) {
		digitalWrite(DPO4, activeStateDPO4 ? LOW : HIGH);
	} else { 
		int dutyCycle = map(DPO4duty, 0, 1000, 0, 255);
		if (!activeStateDPO4) {
		    dutyCycle = 255 - dutyCycle;
		}
		analogWrite(DPO4, dutyCycle);
	}
}


/**
 * @brief Sends the Keep Alive heartbeat message (Original protocol).
 */
void SendKeepAlive() {
	byte KeepAlive[5] = { 0X10, 0x09, 0x0D, 0x01, 0x00 };
	CAN0.sendMsgBuf(KeepAliveCANAddress, 0, 5, KeepAlive);
}

/**
 * @brief Sends DPI/SPI status data via the new 0x6AB message.
 */
void SendDPIValues_NewProtocol() {
	byte DPIdata[8] = {0};

	// 1. Calculate the duty cycle and frequency for DPI1
	noInterrupts(); 
	unsigned long currentTime = micros();
	unsigned long period = highTime + lowTime;
	// Calculate frequency in Hz (F = 1 / T in seconds)
	// Frequency = 1,000,000 / Period in microseconds (if Period > 0)
	unsigned int frequency = (period > 0) ? (1000000UL / period) : 0;
	// Duty Cycle (0-100%)
	unsigned long dutyCycle = (period > 0) ? (highTime * 100 / period) : 0; 
	interrupts(); 

	// If timeout, set to 0
	if (currentTime - lastInterruptTime > timeoutInterval) {
		frequency = 0;
		dutyCycle = 0;
	}

	// 2. Read DPI states (DPI2, DPI3, DPI4 only provide state)
	DPI2in = !digitalRead(DPI2input);
	DPI3in = !digitalRead(DPI3input);
	DPI4in = !digitalRead(DPI4input);

	// 3. Pack Message 0x6AB (Mux ID 3 = SPI)
	// For a simpler implementation (all 4 SPI status in 4 sequential messages):
	for (int mux_index = 0; mux_index < 4; mux_index++) {
	    // Reset buffer
	    for (int i = 0; i < 8; i++) DPIdata[i] = 0; 

	    bool state = false;
	    unsigned int voltage = 0;
	    unsigned int duty = 0;
	    unsigned int freq = 0;

	    // Get values based on index
	    if (mux_index == 0) { // DPI1/SPI1
	        state = !digitalRead(DPI1input); // Current state for SPI1
	        // AVI input reading (5V scale)
	        voltage = map(analogRead(A0), 0, 1023, 0, 5000); 
	        duty = dutyCycle * 10; // Duty Cycle * 10 (0-1000)
	        freq = frequency * 100; // Frequency * 100 (Hz, 1000 = 1000Hz)
	    } else if (mux_index == 1) { // DPI2/SPI2
	        state = DPI2in;
	        voltage = map(analogRead(A1), 0, 1023, 0, 5000);
	    } else if (mux_index == 2) { // DPI3/SPI3
	        state = DPI3in;
	        voltage = map(analogRead(A2), 0, 1023, 0, 5000);
	    } else if (mux_index == 3) { // DPI4/SPI4
	        state = DPI4in;
	        voltage = map(analogRead(A3), 0, 1023, 0, 5000);
	    }

	    // Byte 0: Mux ID (3=SPI) [7-5] and Mux Index [3-0]
	    DPIdata[0] = (MUX_ID_SPI << 5) | (mux_index & 0x0F);

	    // Byte 1: State [0]
	    DPIdata[1] |= (state << 0);

	    // Byte 2, 3: Voltage (mV, 16-bit)
	    DPIdata[2] = (byte)(voltage >> 8); // MSB
	    DPIdata[3] = (byte)voltage;        // LSB

	    // Byte 4, 5: Duty Cycle (%, 1000 = 100.0%) (Only non-zero for DPI1)
	    DPIdata[4] = (byte)(duty >> 8); // MSB
	    DPIdata[5] = (byte)duty;        // LSB

	    // Byte 6, 7: Frequency (Hz, 1000 = 1000Hz) (Only non-zero for DPI1)
	    DPIdata[6] = (byte)(freq >> 8); // MSB
	    DPIdata[7] = (byte)freq;        // LSB

	    CAN0.sendMsgBuf(SPI_STATUS_ID, 0, 8, DPIdata);
        delay(1); // Small delay to avoid flooding the bus when sending 4 messages sequentially
	}
}


/**
 * @brief Sends AVI and SPI Status messages (0x330, 0x332, 0x333) at 100Hz.
 */
void SendAnalogAndSPIStatus() {

	// AVI Voltage and State (0x330)
	// AVI voltage is scaled 0-5000mV (5V max input from Arduino ADC).
	unsigned int avi_v[4];
	// 0-1023 maps to 0-5000mV
	avi_v[0] = map(analogRead(A0), 0, 1023, 0, 5000); 
	avi_v[1] = map(analogRead(A1), 0, 1023, 0, 5000);
	avi_v[2] = map(analogRead(A2), 0, 1023, 0, 5000);
	avi_v[3] = map(analogRead(A3), 0, 1023, 0, 5000);
    
    // Determine State: For this simple implementation, we'll assume state is ON (1) if voltage > 100mV
    bool avi_state[4];
    for(int i = 0; i < 4; i++) {
        avi_state[i] = (avi_v[i] > 100);
    }

	byte AVIdata1[8] = {0};

	// --- AVI 1 PACKING (15-bit Voltage: 0:6 - 1:0) ---
	// Byte 0: Bit 7 = State, Bits 6-0 = Voltage MSBs (7 bits)
	AVIdata1[0] |= (avi_state[0] << 7);
	// Get bits 8-14 of the 15-bit value (MSBs)
	AVIdata1[0] |= (avi_v[0] >> 8) & 0x7F;  

	// Byte 1: Bits 0-7 = Voltage LSBs (8 bits)
	// Get bits 0-7 of the 15-bit value (LSBs)
	AVIdata1[1] |= avi_v[0] & 0xFF;         

	// --- AVI 2 PACKING (15-bit Voltage: 2:6 - 3:0) ---
	// Byte 2: Bit 7 = State, Bits 6-0 = Voltage MSBs (7 bits)
	AVIdata1[2] |= (avi_state[1] << 7);
	AVIdata1[2] |= (avi_v[1] >> 8) & 0x7F;

	// Byte 3: Bits 0-7 = Voltage LSBs (8 bits)
	AVIdata1[3] |= avi_v[1] & 0xFF;

	// --- AVI 3 PACKING (15-bit Voltage: 4:6 - 5:0) ---
	// Byte 4: Bit 7 = State, Bits 6-0 = Voltage MSBs (7 bits)
	AVIdata1[4] |= (avi_state[2] << 7);
	AVIdata1[4] |= (avi_v[2] >> 8) & 0x7F;

	// Byte 5: Bits 0-7 = Voltage LSBs (8 bits)
	AVIdata1[5] |= avi_v[2] & 0xFF;

	// --- AVI 4 PACKING (15-bit Voltage: 6:6 - 7:0) ---
	// Byte 6: Bit 7 = State, Bits 6-0 = Voltage MSBs (7 bits)
	AVIdata1[6] |= (avi_state[3] << 7);
	AVIdata1[6] |= (avi_v[3] >> 8) & 0x7F;

	// Byte 7: Bits 0-7 = Voltage LSBs (8 bits)
	AVIdata1[7] |= avi_v[3] & 0xFF;
    
	CAN0.sendMsgBuf(AVI_STATUS_ID_1, 0, 8, AVIdata1); 


    // SPI Voltage and State (0x332) - Uses the same 15-bit packing
	byte SPIdata1[8] = {0};
    
    // SPI 1-4 Voltage uses the same scaled value as AVI 1-4 (0-5000mV scale)
    bool spi_state[4];
    spi_state[0] = !digitalRead(DPI1input);
    spi_state[1] = !digitalRead(DPI2input);
    spi_state[2] = !digitalRead(DPI3input);
    spi_state[3] = !digitalRead(DPI4input);
    
	// --- SPI 1 PACKING (15-bit Voltage: 0:6 - 1:0) ---
	SPIdata1[0] |= (spi_state[0] << 7);
	SPIdata1[0] |= (avi_v[0] >> 8) & 0x7F;
	SPIdata1[1] |= avi_v[0] & 0xFF; 

	// --- SPI 2 PACKING (15-bit Voltage: 2:6 - 3:0) ---
	SPIdata1[2] |= (spi_state[1] << 7);
	SPIdata1[2] |= (avi_v[1] >> 8) & 0x7F;
	SPIdata1[3] |= avi_v[1] & 0xFF;

	// --- SPI 3 PACKING (15-bit Voltage: 4:6 - 5:0) ---
	SPIdata1[4] |= (spi_state[2] << 7);
	SPIdata1[4] |= (avi_v[2] >> 8) & 0x7F;
	SPIdata1[5] |= avi_v[2] & 0xFF;

	// --- SPI 4 PACKING (15-bit Voltage: 6:6 - 7:0) ---
	SPIdata1[6] |= (spi_state[3] << 7);
	SPIdata1[6] |= (avi_v[3] >> 8) & 0x7F;
	SPIdata1[7] |= avi_v[3] & 0xFF;
    
	CAN0.sendMsgBuf(SPI_STATUS_ID_1, 0, 8, SPIdata1); 
    
    // SPI Frequency (0x333)
	byte SPIdata2[8] = {0};
    
    // Calculate frequency for DPI1 (SPI1)
    unsigned long currentMicros = micros(); 
    noInterrupts(); 
	unsigned long period = highTime + lowTime;
	unsigned int frequency_dpi1 = (period > 0) ? (1000000UL / period) : 0;
	if (currentMicros - lastInterruptTime > timeoutInterval) frequency_dpi1 = 0; 
	interrupts(); 

    // Conversion: 1 = 0.25Hz (So actual frequency * 4 = raw value)
    unsigned int raw_freq_dpi1 = frequency_dpi1 * 4;
    
    // SPI 1 Freq (16-bit: 0:7 - 1:0)
    SPIdata2[0] = (byte)(raw_freq_dpi1 >> 8); // MSB
    SPIdata2[1] = (byte)raw_freq_dpi1;        // LSB
    
    // SPI 2, 3, 4 Freq are 0 for all other DPIs (as per user request)
    // SPIdata2[2-7] remain 0 (default initialized)

	CAN0.sendMsgBuf(SPI_STATUS_ID_2, 0, 8, SPIdata2); 
}

/**
 * @brief Sends HBO status data (Voltage, Load %, Current) via the new 0x6AC message.
 */
void SendHalfBridgeStatus() {
	byte HBOdata[8] = {0};

    // HBO Voltage is typically battery voltage. Using the 0-10V scale of AVI for system voltage indication.
	unsigned int hbo_v[4];
	// HBO Voltage is 12000mV max (12.000V). Using A0-A3 reading scaled to 12V max as a placeholder.
	hbo_v[0] = map(analogRead(A0), 0, 1023, 0, 12000); 
	hbo_v[1] = map(analogRead(A1), 0, 1023, 0, 12000);
	hbo_v[2] = map(analogRead(A2), 0, 1023, 0, 12000);
	hbo_v[3] = map(analogRead(A3), 0, 1023, 0, 12000);

	// For a simpler implementation (all 4 HBO status in 4 sequential messages):
	for (int mux_index = 0; mux_index < 4; mux_index++) {
	    // Reset buffer
	    for (int i = 0; i < 8; i++) HBOdata[i] = 0; 

        // Set status defaults
        byte retry_count = 0; // Placeholder
        byte pin_state = 0;   // 0 = operational (Placeholder)
        unsigned int voltage = hbo_v[mux_index];
        byte load_percent = 0;
        byte hs_current = 0; // High side current (Placeholder 8-bit)

        // Calculate Load % from Duty Cycle (Map 0-1000 to 0-100)
        if (mux_index == 0) load_percent = (DPO1duty / 10);
        else if (mux_index == 1) load_percent = (DPO2duty / 10);
        else if (mux_index == 2) load_percent = (DPO3duty / 10);
        else if (mux_index == 3) load_percent = (DPO4duty / 10);

	    // Byte 0: Mux ID (2=HBO) [7-5] and Mux Index [3-0]
	    HBOdata[0] = (MUX_ID_HBO << 5) | (mux_index & 0x0F);

	    // Byte 1: Retry Count [7-3] and Pin State [2-0]
        HBOdata[1] |= (retry_count << 3) & 0xF8; // Bits 7-3
        HBOdata[1] |= pin_state & 0x07;          // Bits 2-0

	    // Byte 2, 3: Half Bridge Voltage (mV, 16-bit, 12000=12.000V)
	    HBOdata[2] = (byte)(voltage >> 8); // MSB
	    HBOdata[3] = (byte)voltage;        // LSB

	    // Byte 4: Load (%)
        HBOdata[4] = load_percent;

	    // Byte 5: Reserved (0, already initialized)

	    // Byte 6, 7: HS Current (8-bit unsigned)
	    HBOdata[6] = hs_current;
        HBOdata[7] = 0; // Reserved

	    CAN0.sendMsgBuf(HBO_STATUS_ID, 0, 8, HBOdata);
        delay(1); // Small delay to avoid flooding the bus when sending 4 messages sequentially
	}
}

void debugPrint() {
	Serial.print("HBO1 Duty: ");
	Serial.println(DPO1duty);
	Serial.print("HBO1 Freq: ");
	Serial.println(DPO1freq);
	Serial.print("HBO1 Safe State: ");
	Serial.println(safeStateDPO1);
	Serial.print("HBO1 Active State: ");
	Serial.println(activeStateDPO1);
	Serial.print("AVI1 Voltage (mV, 5V Scale): ");
	// FIX: debugPrint uses 5V scaling
	Serial.println(map(analogRead(A0), 0, 1023, 0, 5000));
    noInterrupts(); 
    Serial.print("DPI1 Freq (Hz): ");
    Serial.println((highTime + lowTime > 0) ? (1000000UL / (highTime + lowTime)) : 0);
    interrupts();
}