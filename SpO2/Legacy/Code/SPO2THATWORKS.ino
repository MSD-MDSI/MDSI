

#include <SPI.h>
#include <MCP_DAC.h>

// The Arduino pins used for slave select / chip select (SS/CS)
#define SS_PIN1 5
#define SS_PIN2 3

// Configure the DAC (Check your MCP model, e.g., MCP4921)
MCP_DAC dac1(&SPI);
MCP_DAC dac2(&SPI);

// Parameters
int spO2[] = {100,99,98,97,96,95,94,93,92,91,90,89,88,87,86,85,84,
83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,63,
62,61,60,59,58,57,56,55,54,53,52,51,50};

double ampR[] = {0.3,0.4,0.4,0.4,0.4,0.3,0.3,0.4,0.4,0.4,0.4,0.4,0.4,
0.4,0.4,0.4,0.4,0.4,0.5,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,
0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,
0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4};

double ampIR[] = {1.0,0.9,0.7,1.0,0.9,0.7,0.7,0.6,0.6,0.58,0.57,0.54,
0.52,0.5,0.5,0.48,0.47,0.46,0.45,0.445,0.44,0.43,0.42,0.39,0.4,
0.39,0.38,0.38,0.37,0.35,0.36,0.35,0.35,0.34,0.34,0.34,0.33,0.32,
0.32,0.31,0.3,0.3,0.3,0.29,0.29,0.28,0.28,0.27,0.27,0.26,0.25};

double minR = 1.45;
double minIR = 1.45;

int spO2_old = 100;
float HR_old = 60.0; // HR is often handled as float for precision
double result_sR, result_sIR;
float new_params[2];

const int volt_max = 1023 * 0.39; 

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for Serial Monitor to open
  }
  Serial.println("--- DAC TEST STARTING ---");
  
  pinMode(SS_PIN1, OUTPUT);
  pinMode(SS_PIN2, OUTPUT);
  digitalWrite(SS_PIN1, HIGH); // Set pins HIGH (idle) initially
  digitalWrite(SS_PIN2, HIGH);
  
  dac1.begin(SS_PIN1);
  dac2.begin(SS_PIN2);
  Serial.println("DACs Initialized.");
}

void loop() {
  // 1. Check for Serial updates without stopping the program
  if (Serial.available() > 0) {
    read_params();
    HR_old = new_params[0];
    spO2_old = (int)new_params[1];

    // Confirmation message so you know it worked
    Serial.print("Update Received! HR: ");
    Serial.print(HR_old);
    Serial.print(" | SpO2: ");
    Serial.println(spO2_old);
  }

  // 2. Call the generator
  exec_ppg(HR_old, spO2_old);
}

void exec_ppg(float HR, int current_spO2) {
  if (HR < 10) HR = 60.0; 
  
  int index = findIndex(spO2, current_spO2);
  if (index == -1) index = 0;

  int total_steps = 60;
  // Calculate exactly how long each tiny step should last in milliseconds
  // Formula: (60 seconds / HR) * 1000ms / steps
  float step_duration_ms = (60.0 / HR * 1000.0) / (float)total_steps;

  for (int i = 0; i < total_steps; i++) {
    // We want t to go from 0 to (60/HR) seconds over the course of the loop
    float t = ((60.0 / HR) / (float)total_steps) * (float)i;
    
    ppg_parameters(minR, ampR[index], minIR, ampIR[index], t, HR, result_sR, result_sIR);

    // 1. Reduce the multiplier from 800 to 500 to "shrink" the height of the wave
    // 2. Adjust the subtraction to 1.4 to "center" the wave better
    long valR = (result_sR - 1.4) * 500;   
    long valIR = (result_sIR - 1.4) * 500;

    // 3. Keep the constrain! It prevents the DAC from getting "garbage" data
    uint16_t finalR = constrain(valR, 0, 1023);
    uint16_t finalIR = constrain(valIR, 0, 1023);

    dac1.write(finalR);
    dac2.write(finalIR);

    // 4. DEBUG (Check these values in the Serial Plotter!)
    Serial.print(finalR);
    Serial.print(",");
    Serial.println(finalIR);
    delay(step_duration_ms); 
  }
}
    
int findIndex(int arr[], int value) {
  for(int i = 0; i < 51; i++) {
    if(arr[i] == value) return i; 
  }
  return -1; 
}

void ppg_parameters(double mR, double aR, double mIR, double aIR, float t, float HR, double &resR, double &resIR) {
  double pi = 3.14159265;
  double f = HR / 60.0;
  
  // Basic heartbeat shape (Fundamental + 2nd Harmonic)
  // This creates the "double hump" or "notch" look
  float alpha = (0.5 * sin(2.0 * pi * t * f)) + 
                (0.25 * sin(4.0 * pi * t * f + 0.5)); // Added harmonic for the notch
  
  // Offset the wave so it stays positive
  alpha += 1.0; 

  resR = mR + (aR * alpha);
  resIR = mIR + (aIR * alpha);
}

void read_params() {
  String cad_frec = "";
  String cad_sat = "";
  bool parsing_sat = false;

  while(Serial.available() > 0) {
    char num = Serial.read();
    if(num == ',') {
      parsing_sat = true;
      continue;
    }
    if(num == '\n' || num == '\r') break;

    if(!parsing_sat) {
      cad_frec += num;
    } else {
      cad_sat += num;
    }
  }

  if(cad_frec.length() > 0) new_params[0] = cad_frec.toFloat();
  if(cad_sat.length() > 0) {
    int s = cad_sat.toInt();
    if(s < 50) s = 50;
    new_params[1] = s;
  }
}
