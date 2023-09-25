#include <HX711_ADC.h>
#include <SPI.h>
#include <SD.h>

/*
This program is licenced under the Creative Commons Zero V1.0 Universal Licence

- Settings Are Changed In The Config File And Are Automatically Applied
*/

//SD
File dataFile;
File configFile;
const int sdChipSelect = 8;
bool logData = true; // Should always be true unless system is being tested
int dataLogIntervalSlow_ms = 100; //Constants for data log period.
int dataLogIntervalFast_ms = 10;
int dataLogInterval_ms = 100;


//LoadCell
float motorLoadThreshold = 10;
const int HX711_dout = 9; // HX711 dout pin
const int HX711_sck = 10; // HX711 sck pin
const int loadSampleRate = 50; 
float currentCellData = 0; // Current value of load cell -> declared here so that it can be referenced anywhere
float globalLoadMovingAve; //Just for reading
float calibrationValueFromConfig; //Calibration Value stored in the config file on the SD card
bool loadCellIsCalibrated = false;
int cellCalibrationState = 0;
HX711_ADC LoadCell(HX711_dout, HX711_sck);


//System
bool sysArmed = false;
bool ABORT = false;
bool testLoadcell = false;
int systemState = 0;
const int stateIndicatorLED_GRN = 3;
const int stateIndicatorLED_RED = 5;
const int stateIndicatorLED_BLU = 2;
const int ignitionPyroPin = 4;
const int indicatorBuzzer = 6;
bool allowBuzzer = true;
float loopTime, aveLoopTime, timeOfLastLoop;

//Time
unsigned long cellTime, sdTime, statusIndTime, statusIndTimeLocked, dataSafeEndTime, loopTimeGlobal, systemOnTime_s = 0;
int countdownLength_s = 30;
float testTime_s, countdownEndTime_ms, dataSafeLength_s;

void setup() {

  InitializePins();

  Serial.begin(115200);
  Serial.println("Starting...");  

  IndicateStartup();  

  InitializeSD();
  InitializeCell();
  ProcessConfig();
  PrintSettings();

  testTime_s = -countdownLength_s;

  digitalWrite(stateIndicatorLED_GRN, HIGH);

  Serial.println("\n > Ready to calibrate, begin with 'c'. Load calibration value from config with 'l'");
  while (!loadCellIsCalibrated) CalibrateCell();

  ResetIndicators();

  Serial.println("Online. Standing By.");
}

void loop() {

  WatchCommands();

  if (systemState == 0) { //Standby
    TimeKeeper();
    IndicateStandby();

    GetLoadCellData();
    ManageStandby();
  }
  else if (systemState == 1) { //Countdown 
    TimeKeeper();
    IndicateCountdown();

    GetLoadCellData();
    ManageCountdown();

    WriteDataToSD();
  } 
  else if (systemState == 2) { // Ignite motor
    TimeKeeper();
    IndicateIgnition();

    GetLoadCellData();
    ManageIgnition();
    FireIgnitionPyro();
    
    WriteDataToSD();  
  }
  else if (systemState == 3) { // Motor burn 
    TimeKeeper();
    IndicateBurn();

    GetLoadCellData();
    ManageBurn();

    WriteDataToSD();
  } 
  else if (systemState == 4) { // Make sure we haven't stopped recording data by accident
    TimeKeeper();
    IndicateCountdown();

    GetLoadCellData();
    ManageEndBurnDataSafe();

    WriteDataToSD();
  }
  else if (systemState == 5) { // Motor is burnt
    TimeKeeper();
    ManageEndBurnStandby();

    IndicateEndBurnStandby();
  }
  else if(systemState == 42) { // Abort
    TimeKeeper();
    IndicateAbort();
    if (dataFile) EndDataWrite();
  }
}

//==GENERAL FUNCTIONS==

void WatchCommands() {
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 'S') sysArmed = true;
    if (inByte == 'A') systemState = 42;
    if (inByte == 'T') testLoadcell = !testLoadcell;// Displays loadcell values in Serial Monitor
  }
}

void InitializePins() {
  pinMode(stateIndicatorLED_GRN, OUTPUT);
  pinMode(stateIndicatorLED_RED, OUTPUT);
  pinMode(stateIndicatorLED_BLU, OUTPUT);
  pinMode(ignitionPyroPin, OUTPUT);
}

void PrintSettings() {
  Serial.println();
  Serial.print("Countdown Length: ");
  Serial.print(countdownLength_s);
  Serial.println("s");
  Serial.print("Buzzer On: ");
  Serial.println(allowBuzzer ? "true" : "false");
  Serial.print("Data Safe Length: ");
  Serial.println(dataSafeLength_s);
}

//==LOADCELL==

void  InitializeCell() {
  Serial.print("Initializing Loadcell...");
  LoadCell.begin();

  unsigned long stabilizingtime = 2000;
  boolean _tare = true;

  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    digitalWrite(stateIndicatorLED_RED, HIGH);
    if (allowBuzzer) { tone(indicatorBuzzer, 100, 200); }
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0);
  }

  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct | THIS TAKES TIME, DELAYS PROGRAM

  if (allowBuzzer) { tone(indicatorBuzzer, 1500, 50); }
  Serial.println("done.");
}

void CalibrateCell() { //Calibrate load cell
  float known_mass;

  // Waits to start calibration
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 'c') cellCalibrationState++;
    if (inByte == 'l') cellCalibrationState = -1;
  }

  if (cellCalibrationState == 0) {
    known_mass = 0;
  }
  
  // User directions
  if (cellCalibrationState == 1) {
    Serial.print("   Calibration Started. Ready to tare. Begin with 't'");
    cellCalibrationState++;
  }

  // Tares load cell
  if (cellCalibrationState == 2) {
    LoadCell.update();
      if (Serial.available() > 0) {
        char inByte = Serial.read();
        if (inByte == 't') { LoadCell.tareNoDelay(); Serial.print(" - "); }
      }
    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      cellCalibrationState++;
      Serial.println("   Place known mass on loadcell. Enter into Serial Monitor.");
    }
  }

  // Recieves known mass
  if (cellCalibrationState == 3) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("   Known mass is: ");
        Serial.println(known_mass);
        cellCalibrationState++;
      }
    }
  }

  // Gets and sets calibration value
  if (cellCalibrationState == 4) {
    float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //get the new calibration value
    loadCellIsCalibrated = true;

    SaveLoadCellCalibrationValueToConfig(newCalibrationValue);

    Serial.print(" > New calibration value set to: ");
    Serial.print(newCalibrationValue);
    Serial.println("\n");
  }

  // Loads Calibration Value from config file (state set to -1 as not to interfere with main calibration process)
  if (cellCalibrationState == -1) { 
    Serial.println(" > Loading calibration value from config.");
    LoadCell.tareNoDelay();

    LoadCell.setCalFactor(calibrationValueFromConfig);

    Serial.print(" > Calibration value set to: ");
    Serial.print(calibrationValueFromConfig);
    Serial.println("\n");

    loadCellIsCalibrated = true;
  }
}

void GetLoadCellData() { //Gets data from loadcell
  static boolean newDataReady = 0;

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady && logData) {
    if (millis() > cellTime + loadSampleRate && loadCellIsCalibrated) {
      float i = LoadCell.getData();
      currentCellData = i;
      newDataReady = 0;
      cellTime = millis();
    }
  }
}

//==SD==

void InitializeSD() { //Initializes the SD card reader

  ResetIndicators();

  if (!logData) Serial.println("\n DATA LOGGING DISABLED \n");

  dataLogInterval_ms = dataLogIntervalSlow_ms;

  Serial.print("Initializing SD card...");
  delay(100);
  if (!SD.begin(sdChipSelect)) {
    Serial.println("initialization failed!");
    digitalWrite(stateIndicatorLED_RED, HIGH);
    if (allowBuzzer) { tone(indicatorBuzzer, 100, 200); }
    while (1);
  }

  dataFile = SD.open("data.csv", FILE_READ);
  String fileString = dataFile.readString();

  String headerString = "System_State, System_On_Time_s, Test_Time_s, Load_Cell_Data_g, Load_Cell_Data_Ave_g, Calibration_State, Data_Log_Interval_ms, Data_Log_Rate_Hz, Loop_Run_Time_micros";

	if (fileString[0] != 'S') { // Checks whether headers have been declared already or not
    dataFile.close();
    dataFile = SD.open("data.csv", FILE_WRITE);
    dataFile.println(headerString);
  } else if (fileString[0] = 'S') {
    Serial.print("DATA FOUND ON CARD...");    
  }

  dataFile.close();
  dataFile = SD.open("data.csv", FILE_WRITE);

  if (allowBuzzer) { tone(indicatorBuzzer, 1500, 50); }

  Serial.println("done.");
}

void ProcessConfig() { //Processes config file
  int lineBegin, lineEnd, configVars;
  String line, fileContent;

  configFile = SD.open("config.txt", FILE_READ);
  fileContent = configFile.readString();

  lineBegin = 0;
  configVars = 0;

  for (int i = 0; i < fileContent.length(); i++) {
    if (fileContent[i] == '\n') {
      lineEnd = i;
      for (int l = 0; l < (lineEnd - lineBegin); l++) {
        line += fileContent[lineBegin + l];
      }

      if (line[0] == '*') {
        ProcessVariableLine(line);
        configVars++;
      }

      lineBegin = i + 1;
      line = "";
    }
  }

  Serial.println(String(configVars) + " Variables found in Config.");
  configFile.close(); 
}

void ProcessVariableLine(String line) { //Extracts variable values from line 
  String varName, varValStr;
  int varVal;

  for (int i = 0; i < line.indexOf(':') - 1; i++) {
    varName += line[i + 1];
  }

  for (int i = 0; i < line.indexOf(';') - line.indexOf(':') - 1; i++) {
    varValStr += line[line.indexOf(':') + 1 + i];
  }

  if (varName == "CL") {
    varVal = varValStr.toInt();
    countdownLength_s = varVal;
  } else if (varName == "LCV") {
    calibrationValueFromConfig = varValStr.toFloat();
  } else if (varName == "MLT") {
    motorLoadThreshold = varValStr.toFloat();
  } else if (varName == "DLF") {
    dataLogIntervalFast_ms = varValStr.toInt();
  } else if (varName == "DLS") {
    dataLogIntervalSlow_ms = varValStr.toInt();
  } else if (varName == "BS") {
    allowBuzzer = varValStr.toInt();
  } else if (varName == "DSL") {
    dataSafeLength_s = varValStr.toFloat();
  } else {
    Serial.println();
    Serial.println("! The '" + varName + "' variable saved in config was not accounted for in code. !");
    Serial.println();
  }
}

void SaveLoadCellCalibrationValueToConfig(float calValue) {
  configFile = SD.open("config.txt", FILE_READ);
  String fileContent = configFile.readString();

  fileContent.replace(String(calibrationValueFromConfig), String(calValue));

  configFile.close();

  configFile = SD.open("config.txt", FILE_WRITE | O_TRUNC);
  
  configFile.println(fileContent);

  configFile.close();

  Serial.println(" > Calibration Value Saved to Config.");
}

void WriteDataToSD() {

  //DataFile is opened in initializeSD() and closed in ManageBurn().

  unsigned long dataLogRate_hz = 1000 / dataLogInterval_ms;

  CalcLoopTime();

  if (millis() > sdTime + dataLogInterval_ms && dataFile && logData) {
    dataFile.println(String(systemState) + ", " + String(systemOnTime_s) + ", " + String(testTime_s) + ", "+ String(currentCellData) + ", " + String(globalLoadMovingAve) + ", " + String(cellCalibrationState) + ", " + String(dataLogInterval_ms) + ", " + String(dataLogRate_hz) + ", " + String(loopTimeGlobal)); 
    sdTime = millis(); 
  }

  if (!dataFile && logData) Serial.println("Error writing to file.");
}

void EndDataWrite() {
  dataFile.close();
  logData = false;
}

//==SYSTEM==

void AdvanceState() {
  systemState++;
  ResetIndicators();
}

void ManageStandby() {
  if (sysArmed == true) {
    AdvanceState(); 
    countdownEndTime_ms = millis() + (countdownLength_s * 1000);
  }

  if (testLoadcell) {
      Serial.println(currentCellData);
    }
}

void ManageCountdown() {

  dataLogInterval_ms = dataLogIntervalSlow_ms;

  if(testTime_s >= 0) {
    AdvanceState();
  }
} 

void ManageIgnition() { 
  dataLogInterval_ms = dataLogIntervalFast_ms;

  if (currentCellData > motorLoadThreshold) AdvanceState(); digitalWrite(ignitionPyroPin, LOW);
}

void ManageBurn() {
  digitalWrite(ignitionPyroPin, LOW);

  if (MovingLoadAve(currentCellData) < motorLoadThreshold) {
    AdvanceState();
    dataSafeEndTime = millis() + (dataSafeLength_s * 1000);
  }
}

void ManageEndBurnDataSafe() { // Ensures that we do not lose data if the system mistakenly ends data recording
  if (currentCellData > motorLoadThreshold) {
    systemState--;
  }

  if (dataSafeEndTime <= millis()) {
    AdvanceState();
  }
}

void ManageEndBurnStandby() {
  if (dataFile) {
    EndDataWrite();
  }
}

void FireIgnitionPyro() {
  digitalWrite(ignitionPyroPin, HIGH);
}

float MovingLoadAve(float value) {
  const int nvalues = 50; // Moving average sample size

  static int current = 0; // Moving average window size
  static int cvalues = 0; // Count of values read
  static float sum = 0;
  static float values[nvalues];

  sum += value;
  
  if (cvalues == nvalues) {
    sum -= values[current];
  }
  values[current] = value;  

  if (++current >= nvalues)
    current = 0;

  if (cvalues < nvalues)
    cvalues += 1;

  globalLoadMovingAve = sum/cvalues;
  return sum/cvalues;
}

void CalcLoopTime() { // Calculates Time of one clock cycle
  loopTime = micros() - timeOfLastLoop;
  timeOfLastLoop = micros();
  if (loopTime < 60000) { loopTimeGlobal = loopTime; }
}

//==TIME==

void TimeKeeper() {

  systemOnTime_s = millis() / 1000.00;

  if (countdownEndTime_ms != 0) { testTime_s = (millis() - countdownEndTime_ms) / 1000.00; } // How long mission is (current event, i.e. testing a motor - any time that includes countodwn, pyro firing, motor burn, etc.)
}

//==STATE INDICATION==

void IndicateStartup() {
  if (allowBuzzer) {
    tone(indicatorBuzzer, 750, 500);
    delay(500);
    tone(indicatorBuzzer, 1250, 50);
    delay(100);
    tone(indicatorBuzzer, 1250, 50);
    delay(100);
    tone(indicatorBuzzer, 1250, 50);
    delay(100);
  }
  
  digitalWrite(stateIndicatorLED_GRN, HIGH);
  delay(200);
  digitalWrite(stateIndicatorLED_GRN, LOW);
  digitalWrite(stateIndicatorLED_RED, HIGH);
  delay(200);
  digitalWrite(stateIndicatorLED_RED, LOW);
  digitalWrite(stateIndicatorLED_BLU, HIGH);
  delay(200);
  digitalWrite(stateIndicatorLED_BLU, LOW);
}

void IndicateStandby() {

  if (millis() > statusIndTime + 100) {
    
    if (statusIndTimeLocked >= 3000) {    
      statusIndTimeLocked = 0;
    } else {
      statusIndTimeLocked += 100;      
    }

    statusIndTime = millis();
  }

  if (statusIndTimeLocked == 0 && allowBuzzer) {
    tone(indicatorBuzzer, 750);
  } else { 
    noTone(indicatorBuzzer); 
  }
  if (statusIndTimeLocked == 0 || statusIndTimeLocked == 1000 || statusIndTimeLocked == 2000) {
    digitalWrite(stateIndicatorLED_GRN, HIGH);
  } else {
    digitalWrite(stateIndicatorLED_GRN, LOW);
  }

}

void IndicateCountdown() {
  
  if (millis() > statusIndTime + 100) {

    if (statusIndTimeLocked >= 1000) {
      statusIndTimeLocked = 0;
    } else {
      statusIndTimeLocked += 100;      
    }

    statusIndTime = millis();
  }

  if (statusIndTimeLocked == 0 && allowBuzzer) {
    tone(indicatorBuzzer, 1250);
  } else { 
    noTone(indicatorBuzzer); 
  }
  if (statusIndTimeLocked == 0 || statusIndTimeLocked == 300) {
    digitalWrite(stateIndicatorLED_BLU, HIGH);
  } else {
    digitalWrite(stateIndicatorLED_BLU, LOW);
  }
}

void IndicateIgnition() {
  if (millis() > statusIndTime + 100) {

    if (statusIndTimeLocked >= 1000) {
      statusIndTimeLocked = 0;
    } else {
      statusIndTimeLocked += 100;      
    }

    statusIndTime = millis();
  }

  if ((statusIndTimeLocked == 0 || statusIndTimeLocked == 100) && allowBuzzer) {
    tone(indicatorBuzzer, 1500);
  } else { 
    noTone(indicatorBuzzer); 
  }
  digitalWrite(stateIndicatorLED_BLU, HIGH);
}

void IndicateBurn() {
  if (millis() > statusIndTime + 100) {

    if (statusIndTimeLocked >= 1000) {
      statusIndTimeLocked = 0;
    } else {
      statusIndTimeLocked += 100;      
    }

    statusIndTime = millis();
  }

  if ((statusIndTimeLocked == 0 || statusIndTimeLocked == 100) && allowBuzzer) {
    tone(indicatorBuzzer, 1250  );
  } else { 
    noTone(indicatorBuzzer); 
  }
  if (statusIndTimeLocked == 0 || statusIndTimeLocked == 300) {
    digitalWrite(stateIndicatorLED_RED, HIGH);
  } else {
    digitalWrite(stateIndicatorLED_RED, LOW);
  }
}

void IndicateEndBurnStandby() {
  if (millis() > statusIndTime + 100) {

    if (statusIndTimeLocked >= 3000) {
      statusIndTimeLocked = 0;
    } else {
      statusIndTimeLocked += 100;      
    }

    statusIndTime = millis();
  }

  if (statusIndTimeLocked == 0 && allowBuzzer) {
    tone(indicatorBuzzer, 750);
  } else { 
    noTone(indicatorBuzzer); 
  }
  if (statusIndTimeLocked == 0) {
    digitalWrite(stateIndicatorLED_GRN, HIGH);
  } else {
    digitalWrite(stateIndicatorLED_GRN, LOW);
  }
}

void IndicateAbort() {
  if (millis() > statusIndTime + 100) {

    if (statusIndTimeLocked >= 3000) {
      statusIndTimeLocked = 0;
    } else {
      statusIndTimeLocked += 100;      
    }

    statusIndTime = millis();
  }

  if (statusIndTimeLocked % 100 == 0 && allowBuzzer) {
    tone(indicatorBuzzer, 2000); // #TESTING - make sure tone is loudest and as noticable as possible
  } else { 
    noTone(indicatorBuzzer); 
  }
  if (statusIndTime % 100 == 0) { //#TESTING - check to see if this runs reliably
    digitalWrite(stateIndicatorLED_GRN, HIGH);
  } else {
    digitalWrite(stateIndicatorLED_GRN, LOW);
  }
}

void ResetIndicators() {
  digitalWrite(stateIndicatorLED_GRN, LOW);
  digitalWrite(stateIndicatorLED_BLU, LOW);
  digitalWrite(stateIndicatorLED_RED, LOW);

  digitalWrite(indicatorBuzzer, LOW);
}
