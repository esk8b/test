/*
 Elektroskate.ino
 
 Dieses Programm ermoeglicht die Steuerung eines Eletroskateboards mittels Nunchuck
 Copyright (C) [2012]  [Barney]
 
 This program is free software; you can redistribute it and/or modify it under the terms of 
 the GNU General Public License as published by the Free Software Foundation; either 
 version 3 of the License, or (at your option) any later version.
	
 This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 See the GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License along with this program; 
 if not, see <http://www.gnu.org/licenses/>. 
 
 
 Dieses Programm ist freie Software. Sie koennen es unter den Bedingungen der GNU General Public License, 
 wie von der Free Software Foundation veroeffentlicht, weitergeben und/oder modifizieren, 
 entweder gemaess Version 3 der Lizenz oder (nach Ihrer Option) jeder spaeteren Version.
 
 Die Veroeffentlichung dieses Programms erfolgt in der Hoffnung, dass es Ihnen von Nutzen sein wird, 
 aber OHNE IRGENDEINE GARANTIE, sogar ohne die implizite Garantie der MARKTREIFE 
 oder der VERWENDBARKEIT FUeR EINEN BESTIMMTEN ZWECK. Details finden Sie in der GNU General Public License.
 
 Sie sollten ein Exemplar der GNU General Public License zusammen mit diesem Programm 
 erhalten haben. Falls nicht, siehe <http://www.gnu.org/licenses/>. 
 
 */

// Libraries 
#include <Wire.h>                         // i2c Lib fuer 2 Wire Protokoll(fuer Nunchuck benoetigt)
#include <Nunchuk.h>                      // Lib fuer Nunchuk auslesen modifiziert von Barney (modifizierte Wii Lib) 
#include <OneWire.h>                      // Lib um Dallas Temperaturbausteine auszulesen von http://www.pjrc.com/teensy/arduino_libraries/OneWire.zip
#include <ADC.h>                          // http://forum.pjrc.com/threads/25532-ADC-library-update-now-with-support-for-Teensy-3-1
#include "Elektroskate_Custom_Settings.h" // Festlegen des Debugging und der Funktionalitaeten wie Messungen usw.
#include "Elektroskate_Custom_Parameters.h" // Festlegen der persoenlichen Werte
#include "Elektroskate_Connection.h"      // Festlegen der elektrischen Anschluesse

// Definition der ISR Zeitroutinen
IntervalTimer timer_1;                    // 1 Sekunden Timer (z.B. Ausgabe von Messwerten, Temperaturmessung)
IntervalTimer timer_2;                    // 0.1 Sekunden Timer (z.B. Messung von Spannungs- und Stromwerten und Nunchuk Daten)
IntervalTimer timer_3;                    // 0.05 Sekunden Timer (Motorsteuerung, Umschaltung Licht/Hupe, Hase Igel, DirektDrive,...)

// Definition der ADC / DAC Eigenschaften
#define AnalogReadResolutionBits 12       // Bit-Aufloesung fuer die Analog Digital Umsetzer (ADC) zur Strom- und Spannungsmessung
#define AnalogReadResolutionValue (uint32_t) pow(2,AnalogReadResolutionBits)   // Festlegen der ADC-Aufloesung (Wertebereich)
#define AnalogAveraging 8                 // Durchschnittswert bei der ADC-Messung bilden
#define AnalogWriteResolutionBits 16      // Bit-Aufloesung fuer die Digital Analog Umsetzer (DAC)fuer die PWM-Ansteuerung des Motorstellers (Servo)
#define AnalogWriteResolutionValue (uint32_t) pow(2,AnalogWriteResolutionBits) // Festlegen der DAC-Aufloesung (Wertebereich)

// Nunchuk - Steuerparameter Sollwertvorgabe und Programmablaufsteuerung
Nunchuk nunchuk = Nunchuk();

ADC *adc = new ADC(); // adc object

uint16_t analogX = 0;         
uint16_t analogY = 0;
uint16_t analogX_12bit = 0;
uint16_t analogY_12bit = 0;

// Mapping der Joystickparameter (Nunchuk) auf 12bit Zahlenraum
uint16_t  YAchseNeutral_12bit = (YAchseNeutral - YAchseMin) * 4095 / (YAchseMax - YAchseMin);   
uint16_t  YAchseUT_12bit      = (YAchseUT      - YAchseMin) * 4095 / (YAchseMax - YAchseMin);
uint16_t  YAchseOT_12bit      = (YAchseOT      - YAchseMin) * 4095 / (YAchseMax - YAchseMin);
uint16_t  XAchseLinks_12bit   = (XAchseLinks   - XAchseMin) * 4095 / (XAchseMax - XAchseMin);
uint16_t  XAchseRechts_12bit  = (XAchseRechts  - XAchseMin) * 4095 / (XAchseMax - XAchseMin);

uint16_t cButton = 0;
uint16_t zButton = 0;
uint16_t accelX;
uint16_t accelY;
uint16_t accelZ;
uint16_t cButtonZero = LOW;            // C-Knopf Nunchuck auf Low setzen
uint16_t zButtonZero = LOW;            // Z-Knopf Nunchuck auf Low setzen
uint8_t  PairingStatus = LOW;          // Beim Einschalten besteht kein Pairing
uint16_t PairingLostCounter = 0;       // wird hoch gezaehlt, wenn accelX hintereinander gleiche Werte liefert
uint16_t PairingSuccessCounter = 0;    // wird hoch gezaehlt, wenn accelX hintereinander ungleiche Werte liefert
uint16_t accelXAlt = 0;                // Alter Beschleunigungswert X fuers Pairing und Funkabrisserkennung

// PWM Motorsteuerung
#define PWMFrequenz 50                 // PWM Frequenz fuer die Ansteuerung des Motorstellers: alle 20ms einen Impuls in der Laenge von 1.0 bis 2.0ms

#define ObererStellWertMotorIgel (uint32_t) float(PWM_ObererStellWertMotorIgel*AnalogWriteResolutionValue*PWMFrequenz/1000) // Wert berechnen fuer Oebere Pulsweite Igel (1.7ms Pulsweite)
#define ObererStellWertMotorHase (uint32_t) float(PWM_ObererStellWertMotorHase*AnalogWriteResolutionValue*PWMFrequenz/1000) // Wert berechnen fuer Oebere Pulsweite Hase (2.0ms Pulsweite)

uint32_t BremsDaempfungUmschalt           = BremsDaempfung;
uint32_t BeschleunigungsDaempfungUmschalt = BeschleunigungsDaempfung;

//#if DirectDrive
  #define LeerlaufStellWertMotor   (uint32_t) float(PWM_LeerlaufStellWertMotor*AnalogWriteResolutionValue*PWMFrequenz/1000) // Wert berechnen fuer Leerlauf (1.5ms Pulsweite)
//#else
//  #define LeerlaufStellWertMotor   (uint32_t) float(PWM_LeerlaufStellWertMotor*AnalogWriteResolutionValue*PWMFrequenz/1000) + LeerlaufOffset // Wert berechnen fuer Leerlauf (1.5ms Pulsweite)
//#endif
#define UntererStellWertMotor    (uint32_t) float(PWM_UntererStellWertMotor*AnalogWriteResolutionValue*PWMFrequenz/1000) // Wert berechnen fuer untersten Wert (1.0ms Pulsweite)

// Welcher Modus soll gestartet werden
uint32_t ObererStellWertMotor = ObererStellWertMotorIgel;  // Im Igel Modus starten
uint8_t  ObererStellWertMotorUmschaltCounter = 0;          // Zaehler fuer die Umschaltung Hase Igel (HI)

uint8_t  DriveModeUmschaltCounter = 0;                     // Zaehler fuer die Umschaltung DirectDrive/ IntegralDrive

uint32_t StellWertMotor    = LeerlaufStellWertMotor; // Alle Werte fuer den Motor werden auf Leerlauf eingestellt
uint32_t StellWertMotorAlt = LeerlaufStellWertMotor; // Wird fuer den Ueberstromfall bennoetigt Alle Werte fuer den Motor werden auf Leerlauf eingestellt

uint16_t AbwurfGefahr = LOW;   // Soll versehentliches Abbremsen aus voller Fahrt verhindern

// Temperatursensoren
uint16_t TempSensor;

// Ubatt, Strom und Leistungsmessung
float Ubatt = 0;                      // Spannung Batterie in V gemessen
float Strom[10] = {0};                // Strommessung Array, der Strom kann auch negativ werden, es wird ein Array zum Mitteln verwendet
uint16_t StromIndex = 0;              // Zeiger fuer den Strom
float Momentanleistung = 0;           // Aktuelle Leistung, die gerade aufgenommen wird
float Leistung = 0;                   // Leistungsberechnung
#define LeistungNorm (float) 36000    // Normierung der Leistung auf 1h bei 10 Messungen/s
#define UbattFaktor (float) BattSpgMMax/AnalogReadResolutionValue  // Umrechnungsfaktor UbattMessbarMax/Analoge Referenz Spannung
#define StromFaktor (float) 2*StromMMax/AnalogReadResolutionValue  // Umrechnungsfaktor Strom

// Strombegrenzung und Kurzschlusserkennung
#define Reduktionsgewichtung (float) (100/(Ishort-Imax)) // Der Reduktionsfaktor Bf = 100-((Imess-Imax)*Reduktionsgewichtung) in %
float Reduktionsfaktor = 0;           // Reduktionsfaktor(%): Stellwert = (Stellwertmomentan-Motorneutralstellung)*Reduktionsfaktor + Motorneutralstellung
float Imomentan = 0;                  // Momentanstrom, wird abgeleitet von Strom[0] oder Ishort
uint8_t F_Ueberstrom = LOW;           // Sollte ein Uebserstrom erkannt werden, wird eine Reduzierung des Motorstellwertes durchgefuehrt

// Zeitmessung von Programmabschnitten
uint32_t time;

// -------------------------------------------------------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------------------------------------------------------

void setup()
{
  // ADC Einstellungen
  pinMode(Pin_Ubatt, INPUT);                            // Spannungsmesspin aktivieren
  pinMode(Pin_Strom, INPUT);                            // Strommesspin aktivieren
  // ADC_0
  adc->setReference(ADC_REF_EXTERNAL, ADC_0);           // Bei Extern wird die ca. 3.3V Versorgungsspannung des Teensys benutzt
  adc->setAveraging(AnalogAveraging, ADC_0);            // Anzahl der Durchschnitswertbildungen
  adc->setResolution(AnalogReadResolutionBits, ADC_0);  // Aufloesung in Bits setzen
  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_0);   // change the conversion speed
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_0);     // change the sampling speed
  // ADC_1 diese Einstellung werden erst wirksam, wenn die speziellen ADC-Pins verwendet werden.
  // 16 (A2), 17 (A3), 34-36 (A10-A13)
  adc->setReference(ADC_REF_EXTERNAL, ADC_1);           // Bei Extern wird die ca. 3.3V Versorgungsspannung des Teensys benutzt
  adc->setAveraging(AnalogAveraging, ADC_1);            // Anzahl der Durchschnitswertbildungen
  adc->setResolution(AnalogReadResolutionBits, ADC_1);  // Aufloesung in Bits setzen
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_1);   // change the conversion speed
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_1);     // change the sampling speed
  // kann nur verwendet werden, wenn nicht mehr die Pins A0 und A1, sondern A0 und A2 verwendet werden.
  // Vorteil ist, dass die Abfragezeit der ADC unter einer us sinkt
  //adc->startContinuous(readPin, ADC_0);                // Starte die Dauermessung der beiden
  //adc->startContinuous(readPin2, ADC_1);               // ADC im Hintergrund. Dies beschleunigt die Messwerterfassung um das
                                                         // 150 Fache. Auch hohe Average stören nicht mehr
  // Die Abfrage der ADC muesste entsprechend angepasst werden
  //adc->analogReadContinuous(ADC_0);                    // Hole den Messwert des ADC_0 ab Achtung asynchron
  //adc->analogReadContinuous(ADC_1);                    // Hole den Messwert des ADC_0 ab Achtung asynchron

  // DAC Einstellungen
  analogWriteFrequency(Pin_Motorstelleranschluss_A, PWMFrequenz); // Einstellen der PWM Parameter fuer den Motorsteller
  analogWriteFrequency(Pin_Motorstelleranschluss_B, PWMFrequenz); // Einstellen der PWM Parameter fuer den Motorsteller
  analogWriteFrequency(Pin_Brems_Servo, PWMFrequenz);             // Einstellen der PWM Parameter fuer den Motorsteller
  analogWriteResolution(AnalogWriteResolutionBits);               // Analog Aufloesung auf 16 Bit einstellen

  // Zyklischer IRQ-Timer die einen Software Interrupt ausloest
  timer_1.begin(TimerT1, 1000000);  // 1 Sekunden Timer
  timer_2.begin(TimerT2,  100000);  // 0.1 Sekunden Timer
  timer_3.begin(TimerT3,   50000);  // 0.05 Sekunden Timer

  if (DEBUG) Serial.begin(DEBUG_BAUDRATE);   // Debugausgabe ueber USB Schnittstelle definieren
  Serial_BT.begin(BT_BAUDRATE);              // Werteausgabe ueber Serieller mittels Bluetooth definieren

  // Alle Ausgangspins definieren
  pinMode(Pin_Hupe, OUTPUT);          // Hupe ist ein Ausgang
  pinMode(Pin_Licht, OUTPUT);         // Licht ist ein Ausgang
  pinMode(Pin_Bremse, OUTPUT);        // Bremslicht ist ein Ausgang
  pinMode(Pin_Blinker_L, OUTPUT);     // Blinker Links ist ein Ausgang
  pinMode(Pin_Blinker_R, OUTPUT);     // Blinker Rechts ist ein Ausgang
  pinMode(Pin_ReceiverLED, OUTPUT);   // ReceiverLED ist ein Ausgang
  pinMode(Pin_PowerSwitch, OUTPUT);   // Power MOSFET ist ein Ausgang

  // Startwert der Ausgaegnge definieren
  digitalWrite(Pin_Hupe, LOW);        // Hupe aus
  digitalWrite(Pin_Licht, LOW);       // Licht aus
  digitalWrite(Pin_Bremse, LOW);      // Bremslicht aus
  digitalWrite(Pin_Blinker_L, LOW);   // Blinker Links aus
  digitalWrite(Pin_Blinker_R, LOW);   // Blinker Rechts aus
  digitalWrite(Pin_ReceiverLED, LOW); // Empfangsstatus LED aus 

  // Leistungselektronik ueber PWM soft hochfahren
  if (LESOFT) {
    for(uint32_t PSPWM=0; PSPWM<20; PSPWM++){
      digitalWrite(Pin_PowerSwitch, LOW);  // Leistungselektronik soft einschalten
      delay(20-PSPWM);		           // Pause bis high wird immer kuerzer
      digitalWrite(Pin_PowerSwitch, HIGH); // Leistungselektronik an
      delay(PSPWM);                        // Pause bis low wird immer laenger
    }
    digitalWrite(Pin_PowerSwitch, HIGH);   // Leistungselektronik dauerhaft an
  }
}

// -------------------------------------------------------------------------------------------------------------------
// MAIN
// -------------------------------------------------------------------------------------------------------------------

void loop()
{
  if (DEBUG_Funktion) Serial_DB.println("Funktion: Loop");
  if (PairingStatus == LOW) verbinden(); // Versuche mit Nunchuk neu zu verbinden, wenn Funkabriss erkannt worden ist.
  delay(100);
}

// -------------------------------------------------------------------------------------------------------------------
// SUBROUTINEN
// -------------------------------------------------------------------------------------------------------------------

void TimerT1() {
// Verbindung zum Controller herstellen und Pairing pruefen, Ausgabe Temperatur, Strom, Spannung, Leistung                          
  if (DEBUG_ISR) Serial_DB.println("ISR: Timer1Hz");
  if (TEMPMESS){
    for (uint8_t TempSensor = 0; TempSensor <= defTempSensorAnzahl-1; TempSensor++) {
      Serial_BT.print(Temperaturmessung(TempSensor, defTempSensorAnzahl,defTempAddress),2);  // Temperaturmessung und Ausgabe auf BT oder Serial Monitor
      Serial_BT.print("C, ");
    }
  }
  if (UIMESS) UIausgeben();             // Anzeigen von Strom, Ubatt und Leistung an BT oder Serial Monitor
}

void TimerT2() {
// Messung Strom, Spannung, Leistung und Ueberstromerkennung
  if (DEBUG_ISR) Serial_DB.println("ISR: Timer10Hz");
  if (UIMESS) {
    UIMessung();                            // Messung und Berechnungen Strom, UBatt, Leistung durchfuehren
    F_Ueberstrom = LOW;                     // Ueberstrom Flag loeschen
    if(Strom[StromIndex] >= Imax){
      if (Strom[StromIndex] >= Ishort){     // Strom groesser als der Kurzschlussstrom?
        if (DEBUG_Messung) Serial_DB.println("Kurzschluss!"); 
        Imomentan = Ishort;                 // dann begrenzen auf Ishort
      }
      else Imomentan = Strom[StromIndex];   // wenn nicht ist der Momentanstrom der gemessende Strom
      F_Ueberstrom = HIGH;                  // Motorstellroutine benachrichtigen, dass der Strom zu gross ist
      if (DEBUG_Messung) {
        Serial_DB.print("Ueberstrom :");
        Serial_DB.println(Imomentan);
      } 
      Reduktionsfaktor = (100-((Imomentan-Imax)*Reduktionsgewichtung))/100; // Der Reduktionsfaktor kann beim Kurzschluss 100% erreichen
    }
  }
}

void TimerT3() {
  //time = millis();
  //Serial_DB.println(time);         // prints time since program started 
  if (PairingStatus == HIGH){
    Funkabriss();                  // Nunchuk auslesen bei gleichzeitiger Prüfung auf Funkabriss
    Motorsteuerung();              // Motorsteuerung
    if (LICHTHUPE) Licht_Hupe();   // Licht und Hupe
    Hase_Igel();                   // Geschwindigkeitsumschaltung
    BlinkerRL();                   // Blinker Rechts Links
    Drive_Mode();                  // Umschaltung DirectDrive / Integral Mode
  }
}

static inline int8_t sgn(int16_t val) {
  if (val < 0) return -1;
  if (val >= 0) return 1;
}

void verbinden(){
// Verbindung zum Controller herstellen und Pairing pruefen
  if (DEBUG_Funktion) Serial_DB.println("Funktion: verbinden");
  if (analogX_12bit && analogX_12bit == 0xFF || accelX > 1024  || PairingStatus == LOW){
    if (DEBUG_Nunchuk) Serial_DB.println("Die Kommunikation zum Nunchuck muss erneut hergestellt werden");
    PairingStatus = LOW;                                                // Verbindung zun Controller steht nicht
    digitalWrite(Pin_ReceiverLED,LOW);                                  // ReceiverLED aus
    StellWertMotor = LeerlaufStellWertMotor;                            // StellWertMotor auf Leerlauf setzen
    analogWrite(Pin_Motorstelleranschluss_A, LeerlaufStellWertMotor);   // PWM Register Motorstellwert-A auf Leerlauf
    analogWrite(Pin_Motorstelleranschluss_B, LeerlaufStellWertMotor);   // PWM Register Motorstellwert-B auf Leerlauf
    analogWrite(Pin_Brems_Servo, PWM_Bremse_Ungebremst*AnalogWriteResolutionValue*PWMFrequenz/1000); // PWM Register Bremsservo auf Bremse loesen    
    PairingSuccessCounter = 0;                                          // ParingSuccessCounter zuruecksetzen
    while (PairingStatus == LOW){                                       // Solange pruefen, bis die Verbindung steht
      accelXAlt = accelX;                                               // Alten Wert des Beschleunigungssensors ablegen
      Nunchuk_auslesen();
      if (accelX != accelXAlt && accelX < 1024 && analogY_12bit !=0){   // analogY_12bit = 0 ohne pairing (pruefen)
        PairingSuccessCounter++;                                        // Unterschiedliche empfangene Werte hochzaehlen bis ...
        if (PairingSuccessCounter > PairingSuccessTrigger){             // ... Schwellwert ueberschritten
          PairingLostCounter = 0;                                       // ParingLostCounter zuruecksetzen
          PairingStatus = HIGH;                                         // Verbindung zun Controller steht (wieder)
        }
      }
    }
  }
}

void  Nunchuk_auslesen(){
// Nunchuck Hilfsvariablen auslesen und global zur Verfuegung stellen
  
  if (DEBUG_Funktion) Serial_DB.println("Funktion: Nunchuk");
  if (PairingStatus == LOW) nunchuk.init();   // Nunchuck Kommunikation initialisieren (muss hier rein zum Pairing ohne USB Kabel)
  nunchuk.update();                           // Neue Werte vom Controller holen und in globalen Variablen ablegen
  analogX = nunchuk.analogX;
  analogY = nunchuk.analogY;
  
  if (analogX <= XAchseMin) analogX = XAchseMin; // Sicherheitsfunktion
  if (analogX >= XAchseMax) analogX = XAchseMax;
  if (analogY <= YAchseMin) analogY = YAchseMin;
  if (analogY >= YAchseMax) analogY = YAchseMax; 
  
  analogX_12bit= map(analogX, XAchseMin, XAchseMax, 0, 4095);
  analogY_12bit= map(analogY, YAchseMin, YAchseMax, 0, 4095);
  
  cButton = nunchuk.cButton;
  zButton = nunchuk.zButton;
  accelX = nunchuk.accelX;
  accelY = nunchuk.accelY;
  accelZ = nunchuk.accelZ;

  if (DEBUG_Nunchuk){
    Serial_DB.print("Nunchuk: ");
    Serial_DB.print(" X: ");
    Serial_DB.print(analogX_12bit);
    Serial_DB.print(" Y: ");
    Serial_DB.print(analogY_12bit);
    Serial_DB.print(" cB: ");
    Serial_DB.print(cButton);
    Serial_DB.print(" zB: ");
    Serial_DB.println(zButton);
  }
}

void Funkabriss(){
// Totmannschalter und Funkabriss pruefen
  if (DEBUG_Funktion) Serial_DB.println("Funktion: Funkabriss");
  digitalWrite(Pin_ReceiverLED,HIGH);               // Pairing steht, Status ReceiverLED an
  accelXAlt = accelX;                   
  Nunchuk_auslesen();                   
  if (accelX == accelXAlt || accelXAlt > 1024){     // Wahrscheinlichkeitspruefung Funkabriss
    PairingLostCounter++;
    if (PairingLostCounter > PairingLostTrigger){   // Schwellwert ueberschritten
      digitalWrite(Pin_ReceiverLED,LOW);            // Status LED aus
      PairingStatus = LOW;                          // Verbindung zun Controller steht nicht
    }
  }
  else PairingLostCounter = 0;
}

void Motorsteuerung(){
// PWM-Stellwert Motorservo ermitteln und vorgeben

int16_t Y_UT, Y_OT;        // Abstand zum unteren (UT) und oberen Totpunkt (OT)
int8_t sgnY_UT, sgnY_OT;   // Lage der Sollwertvorgabe der Y-Achse relativ zum oberen bzw. unteren Totpunkt (+/-1)
int8_t u, v, w;            // Hilfsvariablen zur Bereichserkennung der Y-Achse analogY_12bit relativ zu UT, OT (artifiziell)
                           //       y-Achse <  UT: u= 0, v=-1, w=+1
                           // UT <= y-Achse <  OT: u=+1, v= 0, w=-1
                           //       y-Achse >= OT: u= 0, v=+1, w=+1
uint8_t a, b;              // Hilfsvariablen für die Überführung der Bereichserkennung in eine Rechenvorschrift
                           // Die Binärfaktoren a und b können dabei nur die Werte 0 und 1 annehmen, um gezielt unterschiedliche Terme der Berechnungsvorschrift zu aktivieren
						   //       y-Achse <  UT: a=1 && b=0
                           // UT <= y-Achse <  OT: a=0 && b=0
                           //       y-Achse >= OT: a=0 && b=1
						   
uint32_t StellWertMotorMaxUT;   //Maxialer Wertebereich für den Motorstellwert unterhalb UT
uint32_t StellWertMotorMaxOT;   //Maxialer Wertebereich für den Motorstellwert oberhalb OT

  if (DEBUG_Funktion) Serial_DB.println("Funktion: Motorsteuerung");
 
  StellWertMotorMaxUT = LeerlaufStellWertMotor - UntererStellWertMotor;
  StellWertMotorMaxOT = ObererStellWertMotor - LeerlaufStellWertMotor;
 
  Y_UT = analogY_12bit - YAchseUT_12bit;
  Y_OT = analogY_12bit - YAchseOT_12bit;

  // Bereich des Stellwertes der y-Achse ermitteln
  sgnY_UT = sgn(Y_UT);
  sgnY_OT = sgn(Y_OT);
  u = 0.5 * (sgnY_UT - sgnY_OT);
  v = 0.5 * (sgnY_UT + sgnY_OT);
  w =       (sgnY_UT * sgnY_OT);
  a = 0.5 * (u - v + w);
  b = 0.5 * (u + v + w);

  if (zButton == HIGH) {
    if (sgnY_OT) AbwurfGefahr = HIGH;
    StellWertMotor = int(StellWertMotor) * !DirectDrive + int(LeerlaufStellWertMotor) * DirectDrive
                     + a * sgnY_UT * pow(abs(Y_UT),YAchseExpNeg)/pow(YAchseUT_12bit,YAchseExpNeg)        * StellWertMotorMaxUT / BremsDaempfungUmschalt
                     + b *           pow(abs(Y_OT),YAchseExpPos)/pow((4095-YAchseOT_12bit),YAchseExpPos) * StellWertMotorMaxOT / BeschleunigungsDaempfungUmschalt;
  }
  
  if (zButton == LOW) {   // Gleichbehandlung von INTEGRATIONSSTEUERUNG UND DIRECT DRIVE bei losgelassenem Z-Knopf 
    StellWertMotor = LeerlaufStellWertMotor;   // Freilauf des Motors sobald die Z-Taste losgelassen wird
    if (a==0 && b==0) AbwurfGefahr = LOW;      // Direktes Bremsen darf nicht aus versehen ausgelöst werden
    if (a==1 && b==0 && AbwurfGefahr==LOW){    // Direktes Abbremsen wenn zusaetzlich der untere Totpunkt unterschritten wird
      StellWertMotor = LeerlaufStellWertMotor + sgnY_UT * pow(abs(Y_UT),YAchseExpNeg)/pow(YAchseUT_12bit,YAchseExpNeg) * StellWertMotorMaxUT;
    }
  }
  
  if (StellWertMotor < UntererStellWertMotor) StellWertMotor = UntererStellWertMotor;   // untere Grenze fuer den Motorregler
  if (StellWertMotor > ObererStellWertMotor)  StellWertMotor = ObererStellWertMotor;    // obere Grenze fuer den Motorregler

  // Uberstrom detektiert, Motor abbremsen bis auf LeerlaufStellWertMotor
  if (F_Ueberstrom == HIGH && StellWertMotor > LeerlaufStellWertMotor){
    StellWertMotor = LeerlaufStellWertMotor+(StellWertMotorAlt-LeerlaufStellWertMotor)*Reduktionsfaktor;
    if (DEBUG_Motorsteller){
      Serial_DB.print("Reduktionsfaktor: ");
      Serial_DB.println(Reduktionsfaktor);
    }
  }
  StellWertMotorAlt = StellWertMotor;  // Referenzwert für die kontinuierliche Reduktion des Motorstellwertes bei Überstrom
  
  analogWrite(Pin_Motorstelleranschluss_A, StellWertMotor);   // Geschwindigkeitsvorgabe Motor-A in PWM-Register setzen
  analogWrite(Pin_Motorstelleranschluss_B, StellWertMotor);   // Geschwindigkeitsvorgabe Motor-B in PWM-Register setzen
  
  // Bremslich und Servo
  if (StellWertMotor <  LeerlaufStellWertMotor) {
    digitalWrite(Pin_Bremse, HIGH);                       // Bremslicht an
    analogWrite(Pin_Brems_Servo, PWM_Bremse_Vollbremse*AnalogWriteResolutionValue*PWMFrequenz/1000);  // PWM Register Bremsservo auf Vollbremse
    if (DEBUG_Motorsteller) Serial_DB.println(PWM_Bremse_Vollbremse*AnalogWriteResolutionValue*PWMFrequenz/1000); 
  }
  else {
    digitalWrite(Pin_Bremse, LOW);                        // Bremslicht aus
    analogWrite(Pin_Brems_Servo, PWM_Bremse_Ungebremst*AnalogWriteResolutionValue*PWMFrequenz/1000);  // PWM Register Bremsservo auf Bremse loesen
    if (DEBUG_Motorsteller) Serial_DB.println(PWM_Bremse_Ungebremst*AnalogWriteResolutionValue*PWMFrequenz/1000);
  }

  if (DEBUG_Motorsteller){
    Serial_DB.print("StellWertMotor: ");
    Serial_DB.print(StellWertMotor);
    if (StellWertMotor == LeerlaufStellWertMotor) Serial_DB.println(" Neutralstellung");
    if (StellWertMotor >  LeerlaufStellWertMotor) Serial_DB.println(" Beschleunigung");
    if (StellWertMotor <  LeerlaufStellWertMotor) Serial_DB.println(" Bremsen");
  }
}

void Licht_Hupe(){   
// Licht und Hupe benutzen den selben Taster sind aber durch die Nunchuk X-Achse getrennt
  if (DEBUG_Funktion) Serial_DB.println("Funktion: Licht_Hupe");
  if (cButton == LOW) cButtonZero = LOW;                         // Trigger auf ansteigende Flanke, damit Licht nicht blinkt
  if (cButton == HIGH && cButtonZero == LOW && analogX_12bit < XAchseLinks_12bit) {  // Toggle Licht Nunchuck X < 30     
    digitalWrite(Pin_Licht, !digitalRead(Pin_Licht));
    cButtonZero = HIGH;                                          // Licht gegen Blinken sperren
  }
  if (cButton == HIGH && analogX_12bit > XAchseLinks_12bit) {    // Hupe an solange C-Taste gedrueckt und X > 30
    digitalWrite(Pin_Hupe, HIGH);
  }
  else {
    digitalWrite(Pin_Hupe, LOW); 
  }
}

void BlinkerRL(){   
// Licht und Hupe benutzen den selben Taster sind aber durch die Nunchuk X-Achse getrennt
  if (DEBUG_Funktion) Serial_DB.println("Funktion: BlinkerRL");
  if (analogX_12bit < XAchseLinks_12bit) {  // Blinker Links wenn Nunchuck X < XAchseLinks     
    digitalWrite(Pin_Blinker_L, HIGH);      // Blinker Links an
  }
  if (analogX_12bit > XAchseRechts_12bit) { // Blinker Rechts wenn Nunchuck X > XAchseRechts
    digitalWrite(Pin_Blinker_R, HIGH);      // Blinker Rechts an
  }
  else {
      digitalWrite(Pin_Blinker_L, LOW);     // Blinker Links aus
      digitalWrite(Pin_Blinker_R, LOW);     // Blinker Rechts aus
  }
}

void Hase_Igel(){
// Umschaltung Maximalgeschwindigkeitsbegrenzung durch Z-Button und analoger X-Achse
  if (DEBUG_Funktion) Serial_DB.println("Funktion: Hase_Igel");
  if (zButton == LOW) zButtonZero = LOW;
  if (zButton == HIGH && zButtonZero == LOW && analogX_12bit < XAchseLinks_12bit) {   
    ObererStellWertMotorUmschaltCounter ++;          // Counter fuer Geschwindigkeitsumschaltung hochzaehlen 
    if (LICHTHUPE) {
      digitalWrite(Pin_Hupe, HIGH);                  // Zur Bestaetigung 200ms hupen
      delay (200);
      digitalWrite(Pin_Hupe, LOW);
    }
    zButtonZero = HIGH;                              // Routine gegen Festhalten des Z-Tasters sperren
    if (ObererStellWertMotorUmschaltCounter == 3) {  // Es muss dreimal die Z-Taste und der X-Joystick ausgeloest werden
      if      (ObererStellWertMotor == ObererStellWertMotorHase) ObererStellWertMotor = ObererStellWertMotorIgel; // Auf Igel umschalten
      else if (ObererStellWertMotor == ObererStellWertMotorIgel) ObererStellWertMotor = ObererStellWertMotorHase; // Auf Hase umschalten
      ObererStellWertMotorUmschaltCounter = 0;       // Counter zuruecksetzen
      if (LICHTHUPE) {
        digitalWrite(Pin_Hupe, HIGH);                // Zur Bestaetigung der Umschaltung 500ms hupen
        delay (500);
        digitalWrite(Pin_Hupe, LOW);
      }  
    }
  }
}

void Drive_Mode(){
// Umschaltung des Drive Modes durch Z-Button und analoger X-Achse
  if (DEBUG_Funktion) Serial_DB.println("Funktion: Drvie_Mode");
  if (zButton == LOW) zButtonZero = LOW;
  if (zButton == HIGH && zButtonZero == LOW && analogX_12bit > XAchseRechts_12bit) {   
    DriveModeUmschaltCounter ++;          // Counter fuer Geschwindigkeitsumschaltung hochzaehlen 
    if (LICHTHUPE) {
      digitalWrite(Pin_Hupe, HIGH);                  // Zur Bestaetigung 200ms hupen
      delay (200);
      digitalWrite(Pin_Hupe, LOW);
    }
    zButtonZero = HIGH;                              // Routine gegen Festhalten des Z-Tasters sperren
    if (DriveModeUmschaltCounter == 3) { // Es muss dreimal die Z-Taste und der X-Joystick ausgeloest werden
      if      (DirectDrive == true) {
        DirectDrive = false;             // Auf IntegralDrive umschalten
        BremsDaempfungUmschalt = BremsDaempfung;
        BeschleunigungsDaempfungUmschalt = BeschleunigungsDaempfung;
      }
      else if (DirectDrive == false) {
        DirectDrive = true;              // Auf DirectDrive umschalten
        BremsDaempfungUmschalt = 1;
        BeschleunigungsDaempfungUmschalt = 1;
      }
      if (DEBUG_DriveMode) {
        Serial_DB.print("DriveModeUmschaltCounter: ");
        Serial_DB.println(DriveModeUmschaltCounter);
        Serial_DB.print("DriveMode: ");
        Serial_DB.println(DirectDrive);
      }
      DriveModeUmschaltCounter = 0;       // Counter zuruecksetzen
      if (LICHTHUPE) {
        digitalWrite(Pin_Hupe, HIGH);                // Zur Bestaetigung der Umschaltung 500ms hupen
        delay (500);
        digitalWrite(Pin_Hupe, LOW);
      }  
    }
  }
}

void UIMessung(){
// Messen und berechnen von Strom Ubatt und Leistung
  if (DEBUG_Funktion) Serial_DB.println("Funktion: UIMessung");
  Ubatt = adc->analogRead(Pin_Ubatt, ADC_0)*UbattFaktor; // Ubatt in V messen
  Strom[StromIndex] = (adc->analogRead(Pin_Strom, ADC_1)*StromFaktor)-Strom0A; // Strom in A messen und in Array ablegen
  Momentanleistung = Ubatt * Strom[StromIndex];        // Berechnung der Momentanleistung
  StromIndex++;                                        // StromIndex increment
  if (StromIndex > 9) StromIndex = 0;                  // 10 Messwerte im Kreis messen
  Leistung = Leistung + Momentanleistung/LeistungNorm; // Berechnung der insgesamt umgesetzten Stundenleistungsberechnung
  if (DEBUG_Messung) {
    Serial_DB.print("Spannung: ");
    Serial_DB.println(Ubatt);
    Serial_DB.print("Strom: ");
    Serial_DB.println(Strom[0]);
  } 
}

void UIausgeben(){
// Ausgabe der Messwerte ueber die Serielle und oder Bluetooth
  if (DEBUG_Funktion) Serial_DB.println("Funktion: UIausgeben");
  Serial_BT.print(Ubatt);              // Spannung ausgeben
  Serial_BT.print("V, ");
  Serial_BT.print(Iaverage());         // Durchschnittsstrom ausgeben
  Serial_BT.print("A, ");
  Serial_BT.print(Momentanleistung);   // Momentanleistung ausgeben
  Serial_BT.print("W, ");
  Serial_BT.print(Leistung);           // Umgesetzte Leistung ausgeben 
  Serial_BT.println("Wh");
}

float Iaverage()   
// Der Strom wird ueber 10 Abtastwerte gemittelt und damit geglaettet
{
  if (DEBUG_Funktion) Serial_DB.println("Funktion: Iaverage");
  uint32_t i;
  float Isumme = 0;	// Summe der Stroeme
  float Ergebnis = 0;	// Ergebnis der Berechnung -> Iaverage
  // 10 Strommesswerte aufsummieren und Mittelwert berechnen
  for(i=0; i<10; i++) Isumme = Isumme + Strom[i];
  Ergebnis = Isumme / 10;
  return Ergebnis;
} 

float Temperaturmessung(uint8_t TempSensor,uint8_t TempSensorAnzahl, uint8_t TempAddress[3][8]) // Rueckgabewert celsius
// Temperaturmessung mittels OneWire Bussystem
{
  if (DEBUG_Funktion) Serial_DB.println("Funktion: Temperaturmessung");
  uint32_t present = 0;
  uint32_t data[12];
  float celsius;

  ds.reset();
  ds.select(TempAddress[TempSensor]);
  ds.write(0x44);        // start conversion
  present = ds.reset();
  ds.select(TempAddress[TempSensor]);    
  ds.write(0xBE);        // Read Scratchpad

  data[0] = ds.read();   // Temperatur abholen unteres uint32_t
  data[1] = ds.read();   // Temperatur abholen oberes uint32_t

  uint32_t raw = (data[1] << 8) | data[0];
  celsius = (float)raw / 16.0;

  return celsius;
}
