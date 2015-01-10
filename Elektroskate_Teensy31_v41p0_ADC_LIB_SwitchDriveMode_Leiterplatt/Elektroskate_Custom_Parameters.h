// Version v41p0
// Serielle Geschwindigkeit
#define BT_BAUDRATE    (uint32_t) 230400      // Datenrate fuer das Bluetooth Modul, ggf. fuer ein anderes BT-Modul anpassen, Ausgabe ueber Serielle Schnittstelle 1
#define DEBUG_BAUDRATE (uint32_t) 230400      // Datenrate fuer das Debugging in der Arduino IDE per USB

// Wertebereich von (Min..Max,SinnvollUntererWert..SinnvollObenrerWert)

// Nunchuk - Steuerparameter Sollwertvorgabe und Programmablaufsteuerung 
#define YAchseMin (uint16_t)    27 // MinimalWert y-Achse ganz unten (0..65535,0..30)
#define YAchseMax (uint16_t)   250 // MaximalWert y-Achse ganz oben (0..65535, 240..255)
#define YAchseNeutral          140 // Neutralstellung Y-Achse (0..65535, 125..140)
#define YAchseUT               124 // Wert der Nunchuk Y-Achse unterhalb dessen die Beschleunigung zurueckgenommen wird(Integrationssteuerung) (0..65535, 115..124)
#define YAchseOT               145 // Wert der Nunchuk Y-Achse oberhalb dessen beschleunigt wird(Integrationssteuerung) (0..65535, 141..145)

// Plausibilitätscheck und ggf. Korrektur
#if YAchseUT > YAchseNeutral
  #define (uint16_t) YAchseUT (uint16_t) YAchseNeutral
#endif
#if YAchseOT < YAchseNeutral
  #define (uint16_t) YAchseOT (uint16_t) YAchseNeutral
#endif

// Exponentialfunktion
#define YAchseExpNeg (float)              1.0 // Beschleunigungsfunktion 1 linear, 2 quadratisch (0.1..3.0, 1.0..2.0)
#define YAchseExpPos (float)              1.0 // Bremsfunktion 1 linear, 2 quadratisch (0.1..3.0, 1.0..2.0)

#define XAchseMin (uint16_t)                0 // MinmalWert X-Achse ganz unten (0..65535,0..30)
#define XAchseMax (uint16_t)              255 // MinmalWert X-Achse ganz unten (0..65535,0..30)
#define XAchseLinks (uint16_t)             30 // X-Achse wird fuer kleinere Werte als Linksstellung angenommen
#define XAchseRechts (uint16_t)           220 // X-Achse wird fuer groessere Werte als Rechtsstellung angenommen

#define PairingLostTrigger (uint16_t)      50 // Schwellwert fuer Funkabriss, wird dieser Wert ueberschritten liegt ein Funkabriss vor (0..65535, 20..70)
#define PairingSuccessTrigger (uint16_t)    5 // Schwellwert fuer erfolgreiche Verbindung (0..65535, 5..10)

// Temperatursensoren
#define defTempSensorAnzahl (uint8_t) 1       // Die Temperatursensoranzahl hier eintragen
uint8_t defTempAddress[3][8] = {{ 0x28, 0x00, 0xA5, 0xC6, 0x04, 0x00, 0x00, 0x51 }     // Seriennummer des Testsensors
                               ,{ 0x28, 0x75, 0x3F, 0x2A, 0x04, 0x00, 0x00, 0x73 }     // Seriennummer der Temperatursensoren, diese muss extra ermittelt werden
                               ,{ 0x28, 0x0D, 0xE2, 0x00, 0x04, 0x00, 0x00, 0x93 }};   // Seriennummer der Temperatursensoren, diese muss extra ermittelt werden

// PWM Motorsteuerung
#define PWM_ObererStellWertMotorIgel 1.6 // Wert berechnen fuer Oebere Pulsweite Igel (ms Pulsweite) (0.0..2.5, 1.6..1.8)
#define PWM_ObererStellWertMotorHase 2.0 // Wert berechnen fuer Oebere Pulsweite Hase (ms Pulsweite) (0.0..2.5, 1.9..2.1)
#define LeerlaufOffset (uint16_t)      0 // Offset zum schnelleren Ansprechen des ESC bei inkrementeller Steuerung (0..65535, 0..300)
#define PWM_LeerlaufStellWertMotor   1.5 // Wert berechnen fuer Leerlauf (ms Pulsweite)  (0.0..5.0, 1.4..1.6)
#define PWM_UntererStellWertMotor    1.0 // Wert berechnen fuer untersten Wert (ms Pulsweite)  (0.0..5.0, 9.9..1.1)

// PWM Brems-Servo
#define PWM_Bremse_Ungebremst   1.0 // Wert berechnen fuer Leerlauf (ms Pulsweite)  (0.0..2.5, 1.0..1.5)
#define PWM_Bremse_Vollbremse   2.0 // Wert berechnen fuer Leerlauf (ms Pulsweite)  (0.0..2.5, 1.8..2.5)
#define PWM_Bremse_Exp (float)  1.0 // Bremsweg-Servo 1 linear, 2 quadratisch  (0.1..3.0, 1.0..2.0)

// Fahrmodus
// Wie soll gesteuert werden
uint32_t DirectDrive            = FALSE; // TRUE -> Direct Drive, FALSE -> Integrations Drive. Das Startverhalten nicht aendern!

// Incrementel Drive Parameter
#define BeschleunigungsDaempfung (uint32_t)  112 // Je groesser die BeschleunigungsDaempfung, desto langsamer wird inkrementell beschleunigt (ggf. anpassen)   (0..32^2, 0..250)
#define BremsDaempfung           (uint32_t)  80  // Je groesser die BremsDaempfung, desto langsamer wird die Bremskraft inkrementell aufgebaut (ggf. anpassen)   (0..32^2, 0..250)

// Spannungs/- und Strommessung
#define BattSpgMMax (float) 32.6 // Maximale messbare Batterienspannung 33V (10k Ohm / 2 x 180k Ohm), oder 40V (10k Ohm / 121k Ohm) 
#define StromMMax   (float) 50.0 // Maximal messbarer Strom in A (ACS756SCA-050B Stromsensor)

// Strombegrenzung und Kurzschlusserkennung
#define Ishort (uint16_t)     50   // Maximaler Strom, dieser fuehrt zur sofortigen Abschaltung (<= StromMMax)
#define Imax   (uint16_t)     40   // Grenzwert, ab dem die Strombegrenzung einsetzt
#define Strom0A (float)       50.3 // 0A Messwert, wird angezeigt wenn kein Strom fliesst und Strom0A = 0 (muss kalibriert werden)

