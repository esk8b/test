// Version v41p0
/*
Beschaltung
-----------
Pin Vin -> 3.7 to 5.5V (Vom Akku mit BEC, der die AkkuSpannung auf maximal 5V runterteilt.)
Pin 3.3V-> 3.3V (Sollte ein BT-Modul verwendet werden, so muss 3.3V vom Akku mittels Spannungsversorgung bereitgestellt werden.
                 Diese 3.3V koennen dann den Tensy 3.1 und das BT-Modul zentral versorgen)
 
Pin A0  -> Spannungsmessung bis 3.3V (durch einen geeigneten Spannungsteiler kann zum Beispiel ein Messbereich bis 30V erreicht werden)
Pin A2  -> Stromsensor z.B. ACS756 SCA-050B (+- 50A Messbereich, Achtung den Stromsensor mit 3.3V versorgen)

Pin D0 RX1 -> Tx Bluetooth Modul
Pin D1 TX1 -> Rx Bluetooth Modul

Pin A4/D18 SDA0 -> Empfaenger Nunchuk SDA (Nunchuk Lib wird benoetigt.)
Pin A5/D19 SCL0 -> Empfaenger Nunchuk SCL
 
Pin D13 -> DS1820 Temperatur Bus (OneWire Lib wird benoetigt. DS1820 mit 3.3V Versorgungsspannung und Pull UP Widerstand)

Pin D2 -> Status LED fuer Pairing (470 Ohm gegen GND)
 
Pin A9 -> PWM Signal fuer ESC-A (Unbedingt an die GND-Verbindung denken!)
Pin A8 -> PWM Signal fuer ESC-B (Unbedingt an die GND-Verbindung denken!)
Pin A3 -> PWM Signal fuer Brems_Servo         

Pin D3  -> Hupe C-Taste  (MOSFET IRLU2905 vorschalten und 1K Ohm Widerstand zum Gate des MOSFETs)
Pin D11 -> Licht C-Taste (MOSFET IRLU2905 vorschalten und 1K Ohm Widerstand zum Gate des MOSFETs)
Pin D6  -> Bremslicht (MOSFET IRLU2905 vorschalten und 1K Ohm Widerstand zum Gate des MOSFETs)
Pin D4  -> Blinker_L (MOSFET IRLU2905 vorschalten und 1K Ohm Widerstand zum Gate des MOSFETs)
Pin D5  -> Blinker_R (MOSFET IRLU2905 vorschalten und 1K Ohm Widerstand zum Gate des MOSFETs)
 
Pin D12 -> PWM Ausgang zum Schalten des Leistungs-MOSFET fuer die Stromversorgung des Motorreglers (langsames Hochfahren)
           Wegen der 3.3V am Teensy und der Stroeme wird ein Optokoppler (CNY74-2) zum sicheren Schalten des MOSFETs (IRFP3206) empfolen
           
  
*/           

// Anschluesse Teensy definieren
// Messanschluesse
#define Pin_Strom A2                     // Pin A2 Strom. Wichtig fuer die ADC-Lib! durch die Nutzung von A0 und A2 können die beiden ADCs im Teensy getrennt angeprochen und paramertiert werden.
#define Pin_Ubatt A0                     // Pin A0 Spannnung
#define Pin_Temperaturanschluss 13       // Pin D13 Temperaturmessung DS1820 BUS. Die Messbausteine mit 3.3V und Masse versorgen. (Leiterplatte V2.5 macht das)
OneWire ds(Pin_Temperaturanschluss);     // den oben definierten Pin zuweisen
// Signalisierung
#define Pin_ReceiverLED 2                // Pin D2 Ausgang fuer den Status der Funkverbindung 
// Motorsteller und Servos
#define Pin_Motorstelleranschluss_A A9   // Pin A9 Motorsteller_A PWM-Anschluss
#define Pin_Motorstelleranschluss_B A8   // Pin A8 Motorsteller_B PWM-Anschluss
#define Pin_Brems_Servo A6               // Pin A3 Brems_Servo PWM-Anschluss
// MOSFET-Ausgaenge
#define Pin_Hupe  3                      // Pin D3 Ausgang fuer die Hupe
#define Pin_Licht 11                     // Pin D11 Ausgang fuer Licht an aus
#define Pin_Blinker_L 4                  // Pin D4 Ausgang fuer Blinker_L
#define Pin_Blinker_R 5                  // Pin D5 Ausgang fuer Blinker_R
#define Pin_Bremse 6                     // Pin D6 Ausgang fuer Bremslicht
// Anti-Spark Schaltung
#define Pin_PowerSwitch 12               // Pin D12 Ausgang PWM zum Einschalten des Leistungsteils mittels MOSFET oder BTS555

