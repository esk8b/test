// Version v4p9

// Sonderfunktionen
#define UIMESS     TRUE         // HW UI-Messung vorhanden (TRUE,FALSE)
#define LICHTHUPE  TRUE         // HW Licht, Hupe vorhanden (TRUE,FALSE)
#define TEMPMESS   TRUE         // HW Temperaturmessung vorhanden (TRUE,FALSE)
#define LESOFT     TRUE         // HW Leistungselektronik langsam hochfahren vorhanden (TRUE,FALSE)

// Debugflag
#define DEBUG              FALSE // Aktivieren der Debugausgabe (TRUE,FALSE)
#define DEBUG_Funktion     FALSE // Aktivieren der Debugausgabe Welche Funktionen werden aufgerufen (TRUE,FALSE)
#define DEBUG_ISR          FALSE // Aktivieren der Debugausgabe Aufruf der ISR Interrupt Service Routinen (TRUE,FALSE)
#define DEBUG_Nunchuk      FALSE // Aktivieren der Debugausgabe Nunchuk Werte x,y-Achse, Beschleunigung, c,z-Button(TRUE,FALSE)
#define DEBUG_Messung      FALSE // Aktivieren der Debugausgabe interne Messwerte und Berechnungen (TRUE,FALSE)
#define DEBUG_Motorsteller FALSE // Aktivieren der Debugausgabe interne Werte Motorsteller (TRUE,FALSE)
#define DEBUG_DriveMode    FALSE  // Aktivieren der Debugausgabe interne Werte Motorsteller (TRUE,FALSE)


// Welche Schnittstellen sollen verwendet werden
// Serial  -> USB-Anschluss Teensy 3.1
// Serial1 -> Bluetooth Schnittstelle, auf die Baudrate in Elektroskate_Custom_Parameters achten!
#define Serial_BT Serial1        // Sollen die Werte fuer die BT-Schnittstelle auf USB oder Serial fuer das Debugging ausgegeben werden?
#define Serial_DB Serial        // Ausgabe der Debug Werte auf USB oder Serial ausgegeben werden?
