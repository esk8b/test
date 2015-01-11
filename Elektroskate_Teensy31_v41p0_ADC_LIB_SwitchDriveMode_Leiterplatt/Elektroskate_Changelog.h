Funktionen
----------
 1. Z-Taste + Nunchuk Y vorn            -> beschleunigen (Integrationssteuerung/relativ oder DirectDrive/absolut)
 2. Z-Taste + Nunchuk Y hinten          -> entschleunigen mittelbar (Integrationssteuerung/relativ oder DirectDrive/absolut)
 3. Z-Taste + Nunchuk Y mitte           -> Geschwindigkeitsvorgabe halten (Integrationssteuerung), Motorfreilauf (DirectDrive)
 4. Z-Taste losgelassen                 -> Motorfreilauf (Integrationssteuerung/relativ oder DirectDrive/absolut)
 5. Z-Taste losgelassen + Hebel hinten  -> bremsen (unmittelbar Integrationssteuerung und DirectDrive)
 6. C-Taste + Nunchuk X neutral         -> Hupe (getastet)
 7. C-Taste + Nunchuk X links           -> Licht (geschaltet)
 8. 3 x Z-Taste + Nunchuk X links       -> Geschwindingkeitsumschaltung Hase / Igel
 8. Funkabriss                          -> Motor Freilauf
 9. Temperaturmessung
10. Messung von Strom und Akkuspannung mit Bestimmung der umgesetzten Momentanleistung und Gesamtleistung

Revision
 --------
 V1.0 02.12.2012 Barney:
 V1.1 08.02.2013 Barney: Subroutinen eingefuehrt und Bluetooth Schnittstelle vorbereiten
 V1.2 17.02.2013 Barney: Servo Lib durch PWM-Hardwareroutine ersetzt 20% Codeeinsparung
                         und Umstellung von int auf byte weitere 10% Codeeinsparung
 V2.0 18.02.2013 Barney: ISR-Routine fuer festes Zeitraster. Bluetooth Ausgabe eingebaut fuer Ubatt, Strom, Leistung und Temperatur. DS1820 Temperatursensor integriert
 V2.1 27.02.2013 Barney: Wii Lib optimiert heisst jetzt Nunchuk Lib alle Typen optimiert (unsigned oder byte) 
                         Ueberpruefung des Nunchukverbindung ueber Beschleunigungssensor Fehlwerte, Umstellung der ISR-Routine auf 70Hz, damit kann die Sekunde/ Zentelsekunde nahezu erreicht werden.
 V2.2 02.03.2013 Barney: Nunchuk Einschaltproblem und Funkabriss geloest
 V2.3 10.03.2013 Barney: Licht und Hupe getrennt, Fehler in Dokumentation entfernt (SDA, SCL vertauscht)
 V2.4 17.05.2013 Barney: Multiplikator fuer Ubatt genauer, 3 Sekunden fuer Funkabriss. 
 V2.5 20.05.2013 Barney: Strom wird durch Mittelwert ermittelt und angezeigt (die Leistungsberechnung erfolgt 10 mal/s. BeschleunigungsBeschleunigungsDaempfungsfaktor erhoeht fuer angenehmeres Beschleunigen.
 V2.6 10.06.2013 Barney: Der Pin 12 wird zum einschalten des Leistungsteil definiert. Spaeter soll dieser ueber eine PWM den Leistungsteil sanft zuschalten
                         Es wird der Strom dynamisch begrenzt. Die Werte Ishort und Imax definieren die Grenzen fuer eine dynamische Strombegrenzung. 
 V2.7 23.06.2013 Barney: PWM funktioniert, Ueberstrom hatte einen Fehler und sollte jetzt funktionieren
 V2.8 03.09.2013 Barney: Direkte Steuerung der Motorwerte durch Nunchuk, Schreibfehler entfernen
 V2.9 04.09.2013 Barney: Hase- Igel Funktion, Korrektur Direkte Steuerung der Motorwerte durch Nunchuk
 V3.0 06.09.2013 Barney: Hase- Igel Funktion fertig, per define DirectDrive TRUE/FALSE ist Direct Drive oder Integrationssteuerung umschaltbar. Beschreibungstexte erweitert oder korregiert. 
 V3.1 06.10.2013 Barney: Start im Igelmodus

 !!! Teensy 3.1 Umstellung !!!
 V3.2 13.04.2014 Barney: Umstellung auf Teensy 3.1, sowie #define fuer Berechnung und Konstanten. Umstellen der Aufloesung ADC / DAC und TimerISR-Routinen
 V3.3 11.05.2014 Barney: Es werden drei ISR verwendet mit 1Hz, 10Hz und 100Hz Takt. Constanten auf define umgestellt, Anpassung der PWM an Teensy Aufloesung, uint16_t als Vorgabe fuer 16Bit Werte
                         Neue Nunchuk Lib angepasst auf Wire und Wireless Nunchuk, alles noch ungetestet!
 V3.3P4 11.05.2014 Dude: zentrales Debug-Flag
                         HW-Konfigurationen ueber Preprozessor Steuerungen aktivierbar 
 		                 Hase_Igel von Licht_Hupe getrennt
		                 nunchuk.init in Nunchuk_auslesen aufgenommen (wird daher immer beim auslesen ausgefuehrt)
		                 Aufloesung Nunchuck analogY in 2 Bereiche (Minimalwert y-Achse / Neutralstellung / Maximalwert y-Achse ganz oben) 
		                 geteilt -> Motorstellwert Leerlauf bleibt damit beim Druecken der Z-Taste erhalten;
		                 Kommentare mit ### im Header enthalten HW-spezifische Werte, die vom Nutzer angepasst werden sollten
 3.3P5 03.06.2014 Barney: Cleanup der Kommentare, Berechnung von Strom, Spannung und Leistung funktioniert jetzt mit dem Teensy richtig.
 3.4P1 05.06.2014 Barney: Fuer die vereinfachte Konfiguration sind jetzt Custom_Settings und Parameters in eine eigene Datei ausgelagert worden.
                          Überstromroutine überprüft.
 3.4P2 08.06.2014 Barney: Steuerung funktioniert jetzt wieder. Unnoetige nunchuk.init wurden entfernt. Es werden durch die Nunchuk abfrage nur 20 Motorstellwerte/s erzeugt. Spaetere Version mit Teensy optimierter Lib ist angestrebt.
                          Messungen sind O.K.
                          Strombegrenzung nicht optimal, wird noch nachgearbeitet. Es fehlte ein if () wenn gembremst wird sollte die Strombegrenzung nicht die Bremse abschalten. Beim fahren klappt die Strombegrenzung nicht so optimal.
 3.4P3 09.06.2014 Barney: Alle #Define mit Typendefinition versehen. Die Debugzwecke kann die Ausgabe der Messwerte von Serial1 auf Serial (USB) umgeleitet werden (Custom Settings)
                          Bremsen auch mit Ueberstrom ist jetzt moeglich.
 3.4P4 11.06.2014 Barney: Fehler in Auswertung der Y-Achse und der Bremswertberechnung beseitigt. Wenn der gemessene Wert der Y-Achse unter MinYAchse lag, wurde ein negativer Ueberlauf errechnet, welches die direkte Bremsfunktion abgeschaltet hat.
                          Wenn jetzt analogY < MinYAchse ist wird analogY = MinYAchse. Vorsichtshalber wurde dies mit MaxYAchse auch begrenzt. Dies faellt nur bei der Verwendung mehrerer Nunchuks auf.
                          Die Berechnung des Motorstellwertes wird jetzt durch den Timer3 zeitlich festgelegt. Durch das Weglassen einer unnoetigen Nunchuk Abfrage und Umstellung der (wieder) Verbindung zum Nunchuk im Sekundentakt, ist ein 
                          zeitliches Aequidistantes Steuern den Motors moeglich.
 3.4P6 19.07.2014 Dude:   Umstellung des Wertebereiches der X- und Y-Achse auf 12 bit Zahlenraum von 0 bis 4095. Damit kompatibel zum Einsatz einer anderen HW als Nunchuk zur Sollwertvorgabe.
                          Beschleunigungs- und Bremsvorgabe im DirectDrive- und im Tempomat-Modus über eine Kennlinie (Potenzfunktion mit frei wählbarem Exponenten für beide Bereiche)
                          Neutralzone der Y-Achse konsistenz zum Tempomat jetzt auch im DirectDrive implementiert
                          Konsolidierung der Stellwertberechnung für den Motor, DirectDrive und Tempomateinstellung in einer Berechnungsvorschrift integriert (vermeidet Fehler)
                          Abwurfgefahr durch versehentliches Loslassen des Z-Knopfes beim Verringern der Geschwindigkeit, d.h. Sprungfunktion ins Bremsen, wird detektiert und abgefangen					  
 3.4P7 23.07.2014 Barney: Schnittstellen fuer Debug- und Datenausgabe getrennt. Beide können über Bluetooth, USB oder getrennte Schnitstellen fuer die Ausgabe nutzen.
                          Anschluesse werden jetzt in Elektroskate_Connection festgelegt. Anpassung auf die Leiterplatte Elektroskate_Teensy V1.0 wurde durchgefuert.
 3.4P8 26.07.2014 Barney: Die Verbinnden Routine hat die ISR blockiert und damit die Messungen ausser Kraft gesetzt. Durch die Integration von verbinden() in die Loop Schhleife, koennen die drei ISR wieder richtig arbeiten
                          Zum Testen wurde die Temperaturmessung() so umgeschrieben, dass eine richtige Parameteruebergabe durchgefuerht wird. Damit ist ein Erkennen der benoetigten Parameter besser moeglich. Siehe Temperaturmessung(TempSensor, defTempSensorAnzahl,defTempAddress)
                          Erste Ansaetze alle Werteausgaben zentral in die TimerTx ISR einzubetten. Eine Ausgabe der Werte an die Serielle Schnittstelle dezentral ist schlecht wartbar. Es soll an zentraler Stelle die Ausgabe erfolgen. Dies ist die Vorbereitung fuer
                          die eigenen BT-Fernbedienung.
                          Die Dateien Elektroskate_x.h wurden mit einer Versionsnummer versehen, um Anpassungen der Inhalte erkennen zu koennen. D.h. Wenn neue Parameter definiert werden oder Bezeichungen sich veraendern, muss die Version erkennbar sein.
 3.4P9 14.09.2014 Barney: UREF mit 3.3V gelöscht, da diese Variable nicht genutzt wird.
 3.4P9-15.09.2014 Barney: ADC-Lib zugefuegt. Es besteht damit die Moeglichkeit den ADC viel besser zu steuern. Die Spannungs- und Strommessung wurde damit deutlich verbessert.
 4.0p9-01.11.2014 Barney: ES kann waehrend der Fahrt zwischen DirectDrive und IntegralDrive umgeschaltet werden. Die Config-Dateien wurden angepasst!
 Ankündigung: Dwe ADC fuer die Strommessung zieht einen Pin weiter. Damit kann dann im Hintergrung gemessen werden. Die neue Leiterplatte wird entsprechend angepasst! Es kommt auch ein/zwei Anschluesse fuer Bremsservos und ein Bremslicht dazu.
 
 4.1p0-28.12.2014 Barney: Die Leiterplattenversion v2.5 hat jetzt den Stromsensor an A2. Dadurch kann die ADC-Lib die beiden ADCs getrennt im Teensy parametrieren.
                          Durch die neue Leiterplatte wird der Temperatursensor und die beiden HV-Anschlüsse mit 3.3V versorgt.
						  Ich habe ganz viel verändert jaja
                  
