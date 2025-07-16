# Gasverbrauch messen und verarbeiten 

## Funktion

Manche Gaszähler geben pro vebrauchte 10 Liter einen magnetischen Impuls ab.
Dieser Impuls wird mit einem Hall Sensor in elektrische Impulse umgewandelt, ein RTC Baustein PCF8583 arbeitet als Zähler und speichert die Zahl der Impulse im internen RAM ab.

PCB Eagle Dateien (für Version 1.3) und KiCAD Dateien (für Version 1.4) sowie Stückliste (BOM) für beide Versionen anbei.

Die Daten werden mit Hilfe eines ESP8266 (z.B. Wemos D1 mini) über die I²C Schnittstelle ausgelesen und verarbeitet. 
Über das MQTT Protokoll werden die Daten an einen lokalen Broker weitergeleitet, Daten werden nicht ins Internet hochgeladen.
Eine 3 Volt Knopfzelle versorgt den Baustein mit Energie und erhält die Daten bei Stromausfall. In der Version 1.4 wird der Sensor durch den ESP8266 versorgt, die Knopfzelle bildet die Backup Batterie.


* Diese Daten werden im RTC RAM gespeichert

1. Zählimpulse
2. Zählerstand zu Beginn der Messung (Counts = 0)
3. Zählerstand zu Beginn der Abrechnungsperiode
4. monatlicher Verbrauch in m³
5. Datum und Uhrzeit des letzen Auslesens des RTC Bausteins

* Diese Daten werden über MQTT an den Broker gesendet

1. Uhrzeit 
2. Gesamtverbrauch in kWh
3. Verbrauch zwischen zwei Zugriffen auf den RTC Speicher in kWh
4. Anzahl der Impulse seit Reset
5. Gesamtverbrauch in m³
6. Zählerstand zum Begin der Messung (Counts = 0) in Liter
7. Zählerstand zum Begin der Abrechnungsperiode in Liter
8. nach Bedarf monatlicher Verbrauch
9. Status (OK wenn Zugriff auf den RTC Baustein möglich)

* Diese Daten/Kommandos können vom Broker an den ESP8266 gesendet werden

1. Veröffentliche den Verbrauch eine einzelnen Monats oder aller Monate von 1 to 12 
 * SYNTAX  : "Energy/Gas/command"
 * PAYLOAD : "Monat, x"
 * Wert für x: entwede 1 bis 12 für einen einzelnen Monat, oder x > 12 für alle 12 Monate
  
 2. Setze den Wert für den Verbrauch zu Beginn der Heizperiode 
 * SYNTAX  : "Energy/Gas/command"
 * PAYLOAD : "InitP, x"
 * Wert für x: Verbrauch in Litern

 3. Setze den initialen Verbrauchswert für den Counter Null 
 * SYNTAX  : "Energy/Gas/command"
 * PAYLOAD : "InitS, x"
 * Wert für x: Verbrauch in Litern

 4. Setze den initialen Wert für den Verbrauch eines einzelnen Monats  
 * SYNTAX  : "Energy/Gas/command"
 * PAYLOAD : "InitM, x ; y"
 * value for x: ausgewählter Monat
 * value for y: Verbrauch in Litern
 
 5. Setze den Wert für den Start der Abrechnungsperiode 
 * Initialisiert den RTC chip und setzt den initialen Verbrauchswert für count=0 der Abrechnungsperiode 
 * SYNTAX  : "Energy/Gas/command"
 * PAYLOAD : "InitC, x"
 * value for x: Verbrauch in Litern

## Vorbereitung

* in Gas_WiFi SSID und Passwort eintragen
* in Gas_MQTT IP Adresse des MQTT Broker eintragen
* ggfs. Thingspeak aktivieren und Channel und Key eintragen
* Topic für MQTT an die eigenen Wünsche anpassen
  
