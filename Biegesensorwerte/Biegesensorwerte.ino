int a0=0; // Benennung als Variable für den Biegesensor
int a1=0; // Benennung als Variable für den Biegesensor
int a2=0; // Benennung als Variable für den Biegesensor
int a3=0; // Benennung als Variable für den Biegesensor
int a6=0; // Benennung als Variable für den Biegesensor

void setup() 
{
Serial.begin (9600); // Start der seriellen Verbindung für den serial monitor.
}

void loop() 
{
a0=analogRead(A0); 
a1=analogRead(A1);
a2=analogRead(A2);
a3=analogRead(A3); 
a6=analogRead(A6); 
Serial.print ("a0: ");
Serial.print (a0); // Ausgabe des Sensorwertes der x-Achse an den serial monitor
Serial.print (" a1: "); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
Serial.print (a1); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
Serial.print (" a2: "); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
Serial.print (a2); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
Serial.print (" a3: "); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
Serial.print (a3); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
Serial.print (" a6: "); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
Serial.print (a6); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
Serial.println ("    "); // Ausgabe von vier Leerzeichen
delay(100); // Wartezeit zwischen den einzelnen Ausgaben der Sensorwerte
}
