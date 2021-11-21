# greenhouse
## Gewächshaussteuerung

## ToDo: **Schaltgrößen für Frischluftzufuhr herausarbeiten**

Es stehen zwei Werte zur Verfügung O2 Konzentration und CO2 Konzentration.
Erster Ansatz 1,5% bis 2% Verminderung der O2 Konzentration führt zum Einschalten der Abluft
bis bei 0,5% Erhöhung (oder mehr?) =Hysterese der Abtransport warmer verbrauchter Luft wieder beendet wird.
Es muss sich schrittweise an eine unregelmäßige Verbrennung gar Verlöschen der Flamme herangetastet werden!
Dies von 21% O2 Standardwert zu 19% O2 absolut.

Alternativ sollte auf die CO2 Konzentration geachtet werden. Bereits 2% CO2 wären spürbar einschränkend, 5% lethal!
Um den Ausfall eines Sensors kompensieren zu können wäre doppelte Kriterien Oder verknüpft anzustreben!

**Weitere Parametrierung:**
- 12V Lüfter werden ab etwa 30 Grad zugeschaltet
- Dauer eines Messzyklus, derzeit auf 10 Sekunden voreingestellt. Dies kann nicht wesentlich unterschritten werden!

**Software:**
- Fensterausgabe formattieren, z.B wichtige Werte hervorheben, Fehlermeldungen in Rot
- die Degradation des O2 Sensors muss per Software nachgereglt werden (Anm.: Angabe des Herstellers einer Haltbarkeit von 2 Jahren)

**Hardwareergänzungen:**
- ein 12V permant Lüfter mit Ein/Aus Schalter, Dient der Umwälzung um Kältelöcher zu minimieren...
- weitere 1-Wire Temperatursensoren
- externer Bodenfeuchtesensor I2C
- 1 weiterer I2C Port wäre noch frei
- 2 A/D Eingänge sind noch frei
- LoRaRaspberry Hut kann man in der warmen Jahreszeit austesten da durchaus denkbar ist dass Schnee, Regen etc. die WLAN Kommunikation unterbrechen
- kleine bis mittlere Pi Kamera anschließen, ggf. mit Blick auf den Umschalter der Zweiflaschengasanlage (Wenn Rot dann Gasflasche tauschen!)
