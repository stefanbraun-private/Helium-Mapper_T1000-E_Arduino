
# Helium Mapper
*Experimental Helium IoT coverage mapping device,
coded in Arduino running on Seeed Studio T1000-E LoRaWAN GPS tracker.*

![Seeed Studio T1000-E LoRaWAN GPS tracker](img/SeeedStudio_T1000-E.png)

## Usage
This software turns a T1000-E into a custom OpenSource Helium mapping device.
It sends the current GPS location via Helium LoRaWAN to community projects like

- [mappers.helium.com](https://docs.helium.com/iot/coverage-mapping/)
- [helium.coveragemap.net](https://www.coveragemap.net/2024/02/28/mapping-with-chirpstack/)
- [mappers.hexato.io](https://mappers.hexato.io/docs)

## Remarks
The LoRaWAN uplink payload decoder works only on ChirpStack-based LNS (LoRaWAN Network Server),
and at the moment not all community projects can handle those uplink metadata...
Attention: There are different types of similar [Seeed Studio T1000 devices](https://wiki.seeedstudio.com/SenseCAP_T1000_tracker/Introduction/):
- T1000-A =>closed source GPS tracker for LoRaWAN, contains multiple sensors
- T1000-B =>same device as *-A* , just without sensors
- **T1000-E** =>similar to *-A* , but OpenSource for custom LoRa-Peer-to-Peer projects (like [Meshtastic](https://meshtastic.org/)) and custom LoRaWAN applications

## Disclaimer
This code is based on Seeed Studio's  [T1000-E Arduino example code](https://wiki.seeedstudio.com/t1000_e_arduino_examples/)
 and contains code snippets from many sources...
 
 It's an experiment for proof-of-concept,
 as a starting point for other developers :-)

## Implemented Functionality
The Arduino code shows joining LoRaWAN network with custom OTAA-keys,
extracting GPS location from GNSS chip,
sending payload encoded with CayenneLPP as LoRaWAN uplink,
a blinking LED as a test for minimal user interface,
handling of short/long/double button press on button,
and using buzzer for playing Nokia phone ringtone and Star Wars theme...
All these functions are executed quasi-parallel with ProtoThreads :sunglasses: 

## Libraries installed via Arduino IDE Library Manager
- Protothreads by Ben Artin (v1.4.0-arduino.beta1)
- TinyGPSPlus by Mikal Hart (v1.0.3)
- AceButton by Brian T. Park (v1.10.1)

