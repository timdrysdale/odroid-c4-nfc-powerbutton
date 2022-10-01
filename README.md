# odroid-c4-nfc-powerbutton
Arduino code and linux bash scripts for using a near-field-comms tag as a power-off button for an odroid C4

## Background

We need a physical power-off button for the odroids, so we can shut them down safely regardless of whether they have an active admin connection. e.g. during run-up of a new experiment when something is not right.

We've noticed odroids with eMMC are sufficiently susceptible to storage corruption that taking this precaution is necessary.

Using an NFC tag carrying a secret code as the trigger for a safe shutdown provides convenience and security.

Some alternatives we did not choose:

 - an external physical button accessible from outside the box could be used by anyone 
 - a physical button inside the box could be unaccessible when the box is stacked with corner covers in place
 - a magnetic switch could be triggered by anyone 
 
## Single Board Computer 

The shutdown script checks a logic value on GPIO.1 (pin #18 of the header), which is normally pulled up to logic 1. If this is set to logic 0 for more than one second, then a shutdown is immediately initiated, and GPIO.2 is set high (pin #22 of the header) to provide an optional visible indication via LED that the shutdown has started. The LED will remain on once the shutdown has completed, so shutdown can be verified by checking the activity lights on the odroid PCB and/or its network connection.

The shutdown script is run as a systemd service so that it is automatically started at boot.

An ansible task is provided to document / facilitate the set up process. Essentially

- copy the `powerbutton` script to `/usr/local/bin` and set permission to allow execution
- copy the `powerbutton.service` to `/etc/systemd/system`, then enable, and start it

See files in `./sbc`.

## Firmware

The firmware runs on a nano IOT33. The code in the tag should be programmed into each arduino via serial messaging, so that the tag code is never stored in the source code repo (to prevent inadvertent leakage).

The prototype was developed with an RFID-RC522 board from [AZ Delivery](https://www.amazon.co.uk/gp/product/B074S8MRQ7).

THe documentation suggests this library
https://github.com/miguelbalboa/rfid

The wiring table is

|--------|--------------------------------|
| Az-Delivery RF522 | Arduino Nano Iot 33 |
|--------|-----|
| 3.3V   | 3.3V|     
| RST    | D9  |
| GND    | GND |
| IRQ    | D2  |
| MISO   | D12 |
| MOSI   | D11 |
| SCK    | D13 |
| SDA(SS)|  D10|
|--------|------|

See files in `./fw`


