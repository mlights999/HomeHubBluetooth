# HomeHub

Project created for Particle Gen 3 (Argon, Boron, Xenon) for controlling Kasa smart light bulbs with proximity detection using Bluetooth LE.

## Project Features

- Support for controlling TP-Link Kasa smart bulbs/plugs over a LAN
- Automatic control of TP-Link Kasa bulbs/plugs using Bluetooth proximity scanning
- Automatic-on at sunset with phone proximity
- Control TP-Link Kasa bulbs/plugs with a Siri shortcut (using webhooks)
- Quick control of brightness and color temperature using joystick
- Quick on/off setting of lights using integrated button

#### Background:  
Using Bluetooth Low Energy (BLE), Particle Gen 3 devices can scan for local BLE devices periodically and examine their advertising data. Apple's devices (specifically iPhones, Apple Watches, and iPads) periodically advertise as part of their "Continuity" platform. In examining the advertising data of my own devices (iPhone 12, Apple Watch Series 5), I was able to find a set of bytes in the advertising data that are static and unique to those specific devices. Using this, I can measure the signal strength of the BLE broadcast.

Automatic control of the lights is based on the presence of BLE peripherals and is configured to turn on/off lights based on when my devices (based on the static bytes) are present. The Particle device periodically scans and looks for Apple devices (identifiable by the manufacturer ID 21a0102), then checks if that devices static custom data bytes match the ones for my phone. You can find these bytes by putting the phone really close to the Particle device and see which bytes stay constant over time (usually there is scrambling of the other bytes every few minutes). Once you find those bytes, the Particle device can determine if it has scanned your phone. I've found that two bytes are constant, so there should be 2^16 unique Apple device keys. With this you can filter out devices you don't want affecting the control algorithm

#### Automatic Control Method:

- Scan for devices periodically that match the dictionary of "my devices" (based on the static bytes)
- If a known device is found and the lights are on, keep them on.
- If a known device was just found and the lights are off, check the ambient light sensor (ALS):
    - If a manual trigger set the lights off, then leave them off until the device leaves for 10 minutes and comes back
    - If the ALS is "dark", turn on the lights
    - If the ALS is "light", leave them off (saves electricity)
    - Update the timer for when a device was last discovered
- If the lights are on, check when a known device was last discovered:
    - If it's been 3 minutes and we aren't under manual control, turn off the lights
    - If we're under manual control, do nothing
    

