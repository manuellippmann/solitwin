# SY Solitwin - Codebase

_This is the codebase for my autonomous sailboat project. Find out more: http://www.lippe-mann.de/solitwin_

#### What is this about?

I'm transforming a radio controlled model sailboat into an autonomous sailing vessel. In the end it will be able to navigate to a user defined GPS coordinate on its own. Hopefully :)

![solitwin sailing](https://i.imgur.com/Q7F9jBA.gif)

## Technical information

My project is based on the the [Arduino Framework](https://www.arduino.cc).

To make things easier, I'm using the [PlatformIO](https://platformio.org) on [VSCode](https://code.visualstudio.com) to manage and build my code. Therefore, you can find my [main project file in /src](https://github.com/mlippe/solitwin/blob/master/src/main.cpp).

## Equipment

- Head Unit: [Arduino Yun](https://store.arduino.cc/arduino-yun)
- Compass + Accelorometer: [Adafruit LSM303](https://www.adafruit.com/product/1120)
- Magsensor (Windvane): [Adafruit MLX90393](https://www.adafruit.com/product/4022)
- GPS Unit: [uBlox NEO-6M on a Breakout](https://lastminuteengineers.com/neo6m-gps-arduino-tutorial/)
- GPS Antenna : [Eightwood External Active GPS Antenna](https://www.amazon.de/gp/product/B07YW3BKDW/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)
- Battery (Arduino): [Anker Powercore 10000](https://www.anker.com/de/products/variant/powercore-10000/A1263011)
- Battery (Servos & Sensors): [BAKTH 7.2V 4500mAh NiMH](https://www.amazon.de/gp/product/B0797PF744/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1)
- RC Receiver: [V710 2.4G 7CH Receiver](https://www.banggood.com/V710-2_4G-7CH-Receiver-for-Spektrum-Storm-G152-DSMX-DSM2-RC-Drone-FPV-Racing-Multi-Rotor-p-1432827.html?rmmds=buy&cur_warehouse=CN)
- Rudder Servo: Modelcraft ES-035
- Sail Winch: [Hitec Servo HS-785HB](https://www.multiplex-rc.de/produkte/112785-servo-hs-785hb)
- Remote Control: [Spektrum DX6e](https://www.horizonhobby.de/de/-spmr6650eu--3)
