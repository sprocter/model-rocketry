# model-rocketry
This is a repository for model rocketry projects. Currently, there are two:

1. The _Nene_ "Flight Computer" that measures, computes, and logs a variety of properties like altitude and acceleration.
2. A collection of 3D models for "sleds" so you can securely mount sensors, computers, batteries, etc. in a variety of payloads.

## The Nene Flight Computer

If you've ever launched a model rocket, you've probably also wondered how high it went, how fast it traveled, and what sort of g-forces it experienced. There are some commercially-available altimeters and accelerometers, but I wanted to make my own. 

This is very much still a work-in-progress, and I do not recommend you use this code for... anything. I'll update this readme with instructions, part lists, and all the information necessary to build, use, and modify things when the code stabilizes enough that every change isn't breaking.

That said, if you want glance at the code, it's all open-source and there are already a number of neat features:

1. (Almost entirely) written in micropython
2. Different configurations for different rocket sizes -- from tiny 25mm x 75mm (BT-50) to larger 45mm x 128mm (BT-65) to 4" High-Power rockets.
3. Measurement of all sorts of data: acceleration, altitude, rotation, and magnetic heading using easily-available parts from various "maker" stores like [Adafruit](https://www.adafruit.com/) and [Sparkfun](https://www.sparkfun.com/).
4. Cross-checking of the various measurements against each other and previous data using [Kalman Filtering](https://en.wikipedia.org/wiki/Kalman_filter). This also lets us extract fairly reasonable values for properties we can't measure directly, like airspeed. 
5. The measurements are also integrated into a full [Attitude and Heading Reference System](https://en.wikipedia.org/wiki/Attitude_and_heading_reference_system).
6. Help finding your rocket with GPS and long-range radio.

## The Payload-Bay Sleds

The "sleds" are [OpenSCAD](https://openscad.org/) files that you can tweak as necessary and then print with a 3D printer. If you use an "Aero" filament, they should be pretty light, but still relatively sturdy. They provide a grid-type arrangement (patterned after [this product](https://www.adafruit.com/product/5774)) that lets you easily and securely mount breakout boards (or anything with mounting holes) using M2.5 nylon screws.