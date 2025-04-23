# Terminator
A flight termination device for BPP.

## `T1100` - Current Revision

Much the same as T1000 but simpler and more complete.

## `T1000`
Shipman's ENEE499L project. The digital portion integrates a STM32L4, GPS, P/T Sensor, FRAM, and SX1262 Radio. This should provide quite a bit of flexibility in a small package while consuming minimal power. The digital board interfaces with a cutdown board which performs hot-wire termination like many other designs. However, this board is really just a barebones buck converter power stage that will be controlled by the digital board. This should provide increased flexiblity and efficiency over other implementations while also allowing T1000 to terminate even 550 paracord.
