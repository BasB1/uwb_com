# Communicate with UWB

This node will translate specified topics to serialized JSON strings and compresses the strings using Zlib under 100 characters. This is important due to the limited registery size of the Pozyx devices.

## Dependencies
Zlib
```
pip install zlib
```

Pozyx
```
pip install pypozyx
```

## How to use
Plug one Pozyx device into your PC. This is the node that will send and receive odom information with another PC dat also has a Pozyx device pluged in. In the launch file you can specify what topic should be send over the UWB and to which destination. The destination is given by Pozyx in HEX but should be given in decimal in the launch file.
