RFM69 OOK Raw Data Receiver
---------------------------

Designed and coded by Felix Baessler, felix.baessler@gmail.com

The project addresses the problem of: <br/>
&nbsp; &nbsp; &nbsp;    **->** &nbsp; Interoperability / Interconnectivity, Compatibility, Legacy &nbsp; **<-**  <br/>
in smart home automation and IoT applications.

### Features:
- **Universal Receiver** <br/>
  of data sent by a priori unknown types of devices <br/>
  -> no assumptions about: sync, preamble, delimiter, packet, protocol
- **Robust Reception** <br/>
  featuring error correction of spikes, drops and outliers
- **Long Range / High Sensitivity** <br/>
  with low reception threshold of ≈18 dBm

### LICENSE
This project is released under [CC-BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/).<br/>
The licensing TLDR is: You are free to use, copy, distribute and transmit this Software for personal, non-commercial purposes, as long as you give attribution and share any modifications under the same license. Commercial or for-profit use requires a license. <br/>
For more details see the [LICENSE](https://github.com/ookraw/OOK-Raw-Data-Receiver/blob/master/LICENSE)

### Context
- **ISM** are the preferred radio frequency bands (434 / 868 / 912 MHz) used in smart home automation for remote-control and sensor data acquisition over the air
- **OOK**, On-Off Keying, is the modulation technique most widely found in low cost equipment. Information is transmitted by varying the duration of alternating HIGH- and LOW-signals. In general, these durations are restricted to a limited number of duration levels / categories (-> clusters)
- **Raw Data**, in form of signal duration sequences, is the common base level protocol of any OOK sender / receiver

### Setup Guide
This repository contains all that is needed to setup your Arduino workspace:
- receiver.ino
- categorizer.cpp
- categorizer.h
- categorizer_lib.cpp
- recorder.cpp
- radio_lib.cpp
- radio_lib.h
- RFM69_lib.cpp
- RFM69_registers.h

### Required Hardware
- MCU:   Arduino compatible mini pro 3.3V (MEGA328P)
- Radio: Hope RFM69w
- recommended: USB extension cable (5m) with snap-on Ferrite cores 

The Radio - MCU connections are defined in radio_lib.cpp <br/>
The prototype used during the development of the project can be found on  https://sites.google.com/site/rfm69arduino

### Project Presentation
An introduction to the project is available on https://sites.google.com/view/ookraw
