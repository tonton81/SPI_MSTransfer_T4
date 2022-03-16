# SPI_MSTransfer_T4

SPI Slave library designed for Teensy 4.x, but made backwards compatible with Teensy LC & Teensy 3.x.

Features:

1) Allows master to send payloads to slaves
2) Allows master to receive payloads from slaves
3) Allows multiple slaves:
  a) with separate chipselects,
  b) daisy-chained on same chipselect
4) Can control slave's gpios:
  a) digitalWrite
  b) digitalRead
  c) pinMode
  d) analogread
  e) analogwrite
  f) analogreadresolution
  g) analogwriteresolution
5) Slave detection function, reports slave IDs on the specified SPI port.
     Each slave has a unique ID, this allows daisy-chaining to work by shifting data over to the proper slave
