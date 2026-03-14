# Raw PPM channel values from RP2040 (microseconds, 1000–2000)
# Index: 0=throttle, 1=steering, 2=aux, 3=aux, 4=SWA(emergency), 5=SWB(auto), 6-15=aux
uint16[16] channels

string mode          # MANUAL | AUTONOMOUS | RELAY | EMERGENCY | AUTO-NO-HB | AUTO-TIMEOUT
bool sbus_ok         # HM30 SBUS signal healthy
bool rf_link_ok      # nRF24/SX1278 link to slave healthy (master only)

builtin_interfaces/Time stamp
