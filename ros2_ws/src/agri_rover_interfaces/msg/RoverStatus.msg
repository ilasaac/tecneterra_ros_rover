uint8 rover_id            # 1 = master, 2 = slave

bool armed
string mode               # mirrors RCInput.mode
string gps_fix_type       # NO_FIX | GPS | DGPS | RTK_FLT | RTK_FIX

float32 battery_voltage   # V
float32 battery_remaining # 0.0–1.0

string rc_link_status     # OK | LOST | TIMEOUT
string rf_link_status     # OK | LOST (inter-rover SX1278, master only)

builtin_interfaces/Time stamp
