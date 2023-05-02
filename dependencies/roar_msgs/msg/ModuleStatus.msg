string name

std_msgs/Time stamp
uint8 status
string message # Optional

# States (Constants)
uint8 OFFLINE = 0 # Module is shutdown, ready to launch
uint8 ONLINE = 1 # Module is online and running
uint8 UNRESPONSIVE = 2 # Module not publishing heartbeat