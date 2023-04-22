std_msgs/Header header

uint8 status
string message

uint8 OFFLINE = 0 # Module ready to launch node
uint8 ONLINE = 1 # Module launched node (node running)
uint8 UNRESPONSIVE = 2 # Node unresponsive (no heartbeat)
uint8 ERROR = 3 # Module Error