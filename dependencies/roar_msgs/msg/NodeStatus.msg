std_msgs/Header header

uint8 status
string message

uint8 STARTING = 0 # Node not running, Module launching
uint8 STARTED = 1
uint8 RUNNING = 2 # Node running
uint8 ERROR = 3 # Node or Module error
uint8 SHUTDOWN = 4 # Node not running, Module ready to launch
uint8 UNRESPONSIVE = 5