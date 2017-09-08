Header header
string machineName
uint16 tid              # used currently (machineID) -> will be replaced with 'machineName'!
uint16 taskDeadline     # changed
uint16 taskType         # changed

uint8 taskStatus
uint8 OPEN=0
uint8 WORKING=1
uint8 FINISHED=2
uint8 ERROR=3
uint8 CANCELLED=4

uint8 materialType
uint8 typeA=1
uint8 typeB=2
