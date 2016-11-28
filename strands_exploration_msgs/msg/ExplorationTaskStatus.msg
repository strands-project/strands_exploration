uint8 EXECUTED = 0 #executed
uint8 DROPPED = 1 #dropped by the executor
uint8 NOT_ADDED = 2 #task wasn't added to the schedule due to lack of tokens
uint8 IGNORED = 3 #task was ignored, due to lack of tokens
uint8 result
strands_executive_msgs/Task task
int64 bid
uint8 execution_status #if executed, what was the outcome