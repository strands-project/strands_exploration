import rospy

from strands_executive_msgs.srv import AddTask
from strands_executive_msgs.msg import TaskEvent

def msg_event_to_string(msg_event):
    if msg_event == TaskEvent.TASK_FAILED:
        return "TASK_FAILED"
    if msg_event == TaskEvent.TASK_SUCCEEDED:
        return "TASK_SUCCEEDED"
    if msg_event == TaskEvent.TASK_PREEMPTED:
        return "TASK_PREEMPTED"
    return str(msg_event)
    

class ExplorationBidder(object):    

    def __init__(self):
        
        self.add_task = rospy.ServiceProxy('/task_executor/add_task', AddTask) 
        
        self.task_event_sub = rospy.Subscriber('/task_executor/events', TaskEvent, self.process_task_event, queue_size = None)
        
        REAL
        self.period = rospy.Duration(rospy.get_param('~period', 60*60*24))
        self.tokens_per_period = rospy.get_param('/exploration_bidding/tokens_per_period', float(60*60*12)/3)
        
        ##TESTING
        #self.period = rospy.Duration(rospy.get_param('~period', 60))
        #self.tokens_per_period = rospy.get_param('/exploration_bidding/tokens_per_period', 20)
        
        
        self.available_tokens = self.tokens_per_period
        self.currently_bid_tokens = 0 #total number of tokens that this node has currently in play
        self.currently_added_tokens = 0 #total number of tokens that this node has added to the executor (can never be more than the available tokens)
        self.added_tasks = {}
        self.queued_tasks = []
        
        self.timer=rospy.Timer(self.period, self.update_budget)
        
    def update_budget(self, timer_event):
        self.available_tokens+=self.tokens_per_period
        self.process_task_queue()
    
    def add_task_bid(self, task, bid):
        if isinstance(bid, float):
            rospy.logwarn("Float bids are not allowed. Ignoring." + str(task))
            return False
        if bid <= 0:
            rospy.logwarn("Zero or negative bids are not allowed. Ignoring." + str(task))
        if self.available_tokens - bid < 0:
            rospy.logwarn("Not enough tokens available to bid. Ignoring." + str(task))
            return False
        else:
            task.priority = bid
            self.queued_tasks.append(task)
            self.currently_bid_tokens+=bid
            self.process_task_queue()
            return True
    
    def process_task_queue(self):
        i = 0
        rospy.loginfo("Trying to add tasks to the schedule...")
        while i < len(self.queued_tasks):
            task = self.queued_tasks[i]
            if rospy.get_rostime() > task.end_before:
                rospy.logwarn("Task deadline was surpassed before having budget to add it to the schedule. " + str(task))
                self.currently_bid_tokens-=task.priority
                del self.queued_tasks[i]
            elif self.available_tokens - self.currently_added_tokens - task.priority >= 0:
                try:
                    task_id = self.add_task(task).task_id
                    rospy.loginfo("Added task " + str(task) + " with ID: " + str(task_id))
                    self.added_tasks[task_id] = task
                    self.currently_added_tokens+=task.priority
                    del self.queued_tasks[i]
                except Exception, e:
                    rospy.logerr("Error calling add task service: " + str(e))
            else:
                i+=1

    
    def process_task_event(self, msg):
        task = msg.task
        if self.added_tasks.has_key(task.task_id):
            if msg.event == TaskEvent.DROPPED:
                rospy.loginfo("Task was dropped by the executor. " + str(task))
                self.currently_bid_tokens-=task.priority
                self.currently_added_tokens-=task.priority
                del self.added_tasks[task.task_id]
            elif msg.event in [TaskEvent.TASK_FAILED, TaskEvent.TASK_SUCCEEDED, TaskEvent.TASK_PREEMPTED]:
                rospy.loginfo("Task has finished execution with outcome " + msg_event_to_string(msg.event) + ". Retrieving bid of " + str(task.priority) + ". Task: " + str(task))
                self.currently_bid_tokens-=task.priority
                self.available_tokens-=task.priority
                self.currently_added_tokens-=task.priority
                del self.added_tasks[task.task_id]
            self.process_task_queue()

            
        

        
        