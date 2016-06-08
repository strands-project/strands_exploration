import rospy
import rosservice

from copy import deepcopy

from strands_exploration_msgs.srv import GetExplorationTasks
from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils as tu

from datetime import time, date, timedelta, datetime
from dateutil.tz import tzlocal

def get_x_best(scores, n):
    scores=list(scores)
    scores.sort(reverse=True)
    return scores[n-1]

def time_greater_than(t1, t2):
    """ Seems to be a bug in datetime when comparing localtz dates, so do this instead. """
    if t1.hour > t2.hour:
        return True
    elif t1.hour == t2.hour:
        if t1.minute > t2.minute:
            return True 
        elif t1.minute == t2.minute:
            if t1.second > t2.second:
                return True
            elif t1.second == t2.second:
                return t1.microsecond > t2.microsecond

    return False

def time_less_than(t1, t2):
    """ Seems to be a bug in datetime when comparing localtz dates, so do this instead. """
    if t1.hour < t2.hour:
        return True
    elif t1.hour == t2.hour:
        if t1.minute < t2.minute:
            return True 
        elif t1.minute == t2.minute:
            if t1.second < t2.second:
                return True
            elif t1.second == t2.second:
                return t1.microsecond < t2.microsecond
            

class ExplorationTask(object):
    def __init__(self, task_type, score, task_def):
        self.task_type=task_type
        self.score=score
        self.task_def=task_def


class ExplorationServicesManager(object):
    def __init__(self):
        self.current_tasks=[]
        
        self.localtz = tzlocal()
        self.start = time(9,0, tzinfo=self.localtz)
        self.end = time(17,0, tzinfo=self.localtz)
        
    

    def generate_tasks(self, start_time, end_time):
        task_list=[]
        exploration_service_names=rosservice.get_service_list(namespace="/exploration_services")
        print exploration_service_names
        exploration_service_list=[rospy.ServiceProxy(name, GetExplorationTasks) for name in exploration_service_names]
        for (name, service) in zip(exploration_service_names, exploration_service_list):
            try:
                rospy.loginfo("Calling exploration service " + name)
                response=service(start_time, end_time)
                service_name=name.split('/')[-1]
                print service_name
                method_name="create_tasks_for_" + service_name
                if hasattr(self, method_name) and callable(getattr(self, method_name)):
                    method=getattr(self, method_name)
                    task_list+=method(response.task_definition, response.task_score, start_time, end_time)
                else:
                    rospy.logwarn("No method defined to create tasks for service " + name)
            except rospy.ServiceException, e:
                rospy.logwarn("Error calling service " + name + ":" + str(e) +". Skipping...")
        return task_list
    
        
    def create_tasks_for_get_edge_exploration(self, definitions, scores, start_time, end_time):
        res=[]
        for (edge, score) in zip(definitions, scores):
            [start_wp, end_wp]=edge.split("_")
            task=Task(action= '(F ("' + start_wp + '" & (X "' + end_wp + '")))',
                      start_after=start_time,
                      end_before=end_time,
                      max_duration=rospy.Duration(1))
            res.append(ExplorationTask(task_type="edge_exploration",
                                       score=score,
                                       task_def=task))
        return res         

    def create_tasks_for_fremen_grid_exp_srv(self, definitions, scores, start_time, end_time):
        res=[]
        do_learn=False
        if date.today().isoweekday():
            current_time = datetime.fromtimestamp(rospy.get_rostime().to_sec(), tz=self.localtz).time()
            if time_greater_than(current_time, self.start) and time_less_than(current_time, self.end):
                do_learn=True
                n_learns=rospy.get_param("~n_learns_per_trigger", 1)
                learn_thresh=get_x_best(deepcopy(scores), n_learns)
                added_learns=0
        for (wp, score) in zip(definitions, scores):       
            if do_learn and score >= learn_thresh and added_learns < n_learns:
                added_learns+=1
                max_duration=rospy.Duration(rospy.get_param("~object_learn_dur", 60*15))
                task=Task(action= 'learn_object',
                        start_node_id=wp,
                        end_node_id=wp,
                        start_after=start_time,
                        end_before=end_time,
                        max_duration=max_duration)
                tu.add_string_argument(task, wp)
                res.append(ExplorationTask(task_type="fremen_grid_exploration",
                                        score=score,
                                        task_def=task))
            else:
                task=Task(action= 'do_sweep',
                        start_node_id=wp,
                        end_node_id=wp,
                        start_after=start_time,
                        end_before=end_time,
                        max_duration=rospy.Duration(1*60))
                tu.add_string_argument(task, 'medium')
                res.append(ExplorationTask(task_type="fremen_grid_exploration",
                                        score=score,
                                        task_def=task))
        return res     
            

    def create_tasks_for_activity_exp_srv(self, definitions, scores, start_time, end_time):
        res=[]
        for (wp, score) in zip(definitions, scores):
            duration=rospy.Duration(3*60)
            max_duration=duration + rospy.Duration(3*60) #TODO Params for this, and other times 
    
            task=Task(action= 'skeleton_action',
                      start_node_id=wp,
                      end_node_id=wp,
                      start_after=start_time,
                      end_before=end_time,
                      max_duration=max_duration)
            tu.add_duration_argument(task, duration)
            tu.add_string_argument(task, wp)
            res.append(ExplorationTask(task_type="activity_exploration",
                                       score=score,
                                       task_def=task))
        return res
