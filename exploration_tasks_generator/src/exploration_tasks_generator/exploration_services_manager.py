import rospy
import rosservice

from strands_exploration_msgs.srv import GetExplorationTasks
from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils as tu

class ExplorationTask(object):
    def __init__(self, task_type, score, task_def):
        self.task_type=task_type
        self.score=score
        self.task_def=task_def


class ExplorationServicesManager(object):
    def __init__(self):
        self.current_tasks=[]
        
        
    def generate_tasks(self, start_time, end_time):
        task_list=[]
        exploration_service_names=rosservice.rosservice_find("strands_exploration_msgs/GetExplorationTasks")
        exploration_service_list=[rospy.ServiceProxy(name, GetExplorationTasks) for name in exploration_service_names]
        for (name, service) in zip(exploration_service_names, exploration_service_list):
            response=service(start_time, end_time)
            no_slash_name=name.replace('/', '_')
            method_name="create_tasks_for" + no_slash_name
            if hasattr(self, method_name) and callable(getattr(self, method_name)):
                method=getattr(self, method_name)
                task_list+=method(response.task_definition, response.task_score, start_time, end_time)
            else:
                rospy.logwarn("No method defined to create tasks for service " + name)
        return task_list
    
        
    def create_tasks_for_edge_exp_srv(self, definitions, scores, start_time, end_time):
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
        for (wp, score) in zip(definitions, scores):
            task=Task(action= 'ptu_pan_tilt_metric_map',
                      start_node_id=wp,
                      end_node_id=wp,
                      start_after=start_time,
                      end_before=end_time,
                      max_duration=rospy.Duration(4*60))
            res.append(ExplorationTask(task_type="fremen_grid_exploration",
                                       score=score,
                                       task_def=task))
        return res     
            
