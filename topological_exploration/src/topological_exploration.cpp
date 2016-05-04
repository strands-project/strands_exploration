#include <stdlib.h>
#include "ros/ros.h"
#include <mongodb_store/message_store.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <topological_exploration/topological_explorationConfig.h>
#include <strands_navigation_msgs/NavStatistics.h>
#include <strands_navigation_msgs/TopologicalMap.h>
#include <strands_navigation_msgs/TopologicalNode.h>
#include <strands_executive_msgs/AddTask.h>
#include <mongodb_store_msgs/StringPair.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <strands_navigation_msgs/GetTaggedNodes.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <scitos_msgs/BatteryState.h>
#include <strands_exploration_msgs//FremenGrid.h>
#include <strands_exploration_msgs/Frelement.h>
#include <strands_exploration_msgs/SFrelement.h>
#include <strands_exploration_msgs/LoadGrid.h>
#include <strands_exploration_msgs/SaveGrid.h>
#include "CFremenGridSet.h"
#include <time.h> 
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <cassert>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <strands_exploration_msgs/AddView.h>
#include <strands_exploration_msgs/Visualize.h>
#include <std_msgs/Float64.h>
#include <scitos_ptu/PanTiltActionFeedback.h>
#include <strands_exploration_msgs/GetExplorationTasks.h>




using namespace mongodb_store;
using namespace std;

//FIXED parameters
int windowDuration = 1200;//180;
int rescheduleInterval = 86400;

//3D grid parameters
double cellSize = 0.1;
int dimX = 100;
int dimY = 100;
int dimZ = 100;
float camera_range = 4.0;

//standard parameters
string collectionName;
string scheduleDirectory;
string sweep_type;

//runtine parameters
float explorationRatio = 0.15;
int8_t   minimalBatteryLevel = 10;
int32_t   minimalBatteryLevelTime = 0;
int interactionTimeout = 30;
int maxTaskNumber = 1;
int taskDuration = 60;//180;
int taskPriority = 1;	
bool debug = true;
int taskStartDelay = 5;
int rescheduleCheckTime = 5;
float entropyThreshold = 25000;

//ROS communication
MessageStoreProxy *messageStore;
ros::Subscriber robotPoseSub;
ros::Subscriber currentNodeSub;
ros::Subscriber interfaceSub; 
ros::Subscriber batterySub;
ros::Subscriber infoTaskSub;
ros::Subscriber guiSub;
ros::Subscriber mapSub;
ros::Subscriber ptu_state_sub;
ros::Subscriber node_sub;
ros::ServiceClient nodeListClient;
ros::ServiceClient taskAdder;
ros::ServiceClient load_service;
ros::ServiceClient save_service;
ros::Publisher  grid_markers_pub;
ros::Publisher information_pub;


//Fremen grid component
CFremenGridSet fremengridSet;

strands_navigation_msgs::TopologicalMap topoMap;

//state variables
int lastTimeSlot = -1;
int currentTimeSlot = -1;
int numCurrentTasks = 0; 
string nodeName = "ChargingPoint";
string closestNode = "ChargingPoint";
int32_t lastInteractionTime = -1;
bool forceCharging = false;
int timeOffset = 0;

//times and nodes of schedule
uint32_t timeSlots[10000];
int nodes[10000];
float node_entropies[1000];
float max_entropy;
int taskIDs[10000];
int numNodes = 0;

float info;
//unsigned int info_index = 0;

//Ray casting parameters
int integrateMeasurements = 0;
int maxMeasurements = 1;//15;
int measurements = maxMeasurements;
//float *dept;
bool first_grid = true;
bool incorporating = false;
unsigned int timestamp;
int gridIndex;
unsigned int sweep_measurements;
unsigned int current_measurement;

std::vector<std::string> critical_nodes, exploration_nodes;
//std::vector<std::string> exploration_nodes;

string currentNode;

tf::TransformListener *tf_listener;

uint32_t getMidnightTime(uint32_t givenTime)
{
    return ((givenTime+timeOffset)/rescheduleInterval)*rescheduleInterval-timeOffset;
}

/*parameter reconfiguration*/
void reconfigureCallback(topological_exploration::topological_explorationConfig &config, uint32_t level) 
{
    ROS_INFO("Reconfigure Request: %lf %d %d %d %d", config.explorationRatio, config.minimalBatteryLevel, config.interactionTimeout, config.maxTaskNumber, config.taskDuration);
    explorationRatio = config.explorationRatio;
    minimalBatteryLevel = config.minimalBatteryLevel;
    interactionTimeout = config.interactionTimeout;
    maxTaskNumber = config.maxTaskNumber;
    taskDuration = config.taskDuration;
    taskPriority = config.taskPriority;
    debug = config.verbose;
    taskStartDelay = config.taskStartDelay;
    rescheduleCheckTime = config.rescheduleCheckTime;
    entropyThreshold = config.entropyThreshold;
}

/*listen to battery and set forced charging if necessary*/
void batteryCallBack(const scitos_msgs::BatteryState &msg)
{
    ROS_DEBUG("SpatioTemporal Exploration: battery level %i %i",msg.lifePercent,minimalBatteryLevel);
    if (minimalBatteryLevel > msg.lifePercent) forceCharging = true; else forceCharging = false;
}

/*gets topological map*/
void getTopologicalMap(const strands_navigation_msgs::TopologicalMap::ConstPtr& msg)
{
    topoMap = *msg;
}

/*returns the coordinates for a given waypoint*/
int coordinateSearch(string name, geometry_msgs::Point* point)
{
    for(int i = 0; topoMap.nodes.size(); i++)
    {
        if(topoMap.nodes[i].name.compare(name) == 0)
        {
            point->x = topoMap.nodes[i].pose.position.x;
            point->y = topoMap.nodes[i].pose.position.y;
            point->z = topoMap.nodes[i].pose.position.z;
            return 0;
        }
    }

    return -1;
}

int publishGrid(string waypoint, unsigned int stamp, float minP, float maxP, float period, unsigned int type, string name, bool set_color,  std_msgs::ColorRGBA m_color)
{
    //init visualization markers:
    visualization_msgs::Marker markers;
    geometry_msgs::Point cubeCenter;

    //get grid ID
    int id = fremengridSet.find(waypoint.c_str());

    //set type, frame and size
    markers.header.frame_id = "/map";
    markers.header.stamp = ros::Time::now();
    markers.ns = name;
    markers.action = visualization_msgs::Marker::ADD;
    markers.type = visualization_msgs::Marker::CUBE_LIST;
    markers.scale.x = fremengridSet.fremengrid[id]->resolution;
    markers.scale.y = fremengridSet.fremengrid[id]->resolution;;
    markers.scale.z = fremengridSet.fremengrid[id]->resolution;;
    markers.color = m_color;
    markers.points.clear();

    // prepare to iterate over the entire grid
    float resolution = fremengridSet.fremengrid[id]->resolution;
    float minX = fremengridSet.fremengrid[id]->oX;
    float minY = fremengridSet.fremengrid[id]->oY;
    float minZ = fremengridSet.fremengrid[id]->oZ;
    float maxX = minX+fremengridSet.fremengrid[id]->xDim*resolution-3*resolution/4;
    float maxY = minY+fremengridSet.fremengrid[id]->yDim*resolution-3*resolution/4;
    float maxZ = minZ+minY+fremengridSet.fremengrid[id]->zDim*resolution-3*resolution/4;//2.1
    int cnt = 0;
    int cells = 0;
    float estimate;

    if (stamp != 0 && type == 1) fremengridSet.fremengrid[id]->recalculate(stamp);

    //iterate over the cells' probabilities
    for(float z = minZ;z<maxZ;z+=resolution){
        for(float y = minY;y<maxY;y+=resolution){
            for(float x = minX;x<maxX;x+=resolution){
                if (type == 0) estimate = fremengridSet.fremengrid[id]->retrieve(cnt);          	//short-term memory grid
                if (type == 1) estimate = fremengridSet.fremengrid[id]->estimate(cnt,0);			//long-term memory grid
                if (type == 2) estimate = fremengridSet.fremengrid[id]->aux[cnt];               	//auxiliary grid
                if (type == 3) estimate = fremengridSet.fremengrid[id]->getDominant(cnt,period);	//dominant frequency amplitude

                if(estimate > minP && estimate < maxP)
                {
                    if(set_color == 0)
                    {
                        m_color.r = 0.0;
                        m_color.b = z/maxZ;
                        m_color.g = 1.0 - m_color.b;
                        m_color.a = 0.8;
                    }
                    cubeCenter.x = x;
                    cubeCenter.y = y;
                    cubeCenter.z = z;
                    markers.points.push_back(cubeCenter);
                    markers.colors.push_back(m_color);
                    cells++;
                }
                cnt++;
            }
        }
    }

    //publish results
    grid_markers_pub.publish(markers);
    return cells;
}

bool visualizeGrid(strands_exploration_msgs::Visualize::Request  &req, strands_exploration_msgs::Visualize::Response &res)
{
    int numvoxels = publishGrid(req.waypoint, req.stamp, req.minProbability, req.maxProbability, req.period, req.type, req.name, req.set_color, req.color);
    ROS_INFO("%d voxels published.", numvoxels);
}

bool explorationRoutine(strands_exploration_msgs::GetExplorationTasks::Request &req, strands_exploration_msgs::GetExplorationTasks::Response &res)
{
    unsigned int request_interval = req.end_time.sec - req.start_time.sec;
    unsigned int request_slots = request_interval/windowDuration;

    if(request_interval < 0)
    {
        ROS_ERROR("Invalid time interval!");
        return(false);
    }

    ROS_INFO("Exploration routine: start time %d duration %d time slots %d", req.start_time.sec,request_interval,request_slots);

    int initial_slot = (req.start_time.sec-timeSlots[0])/windowDuration;

    int numSlots = 24*3600/windowDuration;

    for(int i = 0; i <= request_slots; i++)
    {
        if(initial_slot + i < numSlots)
        {
            res.task_definition.push_back(fremengridSet.fremengrid[nodes[initial_slot + i]]->id);
            res.task_score.push_back(node_entropies[initial_slot + i]);
            ROS_INFO("Exploration routine: node %s score %f ", fremengridSet.fremengrid[nodes[initial_slot + i]]->id, node_entropies[initial_slot + i]);
        }
        else
            break;
    }

    return true;
}

/*updates the closest node*/
void getCurrentNode(const std_msgs::String::ConstPtr& msg)
{
    closestNode = msg->data;
    if (fremengridSet.find(msg->data.c_str())>-1){
        nodeName = msg->data;
        if (debug) ROS_INFO("Closest Exploration node switched to %s.",nodeName.c_str());
    }else{
        if (debug) ROS_INFO("Closest node %s - however, it's not an Exploration node.",msg->data.c_str());
    }
}

/*loads relevant nodes from the map description*/
int getRelevantNodes(string tag, std::vector<std::string>* exploration_nodes)
{
    int result = -1;

    strands_navigation_msgs::GetTaggedNodes srv;
    srv.request.tag = tag;
    if (nodeListClient.call(srv))
    {
        for (int i = 0;i<srv.response.nodes.size();i++)
        {
            exploration_nodes->push_back(srv.response.nodes[i]);
            geometry_msgs::Point node_coordinates, grid_origin;
            coordinateSearch(srv.response.nodes[i], &node_coordinates);

            //grid origin calculation
            grid_origin.x = node_coordinates.x - (dimX*cellSize)/2;
            grid_origin.y = node_coordinates.y - (dimY*cellSize)/2;
            grid_origin.z = -0.05;

            fremengridSet.add(srv.response.nodes[i].c_str(), grid_origin.x, grid_origin.y, grid_origin.z, dimX, dimY, dimZ, cellSize);
        }

        numNodes = fremengridSet.numFremenGrids;
        ROS_INFO("Found %d nodes with tag %s", (int) srv.response.nodes.size(), tag.c_str());
        for (int i= numNodes - srv.response.nodes.size();i<numNodes;i++) ROS_INFO("FreMEnGrid ID: %i Label: %s.",i,fremengridSet.fremengrid[i]->id);
        result = srv.response.nodes.size();
    }
    else
    {
        ROS_ERROR("Failed to obtain exploration nodes with tag %s.", tag.c_str());
    }
    return result;
}

/*retrieve grids from the database, updates the grids and estimates information gain*/
void retrieveGrids(uint32_t lastTime)//TODO -> call load service
{
    char testTime[1000];
    vector< boost::shared_ptr<strands_exploration_msgs::FremenGrid> > results;
    messageStore->query<strands_exploration_msgs::FremenGrid>(results);
    //    BOOST_FOREACH( boost::shared_ptr<topological_exploration::FremenGrid> p,  results)
    //    {
    //        time_t timeInfo = p->time;
    //        strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
    //        ROS_INFO("There were %d interaction at %s at waypoint %s.",p->number,testTime,p->waypoint.c_str());
    //        if (lastTime > p->time){
    //            if (p->number>1) addResult(p->waypoint.c_str(),1,p->time); else addResult(p->waypoint.c_str(),0,p->time);
    //        }
    //    }
}

/*generates a schedule and saves it in a file*/
int generateNewSchedule(uint32_t givenTime)//TODO -> save schedule in MongoDB
{
    /*establish relevant time frame*/
    int numSlots = 24*3600/windowDuration;
    uint32_t timeSlots[numSlots];
    uint32_t midnight = getMidnightTime(givenTime);
    retrieveGrids(midnight);

    /*create timeslots*/
    for (int i = 0;i<numSlots;i++) timeSlots[i] = midnight+3600*24/numSlots*i;
    numNodes = fremengridSet.numFremenGrids;

    /*evaluate the individual nodes*/
    max_entropy = 0.0;
    int p = 0;
    float slot_entropies[1000];
    float entropy[1];
    uint32_t times[1];
    char dummy[1000];
    float wheel[numNodes];
    float lastWheel;
    geometry_msgs::Point observationPoint;

    for (int s=0;s<numSlots;s++)
    {
        /*evaluate nodes for the given time*/
        lastWheel = 0;
        for (int i=0;i<numNodes;i++)
        {
            times[0] = timeSlots[s];
            coordinateSearch(fremengridSet.fremengrid[i]->id, &observationPoint);
            fremengridSet.recalculate(fremengridSet.fremengrid[i]->id, times[0]);
            slot_entropies[i] = entropy[0] = fremengridSet.estimateEntropy(fremengridSet.fremengrid[i]->id, observationPoint.x, observationPoint.y, 1.662, camera_range, times[0]);

            lastWheel += entropy[0];
            wheel[i] = lastWheel;
        }

        /*some debug prints*/
        if (debug)
        {
            sprintf(dummy, "Cumulative node evaluations: ");
            for (int i=0;i<numNodes;i++) sprintf(dummy,"%s %.3f ",dummy,wheel[i]);
            sprintf(dummy,"%s %.3f",dummy,lastWheel);
            ROS_INFO("%s",dummy);
        }
        /*random seletion of a particular node*/
        int node = 0;
        float randomNum = 0;
        randomNum = (float)rand()/RAND_MAX*lastWheel;
        p = 0;
        while (p <numNodes && randomNum > wheel[p]) p++;
        nodes[s] = p;
        node_entropies[s] = slot_entropies[p];
        if(node_entropies[s] >= max_entropy)
            max_entropy = node_entropies[s];
    }

    /*save a file with a schedule TODO - save to mongo + interface calendar*/
    time_t timeInfo = midnight;
    char fileName[1000];
    strftime(dummy, sizeof(dummy), "InfoTerminal-Schedule-%Y-%m-%d.txt",localtime(&timeInfo));
    sprintf(fileName,"%s/%s",scheduleDirectory.c_str(),dummy);
    printf("Generating schedule from %s\n",fileName);
    FILE* file = fopen(fileName,"w+");
    if (file == NULL)ROS_ERROR("Could not open Schedule file %s.",fileName);
    for (int s=0;s<numSlots;s++)
    {
        node_entropies[s] = node_entropies[s]/max_entropy;
        sprintf(dummy,"%i ",s);
        timeInfo = timeSlots[s];
        strftime(dummy, sizeof(dummy), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
        fprintf(file,"%ld %s %s %f\n",timeInfo,dummy,fremengridSet.fremengrid[nodes[s]]->id, node_entropies[s]);
        ROS_INFO("Schedule: %ld %s %s %f\n",timeInfo,dummy,fremengridSet.fremengrid[nodes[s]]->id, node_entropies[s]);
    }
    fclose(file);
}

int generateSchedule(uint32_t givenTime)//TODO -> save schedule in MongoDB
{
    char dummy[1000];
    int numSlots = 24*3600/windowDuration;
    uint32_t midnight =  getMidnightTime(givenTime);
    if (debug)
    {
        ROS_INFO("TIME: %i",givenTime);
        ROS_INFO("MIDNIGHT: %i %i",midnight,midnight-givenTime);
    }

    /*create timeslots*/
    for (int i = 0;i<numSlots;i++) timeSlots[i] = midnight+3600*24/numSlots*i;

    /*open a schedule file - create a new one if it does not exist*/
    char fileName[1000];
    time_t timeInfo = midnight;
    strftime(dummy, sizeof(dummy), "InfoTerminal-Schedule-%Y-%m-%d.txt",localtime(&timeInfo));
    sprintf(fileName,"%s/%s",scheduleDirectory.c_str(),dummy);
    ROS_INFO("Retrieving schedule from %s",fileName);
    FILE* file = fopen(fileName,"r");
    if (file == NULL){
        ROS_INFO("Schedule file not found %s\n",fileName);
        ROS_INFO("Generating new schedule. Estimating entropies...");
        generateNewSchedule(givenTime);
        file = fopen(fileName,"r");
    }

    /*read schedule file*/
    int checkReturn;
    uint64_t slot;
    float norm_entropy;
    char nodeName[1000];
    char testTime[1000];
    for (int s=0;s<numSlots;s++){
        checkReturn = fscanf(file,"%ld %s %s %f\n",&slot,dummy,nodeName, &norm_entropy);
        timeInfo = slot;
        node_entropies[s] = norm_entropy;
        strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
        if (checkReturn != 4) 			ROS_ERROR("Exploration schedule file %s is corrupt at line %i (wrong number of entries %i)!",dummy,s,checkReturn);
        else if (slot != timeSlots[s]) 		ROS_ERROR("Exploration schedule file %s is corrupt at line %i (ROS time mismatch)!",dummy,s);
        else if (strcmp(testTime,dummy)!=0)	ROS_ERROR("Exploration schedule file %s is corrupt at line %i (time in seconds does not match string time)!",dummy,s);
        else {
            nodes[s] = fremengridSet.find(nodeName);
            if (nodes[s] < 0) ROS_ERROR("Exploration schedule file %s is corrupt at line %i (node %s is not tagged as InfoTerminal in topoogical map)!",dummy,s,nodeName);
        }
    }
    fclose(file);
}

int getNextTimeSlot(int lookAhead)
{
    char dummy[1000];
    char testTime[1000];
    time_t timeInfo;
    int numSlots = 24*3600/windowDuration;
    ros::Time currentTime = ros::Time::now();
    uint32_t givenTime = currentTime.sec+lookAhead*windowDuration+rescheduleCheckTime;
    uint32_t midnight = getMidnightTime(givenTime);
    if (timeSlots[0] != midnight)
    {
        ROS_INFO("Generating new schedule!");
        generateSchedule(givenTime);
    }
    int currentSlot = (givenTime-timeSlots[0])/windowDuration;
    //ROS_INFO("Time %i - slot %i: going to node %i(%s).",currentTime.sec-midnight,currentSlot,nodes[currentSlot],fremengridSet.frelements[nodes[currentSlot]]->id);
    if (debug)
    {
        timeInfo = givenTime;
        strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
        sprintf(dummy,"time %i %s - slot %i: going to node %i(%s).",givenTime,testTime,currentSlot,nodes[currentSlot],fremengridSet.fremengrid[nodes[currentSlot]]->id);
        //ROS_INFO("%s",dummy);
    }
    if (currentSlot >= 0 && currentSlot < numSlots) return currentSlot;
    ROS_ERROR("Exploration schedule error: attempting to get task in a non-existent time slot.");
    return -1;
}

/*creates a task for the given slot*/
int createTask(int slot)
{
    char dummy[1000];
    char testTime[1000];
    time_t timeInfo = timeSlots[slot];
    strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));

    bool critical = false;
    //if critical...
    for(int i = 0; i < critical_nodes.size(); i++)
    {
        if(strcmp(fremengridSet.fremengrid[nodes[slot]]->id, critical_nodes[i].c_str()) == 0)
        {
            critical = true;
            break;
        }

    }

    if(critical)
    {
        mongodb_store_msgs::StringPair taskArg;
        taskArg.second = "complete";
        strands_executive_msgs::Task task;
        task.action = "do_sweep";
        task.start_node_id = fremengridSet.fremengrid[nodes[slot]]->id;
        task.end_node_id = fremengridSet.fremengrid[nodes[slot]]->id;
        task.priority = taskPriority;
        task.arguments.push_back(taskArg);

        task.start_after =  ros::Time(timeSlots[slot]+taskStartDelay,0);
        task.end_before = ros::Time(timeSlots[slot]+windowDuration - 2,0);
        task.max_duration = task.end_before - task.start_after;
        strands_executive_msgs::AddTask taskAdd;
        taskAdd.request.task = task;
        if (taskAdder.call(taskAdd))
        {
            sprintf(dummy,"%s for timeslot %i on %s, between %i and %i.",fremengridSet.fremengrid[nodes[slot]]->id,slot,testTime,task.start_after.sec,task.end_before.sec);
            ROS_INFO("Task %ld created at %s (ground truth node).", taskAdd.response.task_id,dummy);
            taskIDs[slot] = taskAdd.response.task_id;
        }
    }
}

/*saves grid to database*/
int saveGridDB(string name)
{
    int gridIndex = fremengridSet.find(name.c_str());

    if(gridIndex < 0)
    {
        ROS_ERROR("%s grid not found!", name.c_str());
        return -1;
    }
    else
    {

        /*Fremen grid*/
        strands_exploration_msgs::FremenGrid grid_msg;

        //Waypoint name
        grid_msg.waypoint = name;
        //Grid dimensions:
        grid_msg.dimensions.x = fremengridSet.fremengrid[gridIndex]->xDim;
        grid_msg.dimensions.y = fremengridSet.fremengrid[gridIndex]->yDim;
        grid_msg.dimensions.z = fremengridSet.fremengrid[gridIndex]->zDim;
        //Grid origin
        grid_msg.origin.x = fremengridSet.fremengrid[gridIndex]->oX;
        grid_msg.origin.y = fremengridSet.fremengrid[gridIndex]->oY;
        grid_msg.origin.z = fremengridSet.fremengrid[gridIndex]->oZ;
        //resolution
        grid_msg.resolution = fremengridSet.fremengrid[gridIndex]->resolution;
        //Last information gain value (useful to evaluate the strategy performance)
        grid_msg.last_info = fremengridSet.fremengrid[gridIndex]->obtainedInformationLast;
        //Predicted information gain
        grid_msg.predicted_info = fremengridSet.fremengrid[gridIndex]->obtainedInformationPredicted;
        grid_msg.numcells = fremengridSet.fremengrid[gridIndex]->numCells;

        /*Frelement*/
        strands_exploration_msgs::Frelement frelement_msg;

        int frk;

        /*SFrelement*/
        for(int i = 0; i < grid_msg.numcells; i++)
        {
            frk = fremengridSet.fremengrid[gridIndex]->frelements[i].frk;

            frelement_msg.gain = fremengridSet.fremengrid[gridIndex]->frelements[i].gain;
            frelement_msg.measurements = fremengridSet.fremengrid[gridIndex]->frelements[i].measurements;
            frelement_msg.first_time = fremengridSet.fremengrid[gridIndex]->frelements[i].firstTime;
            frelement_msg.last_time = fremengridSet.fremengrid[gridIndex]->frelements[i].lastTime;

            if(frelement_msg.measurements == 0 ||frelement_msg.gain == 0)
                frelement_msg.frequencies = 0;
            else
            {
                frelement_msg.frequencies = frk;

                for(int j = 0; j < frelement_msg.frequencies; j++)
                {
                    frelement_msg.elements[i].rstates = fremengridSet.fremengrid[gridIndex]->frelements[i].allFrelements[j].realStates;
                    frelement_msg.elements[i].istates = fremengridSet.fremengrid[gridIndex]->frelements[i].allFrelements[j].imagStates;
                    frelement_msg.elements[i].rbalance = fremengridSet.fremengrid[gridIndex]->frelements[i].allFrelements[j].realBalance;
                    frelement_msg.elements[i].ibalance = fremengridSet.fremengrid[gridIndex]->frelements[i].allFrelements[j].imagBalance;
                }
            }

            grid_msg.frelements.push_back(frelement_msg);
            frelement_msg.elements.clear();
        }


        ros::Time currentTime = ros::Time::now();
        lastInteractionTime = currentTime.sec;
        grid_msg.time = currentTime.sec;

        strands_exploration_msgs::SaveGrid save_req;
        save_req.request.grid = grid_msg;

        if(save_service.call(save_req))
            ROS_INFO("FremenGrid %s inserted with id %i", name.c_str(), gridIndex);
        else
            ROS_ERROR("Failed to save 3D grid.");
    }

}

int removeGrid(const char *name)
{
    if(fremengridSet.remove(name) < 0)
        ROS_ERROR("Error removing grid. %s grid not found!", name);
    else
        ROS_INFO("Removed %s grid!", name);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    if(current_measurement == 0)
    {
        //get grid index according to the waypoint
        gridIndex = fremengridSet.find(nodeName.c_str());
        ROS_INFO("Waypoint %s and grid index %i\n", nodeName.c_str(), gridIndex);
    }


    timestamp = msg->header.stamp.sec;

    ROS_INFO("Add depth called at Waypoint %s and grid index %i\n", nodeName.c_str(), gridIndex);

    //stores the depth image for easier manipultation
    float depth = msg->data[640*480+640]+256*msg->data[640*480+640+1];
    float *dept;
    dept = (float*)malloc(sizeof(float)*307200*(maxMeasurements+1));
    measurements = 0;

    //to filter some of the noise
    float *dataPtr;
    float di;
    if (measurements < maxMeasurements)
    {
        float di = 0;

        //ROS_INFO("maxMeasurements: %d", maxMeasurements);
        if (measurements == 0) memset(dept,0,sizeof(float)*307200*(maxMeasurements+1));


        for (int i = 0;i<307200;i++)
        {

            dataPtr = &dept[i*(maxMeasurements+1)];
            di = (msg->data[i*2]+256*msg->data[i*2+1])/1000.0;
            dataPtr[measurements+1] = di;
            int j=measurements+1;
            while (dataPtr[j] < dataPtr[j-1])
            {
                dataPtr[j] = dataPtr[j-1];
                dataPtr[j-1] = di;
                j--;
            }
        }
    }
    measurements++;

    if (measurements==maxMeasurements)
    {
        //ray casting auxiliary variables
        int len =  msg->height*msg->width;
        float vx = 1/570.0;
        float vy = 1/570.0;
        float cx = -320.5;
        float cy = -240.5;
        int width = msg->width;
        int height = msg->height;
        float fx = (1+cx)*vx;
        float fy = (1+cy)*vy;
        float lx = (width+cx)*vx;
        float ly = (height+cy)*vy;
        float x[len+1];
        float y[len+1];
        float z[len+1];
        float d[len+1];
        float di,psi,phi,phiPtu,psiPtu,xPtu,yPtu,zPtu,ix,iy,iz;
        int cnt = 0;
        di=psi=phi=phiPtu=psiPtu=xPtu=yPtu=zPtu=0;

        //transform depth image from optical frame to map frame
        tf::StampedTransform st;
        try {
            tf_listener->waitForTransform("/map","/head_xtion_depth_optical_frame",msg->header.stamp, ros::Duration(0.5));
            tf_listener->lookupTransform("/map","/head_xtion_depth_optical_frame",msg->header.stamp,st);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("FreMEn map cound not incorporate the latest depth map %s",ex.what());
            return;
        }

        //incorporate depth image
        //incorporating = true;
        CTimer timer;
        timer.reset();
        timer.start();
        x[len] = xPtu = st.getOrigin().x();
        y[len] = yPtu = st.getOrigin().y();
        z[len] = zPtu = st.getOrigin().z();
        double a,b,c;
        tf::Matrix3x3  rot = st.getBasis();
        rot.getEulerYPR(a,b,c,1);
        psi = -M_PI/2-c;
        phi = a-c-psi;
        int medinda; //median index

        for (float h = fy;h<ly;h+=vy)
        {
            for (float w = fx;w<lx;w+=vx)
            {
                medinda = cnt*(maxMeasurements+1);
                di = dept[medinda+(maxMeasurements+1)/2];//(msg->data[cnt*2]+256*msg->data[cnt*2+1])/1000.0;
                d[cnt] = 1;
                if (di < 0.05 || di >= camera_range || dept[medinda+1] != dept[medinda+maxMeasurements]) //basically, all noise is rejected
                {
                    di = camera_range;
                    d[cnt] = 0;
                }
                ix = di*(cos(psi)-sin(psi)*h);
                iy = -w*di;
                iz = -di*(sin(psi)+cos(psi)*h);
                x[cnt] = cos(phi)*ix-sin(phi)*iy+xPtu;
                y[cnt] = sin(phi)*ix+cos(phi)*iy+yPtu;
                z[cnt] = iz+zPtu;
                cnt++;
            }
        }

        //int lastInfo = fremengridSet.fremengrid[gridIndex]->obtainedInformationLast;
        ROS_INFO("Depth image added to the grid (%d/%d)", current_measurement, sweep_measurements - 1);
        fremengridSet.fremengrid[gridIndex]->incorporate(x,y,z,d,len,timestamp);
    }



    if(current_measurement == sweep_measurements-1)
    {
        std_msgs::ColorRGBA color_aux;
        color_aux.g = color_aux.a = 1.0;
        color_aux.r = color_aux.b = 0.0;
        ROS_INFO("Sweep complete. Publishing and saving 3D grid...");
        int numvoxels = publishGrid(nodeName.c_str(), 100000, 0.9, 1.0, 86400, 0, nodeName.c_str(), false, color_aux);
        ROS_INFO("%d voxels published.", numvoxels);
        saveGridDB(nodeName);
        current_measurement = 0;
    }
    else
        current_measurement++;
}



int main(int argc,char* argv[])
{
    //determine ROS time to local mignight offset - used to replan at midnight
    tzset();
    timeOffset = timezone+daylight*3600;
    if (debug) ROS_DEBUG("Local time offset %i",timeOffset);

    //initialize ROS
    ros::init(argc, argv, "spatiotemporal_exploration");
    ros::NodeHandle n("~");

    //initialize datacentre
    messageStore = new MessageStoreProxy(n,"message_store");


    //load parameters
    n.param<std::string>("sweep_type", sweep_type, "complete");
    n.param<std::string>("collectionName", collectionName, "FremenGrid");
    n.param<std::string>("scheduleDirectory", scheduleDirectory, "/localhome/strands/schedules");
    n.param("taskPriority", taskPriority,1);
    n.param("verbose", debug,false);
    n.param("resolution", cellSize, 0.1);
    n.param("dimX", dimX, 100);
    n.param("dimY", dimY, 100);
    n.param("dimZ", dimZ, 40);

    if(sweep_type.compare("complete") == 0)
        sweep_measurements = 51;
    else if(sweep_type.compare("medium") == 0)
        sweep_measurements = 17;
    else if(sweep_type.compare("short") == 0)
        sweep_measurements = 9;
    else if(sweep_type.compare("shortest") == 0)
        sweep_measurements = 6;


    //initialize dynamic reconfiguration feedback
    dynamic_reconfigure::Server<topological_exploration::topological_explorationConfig> server;
    dynamic_reconfigure::Server<topological_exploration::topological_explorationConfig>::CallbackType dynSer;
    dynSer = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(dynSer);

    /*** subscribers ***/

    //to get the current node
    currentNodeSub = n.subscribe("/closest_node", 1, getCurrentNode);
    //to get the list of nodes
    mapSub = n.subscribe("/topological_map", 1, getTopologicalMap);
    //to determine if charging is required
    batterySub = n.subscribe("battery_state", 1, batteryCallBack);


    /*** publishers ***/

    //to visualize the 3D grid markers in rviz
    grid_markers_pub = n.advertise<visualization_msgs::Marker>("visualize_grid", 100);


    /*** services ***/

    //to visualize 3D grid
    ros::ServiceServer visualize_grid = n.advertiseService("view_grid", visualizeGrid);
    //to get relevant nodes
    nodeListClient = n.serviceClient<strands_navigation_msgs::GetTaggedNodes>("/topological_map_manager/get_tagged_nodes");
    //to create task objects
    taskAdder = n.serviceClient<strands_executive_msgs::AddTask>("/task_executor/add_task");
    //save grid
    save_service = n.serviceClient<strands_exploration_msgs::SaveGrid>("/topological_exploration/save_grid");
    //exploration routine
    ros::ServiceServer routine_service = n.advertiseService("/edge_exp_srv", explorationRoutine);


    /*** tf listener ***/

    //to convert depth image from optical frame to map frame
    tf_listener = new tf::TransformListener();
    ros::Time now = ros::Time(0);
    tf_listener->waitForTransform("/head_xtion_depth_optical_frame","/map",now, ros::Duration(10.0));

    /*** image transport ***/

    //to subscribe the depth image and add it to the grid (ray casting)
    image_transport::ImageTransport imageTransporter(n);
    image_transport::Subscriber image_subscriber = imageTransporter.subscribe("/local_metric_map/depth/depth_filtered", 50, imageCallback);
    ros::spinOnce();
    sleep(0.5);



    //get topological map nodes tagged as Exploration
    int num_critical_nodes = getRelevantNodes("ExplorationGround", &critical_nodes);
    int num_exploration_nodes = getRelevantNodes("Exploration", &exploration_nodes);

    if (num_critical_nodes < 0 || num_exploration_nodes < 0)
    {
        ROS_ERROR("Topological navigation does not report about tagged nodes. Is it running?");
        return -1;
    }

    //generate schedule
    ros::Time currentTime = ros::Time::now();
    generateSchedule(currentTime.sec);

    maxTaskNumber = 1;
    while (ros::ok())
    {
        ros::spinOnce();
        sleep(1);
        if (debug) ROS_INFO("Exploration tasks: %i %i",numCurrentTasks,maxTaskNumber);
        currentTimeSlot = getNextTimeSlot(0);
        if (currentTimeSlot!=lastTimeSlot){
            numCurrentTasks--;
            if (numCurrentTasks < 0) numCurrentTasks = 0;
        }
        if (numCurrentTasks < maxTaskNumber)
        {
            lastTimeSlot=currentTimeSlot;
            int a = getNextTimeSlot(numCurrentTasks);
            if (a >= 0){
                createTask(a);
                numCurrentTasks++;
            }
        }
    }

    return 0;

    delete tf_listener;
}

