#include <stdlib.h>
#include <sstream>
#include <cassert>
#include <time.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <spatiotemporal_exploration/spatiotemporal_explorationConfig.h>
#include <strands_navigation_msgs/NavStatistics.h>
#include <strands_navigation_msgs/TopologicalMap.h>
#include <strands_navigation_msgs/TopologicalNode.h>
#include <strands_navigation_msgs/GetTaggedNodes.h>
#include <strands_executive_msgs/AddTask.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <scitos_msgs/BatteryState.h>
#include <scitos_ptu/PanTiltActionFeedback.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <mongodb_store/message_store.h>
#include <mongodb_store_msgs/StringPair.h>
#include <strands_exploration_msgs/FremenGrid.h>
#include <strands_exploration_msgs/Frelement.h>
#include <strands_exploration_msgs/SFrelement.h>
#include <strands_exploration_msgs/LoadGrid.h>
#include <strands_exploration_msgs/SaveGrid.h>
#include <strands_exploration_msgs/AddView.h>
#include <strands_exploration_msgs/Visualize.h>
#include <strands_exploration_msgs/GetExplorationTasks.h>
#include <strands_exploration_msgs/ExplorationSchedule.h>
#include "CFremenGridSet.h"


using namespace std;

//scheduler parameters
int taskDuration = 1200;
int rescheduleInterval = 86400;

//3D grid parameters
double cellSize = 0.1;
int dimX = 100;
int dimY = 100;
int dimZ = 40;
float camera_range = 4.0;

//standard parameters
string collectionName;
string scheduleDirectory;
string gridsDirectory;
string sweep_type;
string exploration_tag;

//runtine parameters
float explorationRatio = 1.0;
int maxTaskNumber = 1;
int taskPriority = 50;
bool debug = true;
int taskStartDelay = 5;
int rescheduleCheckTime = 5;

//ROS communication
ros::Subscriber currentNodeSub;
ros::Subscriber mapSub;
ros::Publisher  grid_markers_pub;
ros::Publisher  schedule_pub;
ros::ServiceClient nodeListClient;
ros::ServiceClient load_service;
ros::ServiceClient save_service;


//Fremen grid component
CFremenGridSet *fremengridSet;

strands_navigation_msgs::TopologicalMap topoMap;

//state variables
int lastTimeSlot = -1;
int currentTimeSlot = -1;
int numCurrentTasks = 0; 
string nodeName = "ChargingPoint";
string closestNode = "ChargingPoint";
int timeOffset = 0;

//times and nodes of schedule
uint32_t timeSlots[10000];
int nodes[10000];
float node_entropies[1000];
float max_entropy;
int taskIDs[10000];
int numNodes = 0;


//Ray casting parameters
int integrateMeasurements = 0;
int maxMeasurements = 1;
int measurements = maxMeasurements;
bool first_grid = true;
unsigned int timestamp;
int gridIndex;
unsigned int sweep_measurements;
unsigned int current_measurement;

std::vector<std::string> exploration_nodes;
string currentNode;

tf::TransformListener *tf_listener;

uint32_t getMidnightTime(uint32_t givenTime)
{
    return ((givenTime+timeOffset)/rescheduleInterval)*rescheduleInterval-timeOffset;
}

/*parameter reconfiguration*/
void reconfigureCallback(spatiotemporal_exploration::spatiotemporal_explorationConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %lf %d", config.explorationRatio, config.maxTaskNumber);
    explorationRatio = config.explorationRatio;
    maxTaskNumber = config.maxTaskNumber;
    taskPriority = config.taskPriority;
    debug = config.verbose;
    taskStartDelay = config.taskStartDelay;
    rescheduleCheckTime = config.rescheduleCheckTime;
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

/*publishes 3d grid markers */
int publishGrid(string waypoint, unsigned int stamp, float minP, float maxP, float period, unsigned int type, string name, bool set_color,  std_msgs::ColorRGBA m_color)
{
    //init visualization markers:
    visualization_msgs::Marker markers;
    geometry_msgs::Point cubeCenter;

    //get grid ID
    int id = fremengridSet->find(waypoint.c_str());

    //set type, frame and size
    markers.header.frame_id = "/map";
    markers.header.stamp = ros::Time::now();
    markers.ns = name;
    markers.action = visualization_msgs::Marker::ADD;
    markers.type = visualization_msgs::Marker::CUBE_LIST;
    markers.scale.x = fremengridSet->fremengrid[id]->resolution;
    markers.scale.y = fremengridSet->fremengrid[id]->resolution;
    markers.scale.z = fremengridSet->fremengrid[id]->resolution;
    markers.color = m_color;
    markers.points.clear();

    // prepare to iterate over the entire grid
    float resolution = fremengridSet->fremengrid[id]->resolution;
    float minX = fremengridSet->fremengrid[id]->oX;
    float minY = fremengridSet->fremengrid[id]->oY;
    float minZ = fremengridSet->fremengrid[id]->oZ;
    float maxX = minX+fremengridSet->fremengrid[id]->xDim*resolution-3*resolution/4;
    float maxY = minY+fremengridSet->fremengrid[id]->yDim*resolution-3*resolution/4;
    float maxZ = 4.0;
    int cnt = 0;
    int cells = 0;
    float estimate;

    if (stamp != 0 && type == 1) fremengridSet->fremengrid[id]->recalculate(stamp);

    //iterate over the cells' probabilities
    for(float z = minZ;z<maxZ;z+=resolution){
        for(float y = minY;y<maxY;y+=resolution){
            for(float x = minX;x<maxX;x+=resolution){
                if (type == 0) estimate = fremengridSet->fremengrid[id]->retrieve(cnt);          	//short-term memory grid
                if (type == 1) estimate = fremengridSet->fremengrid[id]->estimate(cnt,0);			//long-term memory grid
                if (type == 2) estimate = fremengridSet->fremengrid[id]->aux[cnt];               	//auxiliary grid
                if (type == 3) estimate = fremengridSet->fremengrid[id]->getDominant(cnt,period);	//dominant frequency amplitude

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

/* visualise 3d grids on demand*/
bool visualizeGrid(strands_exploration_msgs::Visualize::Request  &req, strands_exploration_msgs::Visualize::Response &res)
{
    int numvoxels = publishGrid(req.waypoint, req.stamp, req.minProbability, req.maxProbability, req.period, req.type, req.name, req.set_color, req.color);
    ROS_INFO("%d voxels published.", numvoxels);
    res.numcells = numvoxels;
    return true;
}

/*updates the closest node*/
void getCurrentNode(const std_msgs::String::ConstPtr& msg)
{
    closestNode = msg->data;

    if (fremengridSet->find(msg->data.c_str())>-1){
        nodeName = msg->data;
        ROS_INFO("Closest Exploration node switched to %s.",nodeName.c_str());
        if (debug) ROS_INFO("Closest Exploration node switched to %s.",nodeName.c_str());
    }else{
        if (debug) ROS_INFO("Closest node %s - however, it's not an Exploration node.",msg->data.c_str());
    }
}

/*loads relevant nodes from the map description*/
int getRelevantNodes(std::vector<std::string>* exp_nodes)
{
    int result = -1;

    strands_navigation_msgs::GetTaggedNodes srv;
    srv.request.tag = exploration_tag;
    if (nodeListClient.call(srv))
    {
        for (int i = 0;i<srv.response.nodes.size();i++)
        {
            exp_nodes->push_back(srv.response.nodes[i]);
            geometry_msgs::Point node_coordinates, grid_origin;
            coordinateSearch(srv.response.nodes[i], &node_coordinates);

            //grid origin calculation
            grid_origin.x = node_coordinates.x - (dimX*cellSize)/2;
            grid_origin.y = node_coordinates.y - (dimY*cellSize)/2;
            grid_origin.z = -0.05;

            fremengridSet->add(srv.response.nodes[i].c_str(), grid_origin.x, grid_origin.y, grid_origin.z, dimX, dimY, dimZ, cellSize);
        }

        numNodes = fremengridSet->numFremenGrids;
        ROS_INFO("Found %d nodes with tag %s", (int) srv.response.nodes.size(), exploration_tag.c_str());
        for (int i= numNodes - srv.response.nodes.size();i<numNodes;i++) ROS_INFO("FreMEnGrid ID: %i Label: %s.",i,fremengridSet->fremengrid[i]->id);
        result = srv.response.nodes.size();
    }
    else
    {
        ROS_ERROR("Failed to obtain exploration nodes with tag %s.", exploration_tag.c_str());
    }
    return result;
}

/*retrieve grids from the database, updates the grids and estimates information gain*/
void retrieveGrids(void)
{

    char filename[1000];
    char grid_filename[10000];
    char nodeName[1000];

    for(int i = 0; i < fremengridSet->numFremenGrids; i++)
    {
        FILE* file = fopen(filename,"r");
        sprintf(filename,"%s/%s.txt", gridsDirectory.c_str(), fremengridSet->fremengrid[i]->id);
        if(file != NULL)
        {
            ROS_INFO("Last grid for waypoint %s given by %s",fremengridSet->fremengrid[i]->id,filename);
            int check = fscanf(file,"%s %s\n", nodeName, grid_filename);
            ROS_INFO("Loading grid %s from %s", nodeName,grid_filename);
            fremengridSet->fremengrid[i]->load(grid_filename);
        }
        else
        {
            ROS_INFO("Grid not found, creating new grid!");
        }
    }
}

/*generates a schedule and saves it in a file*/
int generateNewSchedule(uint32_t givenTime)//TODO -> save schedule in MongoDB
{
    /*establish relevant time frame*/
    int numSlots = 24*3600/taskDuration;
    uint32_t timeSlots[numSlots];
    uint32_t midnight = getMidnightTime(givenTime);
    //retrieveGrids();

    /*create timeslots*/
    for (int i = 0;i<numSlots;i++) timeSlots[i] = midnight+3600*24/numSlots*i;
    numNodes = fremengridSet->numFremenGrids;

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
            coordinateSearch(fremengridSet->fremengrid[i]->id, &observationPoint);
            fremengridSet->recalculate(fremengridSet->fremengrid[i]->id, times[0]);
            slot_entropies[i] = entropy[0] = fremengridSet->estimateEntropy(fremengridSet->fremengrid[i]->id, observationPoint.x, observationPoint.y, 1.662, camera_range, times[0]);

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

    /*save a file with a schedule TODO - save to mongo*/
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
        fprintf(file,"%ld %s %s %f\n",timeInfo,dummy,fremengridSet->fremengrid[nodes[s]]->id, node_entropies[s]);
        ROS_INFO("Schedule: %ld %s %s %f\n",timeInfo,dummy,fremengridSet->fremengrid[nodes[s]]->id, node_entropies[s]);
    }
    fclose(file);
}

int processSchedule(uint32_t givenTime)
{
    strands_exploration_msgs::ExplorationSchedule schedule_msg;
    char dummy[1000];
    int numSlots = 24*3600/taskDuration;
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
            nodes[s] = fremengridSet->find(nodeName);
            if (nodes[s] < 0)
                ROS_ERROR("Exploration schedule file %s is corrupt at line %i (node %s is not tagged as exploration in topoogical map)!",dummy,s,nodeName);
            else
            {
                //fill schedule message
                schedule_msg.timeInfo.push_back(timeSlots[s]);
                schedule_msg.nodeID.push_back(fremengridSet->fremengrid[nodes[s]]->id);
                schedule_msg.entropy.push_back(node_entropies[s]);
            }
        }        
    }

    //publishes schedule
    schedule_pub.publish(schedule_msg);

    fclose(file);
}

int getNextTimeSlot(int lookAhead)
{
    char dummy[1000];
    char testTime[1000];
    time_t timeInfo;
    int numSlots = 24*3600/taskDuration;
    ros::Time currentTime = ros::Time::now();
    uint32_t givenTime = currentTime.sec+lookAhead*taskDuration+rescheduleCheckTime;
    uint32_t midnight = getMidnightTime(givenTime);
    if (timeSlots[0] != midnight)
    {
        ROS_INFO("Generating new schedule!");
        processSchedule(givenTime);
    }
    int currentSlot = (givenTime-timeSlots[0])/taskDuration;
    //ROS_INFO("Time %i - slot %i: going to node %i(%s).",currentTime.sec-midnight,currentSlot,nodes[currentSlot],fremengridSet->fremengrid[nodes[currentSlot]]->id);
    if (debug)
    {
        timeInfo = givenTime;
        strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
        sprintf(dummy,"time %i %s - slot %i: going to node %i(%s).",givenTime,testTime,currentSlot,nodes[currentSlot],fremengridSet->fremengrid[nodes[currentSlot]]->id);
        //ROS_INFO("%s",dummy);
    }
    if (currentSlot >= 0 && currentSlot < numSlots) return currentSlot;
    ROS_ERROR("Exploration schedule error: attempting to get task in a non-existent time slot.");
    return -1;
}

/*saves grid*/
int saveGrid(string name)
{
    int gridIndex = fremengridSet->find(name.c_str());

    if(gridIndex < 0)
    {
        ROS_ERROR("%s grid not found!", name.c_str());
        return -1;
    }
    else
    {

        time_t timeNow;
        time(&timeNow);
        char timeStr[100];
        char fileName[1000];
        char fileNameee[1000];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H:%M",localtime(&timeNow));
        sprintf(fileName,"%s/%s-%s.3dmap", gridsDirectory.c_str(), name.c_str(),timeStr);
        //ROS_INFO("%s", fileName);
        fremengridSet->fremengrid[gridIndex]->saveSmart(fileName, false, 0);

        sprintf(fileNameee,"%s/%s.txt", gridsDirectory.c_str(), name.c_str());
        FILE* file = fopen(fileNameee,"w+");
        if (file == NULL)
            ROS_ERROR("Could not open waypoint file %s.",fileNameee);

        fprintf(file,"%s %s\n",name.c_str(), fileName);
        fclose(file);
    }
}

int removeGrid(const char *name)
{
    if(fremengridSet->remove(name) < 0)
        ROS_ERROR("Error removing grid. %s grid not found!", name);
    else
        ROS_INFO("Removed %s grid!", name);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    if(current_measurement == 0)
    {
        //get grid index according to the waypoint
        gridIndex = fremengridSet->find(nodeName.c_str());
        ROS_INFO("Waypoint %s and grid index %i\n", nodeName.c_str(), gridIndex);
    }


    timestamp = msg->header.stamp.sec;

    ROS_INFO("Add depth called at %s and grid index %i.", nodeName.c_str(), gridIndex);

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

        int lastInfo = fremengridSet->fremengrid[gridIndex]->obtainedInformationLast;
        ROS_INFO("Depth image added to the grid (%d/%d)", current_measurement, sweep_measurements - 1);
        fremengridSet->fremengrid[gridIndex]->incorporate(x,y,z,d,len,timestamp);
        int newInfo = fremengridSet->fremengrid[gridIndex]->obtainedInformationLast;
        ROS_INFO("last: %d new: %d", lastInfo, newInfo);
    }



    if(current_measurement == sweep_measurements-1)
    {
        std_msgs::ColorRGBA color_aux;
        color_aux.g = color_aux.a = 1.0;
        color_aux.r = color_aux.b = 0.0;
        ROS_INFO("Sweep complete. Publishing and saving 3D grid...");
        int numvoxels = publishGrid(nodeName.c_str(), 0, 0.9, 1.0, 0, 0, nodeName.c_str(), false, color_aux);
        ROS_INFO("%d voxels published.", numvoxels);
        saveGrid(nodeName);


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

    //load parameters
    n.param<std::string>("sweep_type", sweep_type, "complete");
    n.param<std::string>("schedule_directory", scheduleDirectory, "/localhome/strands/schedules");
    n.param<std::string>("grids_directory", gridsDirectory, "/localhome/strands/3dmaps");
    n.param<std::string>("exploration_tag", exploration_tag, "Exploration");
    n.param("verbose", debug,false);
    n.param("resolution", cellSize, 0.1);
    n.param("dimX", dimX, 100);
    n.param("dimY", dimY, 100);
    n.param("dimZ", dimZ, 40);
    n.param("taskDuration", taskDuration, 1200);

    if(sweep_type.compare("complete") == 0)
        sweep_measurements = 51;
    else if(sweep_type.compare("medium") == 0)
        sweep_measurements = 17;
    else if(sweep_type.compare("short") == 0)
        sweep_measurements = 9;
    else if(sweep_type.compare("shortest") == 0)
        sweep_measurements = 6;

    fremengridSet = new CFremenGridSet();

    //initialize dynamic reconfiguration feedback
    dynamic_reconfigure::Server<spatiotemporal_exploration::spatiotemporal_explorationConfig> server;
    dynamic_reconfigure::Server<spatiotemporal_exploration::spatiotemporal_explorationConfig>::CallbackType dynSer;
    dynSer = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(dynSer);

    /*** SUBSCRIBERS ***/
    //to get the list of nodes
    mapSub = n.subscribe("/topological_map", 1, getTopologicalMap);
    //to get the current node
    currentNodeSub = n.subscribe("/closest_node", 1, getCurrentNode);



    /*** PUBLISHERS ***/
    //to visualize the 3D grid markers in rviz
    grid_markers_pub = n.advertise<visualization_msgs::Marker>("/visualize_grid", 100);
    //scheduler publisher (for bidder only)
    schedule_pub = n.advertise<strands_exploration_msgs::ExplorationSchedule>("/exploration_schedule", 100);


    /*** SERVICES ***/
    //3d grid visualisation
    ros::ServiceServer visualize_grid = n.advertiseService("view_grid", visualizeGrid);
    //list of relevant nodes (tagged nodes)
    nodeListClient = n.serviceClient<strands_navigation_msgs::GetTaggedNodes>("/topological_map_manager/get_tagged_nodes");



    /*** TF ***/
    //to convert depth image from optical frame to map frame
    tf_listener = new tf::TransformListener();
    ros::Time now = ros::Time(0);
    tf_listener->waitForTransform("/head_xtion_depth_optical_frame","/map",now, ros::Duration(15.0));

    /*** IMAGE TRANSPORT ***/
    //to subscribe the depth image and add it to the grid (ray casting)
    image_transport::ImageTransport imageTransporter(n);
    image_transport::Subscriber image_subscriber = imageTransporter.subscribe("/local_metric_map/depth/depth_filtered", 50, imageCallback);
    ros::spinOnce();


    //get number of exploration nodes
    int num_nodes = getRelevantNodes(&exploration_nodes);

    if(num_nodes == 0)
    {
        ROS_ERROR("Topological navigation does not report any nodes with %s tag.", exploration_tag.c_str());
        return -1;
    }

    //initialize grids
    retrieveGrids();

    //generate schedule
    ros::Time currentTime = ros::Time::now();
    processSchedule(currentTime.sec);

    ros::spinOnce();
    maxTaskNumber = 1;

    ros::Rate r(4);

    while (ros::ok())
    {
        ros::spinOnce();
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
                numCurrentTasks++;
            }
        }

        r.sleep();
    }

    delete fremengridSet;
    delete tf_listener;

    return 0;

}

