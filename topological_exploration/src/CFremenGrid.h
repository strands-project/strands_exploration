#ifndef CFREMENGRID_H
#define CFREMENGRID_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "CTimer.h"
#include "CFrelement.h"

#define MAX_ID_LENGTH 100

/**
@author Tom Krajnik
*/

using namespace std;

class CFremenGrid
{
    public:
        CFremenGrid(const char* name, float originX,float originY,float originZ,int dimX,int dimY,int dimZ,float cellSize);
        ~CFremenGrid();

        /*fills the grid borders*/
        void buildLimits(float* grid);

        /*state estimation: estimates the grid for the given time*/
        bool recalculate(uint32_t timeStamp);

        /*repopulates the set P - use to recover after map load*/
        void update();

        float getObtainedInformationLast();
        float getObtainedInformationPredicted();

        float estimateInformation(float x,float y,float z,float range,uint32_t t);
        float getClosestObstacle(float x,float y,float zlimit,float range);

        /*changes the model order*/
        void print(bool verbose);
        void save(const char*name,bool lossy = false,int forceOrder = -1);
        void saveSmart(const char*name,bool lossy = false,int forceOrder = -1);
        bool load(const char*name);
        bool loadHeader(const char*name);
        int numStatic(float tolerance = 0);

        float estimate(unsigned int index,uint32_t timeStamp);
        float retrieve(unsigned int index);
        float getDominant(unsigned int index,int period);

        float incorporate(float *x,float *y,float *z,float *d,int size,uint32_t t);
        int getIndex(float x,float y,float z);

        //center of the first cell
        float oX,oY,oZ;
        //size of the grid cells
        float resolution;
        //grid dimensions
        int xDim,yDim,zDim;
        float* probs;
        float* predicted;
        char *aux;
        int numCells;
        bool debug;

        char id[MAX_ID_LENGTH];
        uint32_t lastTimeStamp;
        float minProb,maxProb,residualEntropy,residualInformation;
        float lastPhiRange,lastPsiMin,lastPsiMax,lastRange;
        int *raycasters;
        int numRaycasters;
        float obtainedInformationLast;
        float obtainedInformationPredicted;
        CFrelement* frelements;
};

#endif
