#include "CFremenGrid.h"

using namespace std;

CFremenGrid::CFremenGrid(const char* name, float originX,float originY,float originZ,int dimX,int dimY,int dimZ,float cellSize)
{
    strcpy(id,name);
    lastTimeStamp = 0;
    obtainedInformationLast = 0;
    obtainedInformationPredicted = 0;
    minProb = 0.05;
    maxProb = 1-minProb;
    residualEntropy = minProb*log2f(minProb)+(maxProb)*log2f(maxProb);
    residualInformation = log2f(maxProb);
    debug = false;
    oX = originX;
    oY = originY;
    oZ = originZ;
    xDim = dimX;
    yDim = dimY;
    zDim = dimZ;
    resolution = cellSize;
    numCells = xDim*yDim*zDim;
    aux = (char*) malloc(numCells*sizeof(char));
    probs = (float*) malloc(numCells*sizeof(float));
    predicted = (float*) malloc(numCells*sizeof(float));
    frelements = (CFrelement*) malloc(numCells*sizeof(CFrelement));
    for (int i = 0;i<numCells;i++) probs[i] = 0.5;
    for (int i = 0;i<numCells;i++) frelements[i].init();
    buildLimits(probs);
    buildLimits(predicted);
    if (debug) printf("Float size: %i \n",(int)sizeof(double));
    lastPhiRange=lastPsiMin=lastPsiMax=lastRange=numRaycasters = 0;
    //cellArray = (CFrelement**) malloc(numCells*sizeof(CFrelement*));
    //for (int i=0;i<numCells;i++) cellArray[i] = new CFrelement();

}

CFremenGrid::~CFremenGrid()
{
    free(raycasters);
    free(aux);
    free(probs);
    free(predicted);
    free(frelements);
    //for (int i=0;i<numCells;i++) free(cellArray[i]);
    //free(cellArray);

}

int CFremenGrid::getIndex(float x,float  y,float  z)
{
    int iX,iY,iZ;
    iX = (int)((x-oX)/resolution);
    iY = (int)((y-oY)/resolution);
    iZ = (int)((z-oZ)/resolution);
    if (iX < xDim && iY < yDim && iZ < zDim && iX >= 0 && iY >=0 && iZ >= 0) return iX+xDim*(iY+yDim*iZ);
    return 0;
}

float CFremenGrid::getClosestObstacle(float xp,float yp,float zt,float range)
{
    float rangeLimit=(range/resolution);
    float minRange=rangeLimit*rangeLimit;
    int xStart = (int)((xp-oX-range)/resolution);
    int xEnd   = (int)((xp-oX+range)/resolution);
    int yStart = (int)((yp-oY-range)/resolution);
    int yEnd   = (int)((yp-oY+range)/resolution);
    int zStart = (int)((0.5-oZ)/resolution);
    int zEnd   = (int)((1.5-oZ)/resolution);
    int xM = (int)((xp-oX)/resolution);
    int yM = (int)((yp-oY)/resolution);
    xM = fmax(fmin(xM,xDim-1),0);
    yM = fmax(fmin(yM,yDim-1),0);
    xStart = fmax(fmin(xStart,xDim-1),0);
    xEnd   = fmax(fmin(xEnd,xDim-1),0);
    yStart = fmax(fmin(yStart,yDim-1),0);
    yEnd   = fmax(fmin(yEnd,yDim-1),0);
    zStart = fmax(fmin(zStart,zDim-1),0);
    zEnd   = fmax(fmin(zEnd,zDim-1),0);

    int cellIndex=0;
    for (int z = zStart;z<zEnd;z++)
    {
        for (int y = yStart;y<=yEnd;y++)
        {
            cellIndex = xDim*(y+yDim*z);
            for (int x = xStart;x<=xEnd;x++)
            {
                if (predicted[cellIndex+x] > 0.7 && ((x-xM)*(x-xM)+(y-yM)*(y-yM)<minRange))
                {
                    minRange = (x-xM)*(x-xM)+(y-yM)*(y-yM);
                }
            }
        }
    }
    return sqrt(minRange)*resolution;
}

float CFremenGrid::estimateInformation(float sx,float sy,float sz,float range,uint32_t timestamp)
{
    CTimer timer;
    timer.reset();
    timer.start();
    memset(aux,0,numCells*sizeof(char));
    int prepare  = 0;
    int calculate = 0;
    int preprocess = 0;
    float entropy = 0;
    float px = (floor(sx/resolution)+0.5);
    float py = (floor(sy/resolution)+0.5);
    float pz = (floor(sz/resolution)+0.5);
    float offX = oX/resolution;
    float offY = oY/resolution;
    float offZ = oZ/resolution;
    px -= offX;
    py -= offY;
    pz -= offZ;
    int startIndex =  (int)px+xDim*((int)py+yDim*((int)pz));
    int backupIndex = startIndex;
    float psiMax = 0.9;
    float psiMin = -0.6;
    float phiRange = M_PI;
    if (phiRange != lastPhiRange || psiMax != lastPsiMax || psiMin != lastPsiMin || range != lastRange)
    {
        if (numRaycasters > 0) free(raycasters);
        numRaycasters = 0;
        lastPhiRange = phiRange;
        lastPsiMax  = psiMax;
        lastPsiMin  = psiMin;
        lastRange = range;
    }
    //precalculate raycasting structures
    if (numRaycasters == 0)
    {
        px = floor(oX/resolution+xDim/2)+0.5;
        py = floor(oY/resolution+yDim/2)+0.5;
        pz = floor(oZ/resolution+zDim/2)+0.5;
        numRaycasters = 0;
        float *x = (float*)malloc(sizeof(float)*10000000);
        float *y = (float*)malloc(sizeof(float)*10000000);
        float *z = (float*)malloc(sizeof(float)*10000000);
        int size = 0;
        float phiStep;
        float granularity = resolution/range/5;
        float rx,ry,rz,ax,ay,az,bx,by,bz,cx,cy,cz;
        int ix,iy,iz,index,final,xStep,yStep,zStep;

        range/=resolution;
        for (float psi = psiMin;psi<=psiMax;psi+=granularity)
        {
            phiStep = granularity/cos(psi);
            for (float phi = -phiRange;phi<=phiRange;phi+=phiStep){
                x[size] = range*cos(phi)*cos(psi)+px;
                y[size] = range*sin(phi)*cos(psi)+py;
                z[size] = -range*sin(psi)+pz;
                size++;
            }
        }
        //printf("Number of rays: %i\n",size);

        unsigned char process[size];
        //rescale only one ray per cell, adjust ray direction to go to the ray centre
        memset(process,0,size);
        int unique = 0;

        for (int i = 0;i<size;i++){
            x[i] = fmin(fmax(floor(x[i]-offX),0)+0.5,xDim-1);
            y[i] = fmin(fmax(floor(y[i]-offY),0)+0.5,yDim-1);
            z[i] = fmin(fmax(floor(z[i]-offZ),0)+0.5,zDim-1);
            final = (int)x[i]+xDim*((int)y[i]+yDim*((int)z[i]));
            if (aux[final] != 1){
                aux[final] = 1;
                process[i] = 1;
                unique++;
            }
        }

        px -= offX;
        py -= offY;
        pz -= offZ;
        //raycast origin in float and int
        int i = 0;
        //calculate the point of origin
        startIndex =  (int)px+xDim*((int)py+yDim*((int)pz));
        prepare  = 0;
        calculate = 0;;
        preprocess = timer.getTime();
        numRaycasters = 0;
        for (int i = 0;i<size;i++) numRaycasters += process[i];

        //pre-calculate the grid
        int raycastSize = sizeof(int)*300*numRaycasters;
        printf("Rays total %i raycast memory size %i\n",numRaycasters,raycastSize);
        raycasters  = (int*)malloc(raycastSize);
        int raycastIndex = 0;
        for (int i = 0;i<size;i++)
        {
            timer.reset();
            if (process[i]){
                //calculate the ray vector
                rx = (x[i]-px);
                ry = (y[i]-py);
                rz = (z[i]-pz);

                //calculate the general direction of the ray
                ix = 1;
                iy = 1;
                iz = 1;
                if (rx < 0) ix = -1;
                if (ry < 0) iy = -1;
                if (rz < 0) iz = -1;
                if (fabs(rx) < 1.0/xDim) ix = 0;
                if (fabs(ry) < 1.0/yDim) iy = 0;
                if (fabs(rz) < 1.0/zDim) iz = 0;

                //establish increments when moving in the grid
                xStep = ix;
                yStep = iy*xDim;
                zStep = iz*xDim*yDim;

                //establish the first and last cell index
                index = startIndex;
                final = (int)x[i]+xDim*((int)y[i]+yDim*((int)z[i]));
                //initialize values of the expected intersections
                bx=by=bz=cx=cy=cz = xDim*yDim*zDim; //high enough to be outside of the grid

                //initialize first intersection planes - these are in the general direction of the ray vector
                ax = floor(px)+(ix+1)/2;
                ay = floor(py)+(iy+1)/2;
                az = floor(pz)+(iz+1)/2;

                //calculate intersections with the planes
                if (ix != 0) bx = (ax-px)/rx;
                if (iy != 0) by = (ay-py)/ry;
                if (iz != 0) bz = (az-pz)/rz;

                //calculate the position increments when intersecting the following planes
                if (ix != 0) cx = ix/rx;
                if (iy != 0) cy = iy/ry;
                if (iz != 0) cz = iz/rz;

                //if (debug) printf("Indices %i %i %.2f %.2f %.2f %.2f %.2f %.2f \n",index,final,bx,by,bz,cx,cy,cz);
                prepare += timer.getTime();
                timer.reset();
                bool free = true;
                int indexI = 0;
                // start the grid traversal process
                int raycastStart = raycastIndex++;
                for (int j=0;index!=final;j++)
                {
                    index = startIndex + indexI;
                    if (bx < by && bx < bz)
                    {
                        bx+=cx;
                        indexI+=xStep;
                    }
                    else if (by < bz)
                    {
                        by+=cy;
                        indexI+=yStep;
                    }else{
                        bz+=cz;
                        indexI+=zStep;
                    }
                    raycasters[raycastIndex++] = indexI;
                }
                raycastIndex--;
                raycasters[raycastStart] = raycastIndex-raycastStart;
                calculate += timer.getTime();
            }
        }
        free(x);
        free(y);
        free(z);
    }
    timer.reset();
    int rayIndex = 0;
    startIndex = backupIndex;
    int cellIndex = startIndex;
    bool cellFree=true;
    float prob;
    for (int i = 0;i<numRaycasters;i++){
        int castLength = raycasters[rayIndex]+rayIndex;
        cellFree=true;
        for (int j = rayIndex+1;j<castLength&&cellFree;j++)
        {
            cellIndex = startIndex+raycasters[j];
            prob = predicted[cellIndex];
            cellFree = prob < 0.7;
            if (aux[cellIndex] == 0){
                aux[cellIndex] = 1;
                entropy-=fmin(prob*log2f(prob)+(1-prob)*log2f(1-prob)-residualEntropy,0);
            }
        }
        //aux[cellIndex] = 2;
        rayIndex=castLength;
    }
    //printf("Entropy %.0f took %i ms to preprocess, %i ms to prepare, %i ms to raycast and %i to calculate.\n",entropy,preprocess,prepare,calculate,timer.getTime());
    return entropy;
}

//fast grid update
float CFremenGrid::incorporate(float *x,float *y,float *z,float *d,int size,uint32_t timestamp)
{
    CTimer timer;
    timer.reset();
    timer.start();
    float dumInf = 0;
    float px,py,pz,rx,ry,rz,ax,ay,az,bx,by,bz,cx,cy,cz;
    int startIndex,ix,iy,iz,index,final,xStep,yStep,zStep;
    unsigned char process[size];
    bool subsample = false;
    float maxRange = 4.0;

    memset(aux,0,numCells*sizeof(char));
    uint32_t times[1];
    unsigned char oneVal[1];
    unsigned char zeroVal[1];
    oneVal[0] = 1;
    zeroVal[0] = 0;
    times[0] = timestamp;
    int processed=0;
    //rescale ray intersections to match the grid
    float xi,yi,zi;
    int setToOne = 0;
    int setToZero = 0;
    int adIndex[] = {xDim*yDim-xDim-1,xDim*yDim-xDim+0,xDim*yDim-xDim+1,xDim*yDim-1,xDim*yDim+0,xDim*yDim+1,xDim*yDim+xDim-1,xDim*yDim+xDim+0,xDim*yDim+xDim+1,-xDim*yDim-xDim-1,-xDim*yDim-xDim+0,-xDim*yDim-xDim+1,-xDim*yDim-1,-xDim*yDim+0,-xDim*yDim+1,-xDim*yDim+xDim-1,-xDim*yDim+xDim+0,-xDim*yDim+xDim+1,-xDim-1,-xDim+0,-xDim+1,-1,+0,+1,+xDim-1,+xDim+0,+xDim+1};
    if (subsample == false){
        //rescale all
        for (int i = 0;i<size+1;i++)
        {
            //x[i] = (x[i]-oX)/resolution;
            //y[i] = (y[i]-oY)/resolution;
            //z[i] = (z[i]-oZ)/resolution;
            x[i] = fmin(fmax((x[i]-oX)/resolution,0.5),xDim-1);
            y[i] = fmin(fmax((y[i]-oY)/resolution,0.5),yDim-1);
            z[i] = fmin(fmax((z[i]-oZ)/resolution,0.5),zDim-1);
            //if (x[i] > 0.5 && x[i] < xDim-1 && y[i] > 0.5 && y[i] < yDim-1 && z[i] > 0.5 && z[i] < zDim -1)
            //{
            final = (int)x[i]+xDim*((int)y[i]+yDim*((int)z[i]));
            /*for (int b = 0;b<27;b++)
            {
                int c = final + adIndex[b];
                if (c>0&&c<numCells) aux[c] = 1;
            }*/
            //if (aux[final] != 1){
                aux[final] = 1;
                if (d[i]==1)
                {
                    dumInf = fmax(fmin(probs[final],maxProb),minProb);
                    obtainedInformationLast += -(log2f(dumInf)-residualInformation);
                    dumInf = fmax(fmin(predicted[final],maxProb),minProb);
                    obtainedInformationPredicted += -(log2f(dumInf)-residualInformation);
                    if (probs[final] < 0.5)setToOne++;
                    frelements[final].add(times,oneVal,1);
                    probs[final] = maxProb; //else probs[final] = minProb;
                }
                process[i] = 1;
                processed++;
            //}
            //}else{
            //	process[i] = 0;
            //}
        }
        //memset(process,1,size);
    }else{
        //rescale only one ray per cell, adjust ray direction to go to the ray centre
        memset(process,0,size);
        for (int i = 0;i<size+1;i++){
            x[i] = fmin(fmax(floor((x[i]-oX)/resolution),1)+0.5,xDim-1);
            y[i] = fmin(fmax(floor((y[i]-oY)/resolution),1)+0.5,yDim-1);
            z[i] = fmin(fmax(floor((z[i]-oZ)/resolution),1)+0.5,zDim-1);
            final = (int)x[i]+xDim*((int)y[i]+yDim*((int)z[i]));
            //if (aux[final] != 1){

                aux[final] = 1;
                if (d[i]==1)
                {
                    dumInf = fmax(fmin(probs[final],maxProb),minProb);
                    obtainedInformationLast += -(log2f(dumInf)-residualInformation);
                    dumInf = fmax(fmin(predicted[final],maxProb),minProb);
                    obtainedInformationPredicted += -(log2f(dumInf)-residualInformation);
                        frelements[final].add(times,oneVal,1);
                    if (probs[final] < 0.5) setToOne++;
                    probs[final] = maxProb;//(maxProb+probs[final]*3)/4; //else probs[final] = minProb;
                }
                process[i] = 1;
                processed++;
            //}
        }
    }
    //printf("rescaling from %i rays to %i rays took %i ms, ",size,processed,timer.getTime());
    //raycast origin in float and int
    int i = 0;
    //calculate the point of origin

    px = x[size];
    py = y[size];
    pz = z[size];

    //calculate the initial cell index
    startIndex =  (int)px+xDim*((int)py+yDim*((int)pz));
    int prepare  = 0;
    int calculate = 0;;
    int cells = 0;
    for (int i = 0;i<size;i++)
    {
        timer.reset();
        if (process[i]){
            //calculate the ray vector
            rx = (x[i]-px);
            ry = (y[i]-py);
            rz = (z[i]-pz);

            //calculate the general direction of the ray
            ix = 1;
            iy = 1;
            iz = 1;
            if (rx < 0) ix = -1;
            if (ry < 0) iy = -1;
            if (rz < 0) iz = -1;
            if (fabs(rx) < 1.0/xDim) ix = 0;
            if (fabs(ry) < 1.0/yDim) iy = 0;
            if (fabs(rz) < 1.0/zDim) iz = 0;

            //establish increments when moving in the grid
            xStep = ix;
            yStep = iy*xDim;
            zStep = iz*xDim*yDim;

            //establish the first and last cell index
            index = startIndex;
            final = (int)x[i]+xDim*((int)y[i]+yDim*((int)z[i]));
            //initialize values of the expected intersections
            bx=by=bz=cx=cy=cz = xDim*yDim*zDim; //high enough to be outside of the grid

            //initialize first intersection planes - these are in the general direction of the ray vector
            ax = floor(px+(ix+1)/2);
            ay = floor(py+(iy+1)/2);
            az = floor(pz+(iz+1)/2);

            //calculate intersections with the planes
            if (ix != 0) bx = (ax-px)/rx;
            if (iy != 0) by = (ay-py)/ry;
            if (iz != 0) bz = (az-pz)/rz;

            //calculate the position increments when intersecting the following planes
            if (ix != 0) cx = ix/rx;
            if (iy != 0) cy = iy/ry;
            if (iz != 0) cz = iz/rz;

            //if (debug) printf("Indices %i %i %.2f %.2f %.2f %.2f %.2f %.2f \n",index,final,bx,by,bz,cx,cy,cz);
            prepare += timer.getTime();
            timer.reset();
            int j = 0;
            // start the grid traversal process
            for (j=0;index!=final;j++)
            {
                //if (debug) printf("Index %06i %06i %06i %.2f %.2f %.2f %.2f\n",index,final,startIndex,bx,bx*rx+px,by*ry+py,bz*rz+pz);
                if (index < 0 || index >= numCells)
                {
                    printf("break!!\n");
                    break;
                }
                if (j > 400){
                    printf("Stuckup, %i %i %i Step: %i %i %i Increment: %.5f %.5f %.5f \n",startIndex,index,final,xStep,yStep,zStep,cx,cy,cz);
                    break;
                }
                if (aux[index] == 0){
                    aux[index] = 1;
                    dumInf = fmax(fmin(probs[index],maxProb),minProb);
                    obtainedInformationLast += -(log2f(1-dumInf)-residualInformation);
                    dumInf = fmax(fmin(predicted[index],maxProb),minProb);
                    obtainedInformationPredicted += -(log2f(1-dumInf)-residualInformation);
                    if (probs[index] > 0.5) setToZero++;
                    probs[index] = minProb;//(minProb+probs[index]*3)/4;
                    frelements[index].add(times,zeroVal,1);
                }
                if (bx < by && bx < bz)
                {
                    bx+=cx;
                    index+=xStep;
                }
                else if (by < bz)
                {
                    by+=cy;
                    index+=yStep;
                }else{
                    bz+=cz;
                    index+=zStep;
                }
            }
            cells+=j;
            calculate += timer.getTime();
        }
    }
    printf("Cells: set to 0: %i. Set to 1: %i.\n",setToZero,setToOne);
    //printf("preparation %i ms and update of %i cells took %i ms.\n",prepare,cells,calculate);
    return obtainedInformationLast;
}

void CFremenGrid::save(const char* filename,bool lossy,int forceOrder)
{
    FILE* f=fopen(filename,"w");
    fwrite(&xDim,sizeof(int),1,f);
    fwrite(&yDim,sizeof(int),1,f);
    fwrite(&zDim,sizeof(int),1,f);
    fwrite(&oX,sizeof(float),1,f);
    fwrite(&oY,sizeof(float),1,f);
    fwrite(&oZ,sizeof(float),1,f);
    fwrite(&resolution,sizeof(float),1,f);
    fwrite(&obtainedInformationLast,sizeof(float),1,f);
    fwrite(&obtainedInformationPredicted,sizeof(float),1,f);
    fwrite(probs,sizeof(float),numCells,f);
    for (int i=0;i<numCells;i++) frelements[i].save(f,false);
    fclose(f);
}

void CFremenGrid::saveSmart(const char* filename,bool lossy,int forceOrder)
{
    FILE* f=fopen(filename,"w");
    fwrite(&xDim,sizeof(int),1,f);
    fwrite(&yDim,sizeof(int),1,f);
    fwrite(&zDim,sizeof(int),1,f);
    fwrite(&oX,sizeof(float),1,f);
    fwrite(&oY,sizeof(float),1,f);
    fwrite(&oZ,sizeof(float),1,f);
    fwrite(&resolution,sizeof(float),1,f);
    fwrite(&obtainedInformationLast,sizeof(float),1,f);
    fwrite(&obtainedInformationPredicted,sizeof(float),1,f);
    fwrite(probs,sizeof(float),numCells,f);
    for (int i=0;i<numCells;i++) frelements[i].saveSmart(f,false);
    fclose(f);
}


bool CFremenGrid::load(const char* filename)
{
    int ret = 0;
    FILE* f=fopen(filename,"r");
    if (f == NULL){
        printf("FrOctomap %s not found, aborting load.\n",filename);
        return false;
    }
/*	printf("Loading FrOctomap %s.\n",filename);
    for (int i=0;i<numCells;i++){
         free(cellArray[i]);
    }
    free(cellArray);*/
    ret = fread(&xDim,sizeof(int),1,f);
    ret = fread(&yDim,sizeof(int),1,f);
    ret = fread(&zDim,sizeof(int),1,f);
    ret = fread(&oX,sizeof(float),1,f);
    ret = fread(&oY,sizeof(float),1,f);
    ret = fread(&oZ,sizeof(float),1,f);
    ret = fread(&resolution,sizeof(float),1,f);
    ret = fread(&obtainedInformationLast,sizeof(float),1,f);
    ret = fread(&obtainedInformationPredicted,sizeof(float),1,f);
    numCells = xDim*yDim*zDim;
    ret = fread(probs,sizeof(float),numCells,f);
    for (int i=0;i<numCells;i++) frelements[i].load(f);
    /*cellArray = (CFrelement**) malloc(numCells*sizeof(CFrelement*));
    for (int i=0;i<numCells;i++) cellArray[i] = new CFrelement();
    for (int i=0;i<numCells;i++){
        cellArray[i]->load(f);
        cellArray[i]->signalLength = signalLength;
    }*/
    printf("FrOctomap with %i: %ix%ix%i cells and %.0f/%.0f information.\n",numCells,xDim,yDim,zDim,obtainedInformationLast,obtainedInformationPredicted);
    fclose(f);
    update();
    buildLimits(probs);
    buildLimits(predicted);
    return true;
}

bool CFremenGrid::loadHeader(const char* filename)
{
    int ret = 0;
    FILE* f=fopen(filename,"r");
    if (f == NULL){
        printf("FrOctomap %s not found, aborting load.\n",filename);
        return false;
    }
    ret = fread(&xDim,sizeof(int),1,f);
    ret = fread(&yDim,sizeof(int),1,f);
    ret = fread(&zDim,sizeof(int),1,f);
    ret = fread(&oX,sizeof(float),1,f);
    ret = fread(&oY,sizeof(float),1,f);
    ret = fread(&oZ,sizeof(float),1,f);
    ret = fread(&resolution,sizeof(float),1,f);
    ret = fread(&obtainedInformationLast,sizeof(float),1,f);
    ret = fread(&obtainedInformationPredicted,sizeof(float),1,f);
    numCells = xDim*yDim*zDim;
    fclose(f);
    return true;
}

void CFremenGrid::print(bool verbose)
{
    for (int i = 0;i<numCells;i++)
    {
        if (frelements[i].periodicities > 0)
        {
            printf("Cell: %i %i ",i,frelements[i].periodicities);
            frelements[i].print(verbose);
        }
    }
}

void CFremenGrid::buildLimits(float* grid)
{
    ///floor
    int minCells = 0;
    int limCells = xDim*yDim;
    for (int x = minCells;x<limCells;x++) grid[x] = 1.0;

    ///ceiling
    minCells = xDim*yDim*(zDim-1);
    limCells = xDim*yDim*zDim;
    for (int x = minCells;x<limCells;x++) grid[x] = 1.0;

    ///front
    minCells = 0;
    limCells = yDim*xDim*zDim;
    for (int x = minCells;x<limCells;x+=xDim) grid[x] = 1.0;

    ///back
    limCells = yDim*xDim*zDim-xDim+1;
    minCells = xDim-1;
    for (int x = minCells;x<limCells;x+=xDim) grid[x] = 1.0;

    minCells = 0;
    limCells = xDim*yDim*(zDim-1);
    for (int x = minCells;x<limCells;x++){
        if (x%xDim == 0) x+=(yDim-1)*xDim;
        grid[x] = 1.0;
    }

    minCells = 1;
    limCells = xDim*yDim*(zDim-1);
    for (int x = minCells;x<limCells;x++){
        if (x%xDim == 0) x+=(yDim-1)*xDim;
        grid[x] = 1.0;
    }
}

bool CFremenGrid::recalculate(uint32_t timestamp)
{
    int sum = 0;
    if (timestamp == 0){
        for (int i =0;i<numCells;i++){
            predicted[i] = frelements[i].estimate(0,0);
            if (predicted[i] == 0.5) sum++;
        }
    } //memcpy(predicted,probs,numCells*sizeof(float));
    else if (lastTimeStamp !=timestamp)
    {
        for (int i =0;i<numCells;i++){
             predicted[i] = frelements[i].estimate(timestamp,1);
             if (predicted[i] == 0.5) sum++;
        }
        printf("Recalculating with timestamp %i %i %i %i\n",timestamp,lastTimeStamp,numCells,sum);
    }
    lastTimeStamp =timestamp;
    buildLimits(predicted);
}

void CFremenGrid::update()
{
    for (int i =0;i<numCells;i++) frelements[i].update();
}

float CFremenGrid::getObtainedInformationLast()
{
    return obtainedInformationLast;
}

float CFremenGrid::getObtainedInformationPredicted()
{
    return obtainedInformationPredicted;
}


float CFremenGrid::estimate(unsigned int index,uint32_t timeStamp)
{
    return predicted[index];
}

float CFremenGrid::retrieve(unsigned int index)
{
    return probs[index];
}


int CFremenGrid::numStatic(float tolerance)
{
    int number = 0;
    for (int i =0;i<numCells;i++)
    {
        if (frelements[i].gain <= tolerance || frelements[i].gain >= 1-tolerance || frelements[i].measurements == 0) number++;
    }
    return number;
}


float CFremenGrid::getDominant(unsigned int i,int period)
{
    if (frelements[i].frelements[0].period == period) return  frelements[i].frelements[0].amplitude;
    return 0;
}
