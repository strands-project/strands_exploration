#include "CFrelement.h"

using namespace std;
static bool debug = false;

int fremenSort(const void* i,const void* j)
{
     if (((SFrelement*)i)->amplitude < ((SFrelement*)j)->amplitude) return +1;
     return -1;
}

CFrelement::CFrelement()
{
    //initialization of the frequency set
    for (int i=0;i<MAX_ADAPTIVE_ORDER;i++){
            frelements[i].amplitude = frelements[i].phase = 0;
        frelements[i].period = 1;
    }
    //for (int i=0;i<NUM_PERIODICITIES;i++) frelements[i].period = (24*3600)/(i+1);
    gain = 0.5;
    order = 0;
    periodicities = 0;
    firstTime = 1;
    lastTime = 1;
    measurements = 0;
}

void CFrelement::init()
{
    for (int i=0;i<MAX_ADAPTIVE_ORDER;i++){
            frelements[i].amplitude = frelements[i].phase = 0;
        frelements[i].period = 1;
    }
    gain = 0.5;
    order = 0;
    periodicities = 0;
    firstTime = 1;
    lastTime = 1;
    measurements = 0;
    frk = NUM_PERIODICITIES;
}

CFrelement::~CFrelement()
{
}

// adds new state observations at given times
int CFrelement::add(uint32_t times[],unsigned char states[],int length)
{
    if (measurements == 0 && length > 0)
    {
        for (int i = 0;i<NUM_PERIODICITIES;i++){
            allFrelements[i].realStates = 0;
            allFrelements[i].imagStates = 0;
        }
        firstTime = times[0];
    }
    if (length == 0) return 0;
    int duration = times[length-1]-firstTime;
    int firstIndex = 0;

    //discard already known observations
    for (int i=0;i<length;i++)if (times[i] <= lastTime)firstIndex++;
    int numUpdated = length-firstIndex;

    //verify if there is an actual update
    //printf("Update %i %i %i\n",numUpdated,times[0],lastTime);
    if (numUpdated <= 0)return numUpdated;

    //remember the latest measurement
    lastTime = times[length-1];
    lastMeasurement = states[length-1];

    SFrelement tmp[NUM_PERIODICITIES];
    for (int i=0;i<NUM_PERIODICITIES;i++) tmp[i].period = (24*3600)/(i+1);

    //update the gains accordingly
    float oldGain=0;
    float newGain=0;
    for (int j = firstIndex;j<length;j++)newGain+=states[j];
    gain = (gain*measurements+newGain)/(measurements+length);

    //recalculate spectral balance - this is beneficial is the process period does not match the length of the data
    if (oldGain > 0){
        for (int i = 0;i<NUM_PERIODICITIES;i++)
        {
            allFrelements[i].realBalance  = gain*allFrelements[i].realBalance/oldGain;
            allFrelements[i].imagBalance  = gain*allFrelements[i].imagBalance/oldGain;
        }
    }else{
        for (int i = 0;i<NUM_PERIODICITIES;i++)
        {
            allFrelements[i].realBalance  = 0;
            allFrelements[i].imagBalance  = 0;
        }
    }


    float angle = 0;
    //recalculate the spectral components
    for (int j = firstIndex;j<length;j++)
    {
        for (int i = 0;i<NUM_PERIODICITIES;i++)
        {
            angle = 2*M_PI*(float)times[j]/tmp[i].period;
            allFrelements[i].realStates   += states[j]*cos(angle);
            allFrelements[i].imagStates   += states[j]*sin(angle);
            allFrelements[i].realBalance  += gain*cos(angle);
            allFrelements[i].imagBalance  += gain*sin(angle);
        }
    }
    measurements+=length;

    //establish amplitudes and phase shifts
    float re,im;
    for (int i = 0;i<NUM_PERIODICITIES;i++)
    {
        re = allFrelements[i].realStates-allFrelements[i].realBalance;
        im = allFrelements[i].imagStates-allFrelements[i].imagBalance;
        if (1.5*tmp[i].period <= duration && tmp[i].period>3600) tmp[i].amplitude = sqrt(re*re+im*im)/measurements; else tmp[i].amplitude = 0;
        if (tmp[i].amplitude < FREMEN_AMPLITUDE_THRESHOLD) tmp[i].amplitude = 0;
        //frelements[i].amplitude = sqrt(re*re+im*im)/measurements;
        tmp[i].phase = atan2(im,re);
    }

    //sort the spectral components
    qsort(tmp,NUM_PERIODICITIES,sizeof(SFrelement),fremenSort);

    //put them in the set P
    memcpy(frelements,tmp,MAX_ADAPTIVE_ORDER*sizeof(SFrelement));
    return numUpdated;
}

int CFrelement::evaluate(uint32_t* times,unsigned char* signal,int length,int orderi,float* evals)
{
    float estimate = 0;
    float time;
    unsigned char state;
    for (int j = 0;j<=orderi;j++) evals[j] = 0;
    for (int j = 0;j<length;j++)
    {
        time = times[j];
        state = signal[j];
        estimate = gain;
        evals[0]+= abs(state-(estimate>0.5));
        for (int i = 0;i<orderi;i++){
             estimate+=2*frelements[i].amplitude*cos(time/frelements[i].period*2*M_PI-frelements[i].phase);
             evals[i+1]+= abs(state-(estimate>0.5));
        }
    }
    for (int j = 0;j<=order+1;j++) evals[j]=evals[j]/length;

    //get best model order
    float error = 10.0;
    int index = 0;
    for (int j = 0;j<=orderi;j++)
    {
        if (evals[j] < error-0.001){
            index = j;
            error = evals[j];
        }
    }
    return index;
}


/*required only after loading*/
void CFrelement::update()
{
    //establish amplitudes and phase shifts
    float re,im;
    int duration = lastTime-firstTime;
    SFrelement tmp[NUM_PERIODICITIES];
    for (int i=0;i<NUM_PERIODICITIES;i++) tmp[i].period = (24*3600)/(i+1);
    periodicities = 0;
    for (int i = 0;i<NUM_PERIODICITIES;i++)
    {
        re = allFrelements[i].realStates-allFrelements[i].realBalance;
        im = allFrelements[i].imagStates-allFrelements[i].imagBalance;
        if (1.5*tmp[i].period <= duration && tmp[i].period>3600) tmp[i].amplitude = sqrt(re*re+im*im)/measurements; else tmp[i].amplitude = 0; 	//TODO zero!
        if (tmp[i].amplitude < FREMEN_AMPLITUDE_THRESHOLD) tmp[i].amplitude = 0;
        //frelements[i].amplitude = sqrt(re*re+im*im)/measurements;
        tmp[i].phase = atan2(im,re);
    }

    //sort the spectral components
    qsort(tmp,NUM_PERIODICITIES,sizeof(SFrelement),fremenSort);

    //put them in the set P
    memcpy(frelements,tmp,MAX_ADAPTIVE_ORDER*sizeof(SFrelement));
    for (int i = 0;i<MAX_ADAPTIVE_ORDER;i++){
        if (frelements[i].amplitude >=FREMEN_AMPLITUDE_THRESHOLD)periodicities++;
    }
    return;
}

/*text representation of the fremen model*/
void CFrelement::print(bool verbose)
{
    int errs = 0;
    std::cout << "Model prior: " << gain << " Size: " << measurements << " ";
    if (periodicities > 0) std::cout  << endl;
    if (verbose){
        float ampl = gain;
        for (int i = 0;i<min(periodicities,1);i++)
        {
            std::cout << "Frelement " << i << " " << frelements[i].amplitude << " " << frelements[i].phase << " " << frelements[i].period << endl;
        }
    }
    std::cout << endl;
}


float CFrelement::estimate(uint32_t time,int orderi)
{
    float saturation = 0.05;
    float estimate = gain;
    for (int i = 0;i<orderi;i++) estimate+=2*frelements[i].amplitude*cos(time/frelements[i].period*2*M_PI-frelements[i].phase);
    //float decayConstant = expf(-((float)(lastTime-time))/3600);
    //estimate = lastMeasurement*decayConstant+(1-decayConstant)*estimate;
    if (estimate > 1.0-saturation) estimate =  1.0-saturation;
    if (estimate < 0.0+saturation) estimate =  0.0+saturation;
    return estimate;
}

int CFrelement::estimate(uint32_t times[],float probs[],int length,int orderi)
{
    float estimate = 0;
    float time;
    float saturation = 0.05;
    for (int j = 0;j<length;j++)
    {
        time = times[j];
        estimate = gain;
        for (int i = 0;i<orderi;i++) estimate+=2*frelements[i].amplitude*cos(time/frelements[i].period*2*M_PI-frelements[i].phase);
        if (estimate > 1.0-saturation) estimate =  1.0-saturation;
        if (estimate < 0.0+saturation) estimate =  0.0+saturation;
        probs[j]=estimate;
    }
    return length;
}

int CFrelement::estimateEntropy(uint32_t times[],float entropy[],int length,int orderi)
{
    float estimate = 0;
    float time;
    float saturation = 0.05;
    for (int j = 0;j<length;j++)
    {
        time = times[j];
        estimate = gain;
        for (int i = 0;i<orderi;i++) estimate+=2*frelements[i].amplitude*cos(time/frelements[i].period*2*M_PI-frelements[i].phase);
        if (estimate > 1.0-saturation) estimate =  1.0-saturation;
        if (estimate < 0.0+saturation) estimate =  0.0+saturation;
        if (estimate <= 0 || estimate >= 1) entropy[j] = 0; else  entropy[j] = -(estimate*log2f(estimate)+(1-estimate)*log2f((1-estimate)));
    }
    return length;
}

int CFrelement::save(char* name,bool lossy)
{
    FILE* file = fopen(name,"w");
    save(file);
    fclose(file);
}

int CFrelement::saveSmart(char* name,bool lossy)
{
    FILE* file = fopen(name,"w");
    save(file);
    fclose(file);
}

int CFrelement::load(char* name)
{
    FILE* file = fopen(name,"r");
    load(file);
    fclose(file);
    return 0;
}


int CFrelement::save(FILE* file,bool lossy)
{
    //int frk = NUM_PERIODICITIES;
    fwrite(&frk,sizeof(uint32_t),1,file);
    fwrite(&gain,sizeof(float),1,file);
    fwrite(&measurements,sizeof(int),1,file);
    fwrite(&firstTime,sizeof(uint32_t),1,file);
    fwrite(&lastTime,sizeof(uint32_t),1,file);
    fwrite(allFrelements,sizeof(SFrelementFull),NUM_PERIODICITIES,file);
    return 0;
}

int CFrelement::saveSmart(FILE* file,bool lossy)
{
    //int frk = NUM_PERIODICITIES;
    if (measurements == 0 || gain == 0) frk = 0;
    fwrite(&frk,sizeof(uint32_t),1,file);
    fwrite(&gain,sizeof(float),1,file);
    fwrite(&measurements,sizeof(int),1,file);
    fwrite(&firstTime,sizeof(uint32_t),1,file);
    fwrite(&lastTime,sizeof(uint32_t),1,file);
    if (frk > 0) fwrite(allFrelements,sizeof(SFrelementFull),NUM_PERIODICITIES,file);
    return 0;
}

int CFrelement::load(FILE* file)
{
    //int frk = NUM_PERIODICITIES;
    int koko = 0;
    koko+=fread(&frk,sizeof(uint32_t),1,file);
    koko+=fread(&gain,sizeof(float),1,file);
    koko+=fread(&measurements,sizeof(int),1,file);
    koko+=fread(&firstTime,sizeof(uint32_t),1,file);
    koko+=fread(&lastTime,sizeof(uint32_t),1,file);
    if (frk == 0){
        memset(allFrelements,0,sizeof(SFrelementFull)*NUM_PERIODICITIES);
    }else{
        koko+=fread(allFrelements,sizeof(SFrelementFull),NUM_PERIODICITIES,file);
    }
    return 0;
}
