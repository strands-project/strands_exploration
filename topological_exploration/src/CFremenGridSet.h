#ifndef CFremenGridSet_H
#define CFremenGridSet_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "CFremenGrid.h"

#define MAX_LENGTH 10000

using namespace std;

class CFremenGridSet
{
	public:
        CFremenGridSet();
        ~CFremenGridSet();

		/*add new measurements  
		  - if the name (ID) is not in the collection of states and createNew is false, the function aborts with return code -1 
		  - if the name (ID) is not in the collection of states and createNew is true, the new state is added to the collection
		  - if not, the measurements are added to the state with the given ID 
		  - the function returns -1 if the state does not exist (and was not created), otherwise it returne the number of added states*/
        int add(const char* name, float originX,float originY,float originZ,int dimX,int dimY,int dimZ,float cellSize);

        int incorporate(const char *name, float *x,float *y,float *z,float *d,int size,uint32_t t);

		/*estimate probabilities of the given state for the given times - the probs array is an output
		  returns false if the state with the given ID is not present in the collection
          otherwise returns true and fills the probs array with the calculated predictions*/
        float estimate(const char *name, unsigned int index, uint32_t timeStamp);

        float retrieve(const char *name, unsigned int index, uint32_t timeStamp);

        float dominant(const char *name, unsigned int index, uint32_t timeStamp);


		/*estimate entropies of the given state for the given times - the entropy array is an output
		  returns false if the state with the given ID is not present in the collection
		  otherwise returns true and fills the probs array with the calculated predictions*/
        int estimateEntropy(const char *name, float x,float y,float z,float range,uint32_t t);

		/*evaluate the prediction/estimation for the given states and times
		  returns -1 if the state with the given ID is not present in the collection
		  otherwise returns the best performing model order and the errors in the eval array*/
//		int evaluate(const char *name,uint32_t times[],unsigned char probs[],int length,int order,float eval[]);

		/*remove states from the collection
		  return the number of remaining states*/
		int remove(const char *name);

		/*remove states from the collection
		  returns false if the state with the given ID is not present in the collection*/
		bool update(const char* name,int order);

		/*print model info*/
        bool print(bool verbosityLevel);

		/*load the model from a file*/
		bool load(const char* name);

		/*save the model to a file*/
        bool save(const char* name);

		int find(const char *name);


        CFremenGrid* fremengrid[MAX_LENGTH];
        CFremenGrid* active;
        int numFremenGrids;
		int activeIndex;
};

#endif //CEDGESTATISTICS_H

