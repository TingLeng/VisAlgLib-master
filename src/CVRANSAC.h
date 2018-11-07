/*******************************************************************************
* Copyright (C) 2011, 上海微电子装备有限公司
* All rights reserved.
* 产品号   : SSB225, SSB500, SSB300, SLD500
* 所属组件 : CV
* 模块名称 : CV
* 文件名称 : CVRANSAC.h
* 概要描述 : CVRANSAC定义
* 历史记录 :
* 版本    日  期    作  者  内容
*
* V1.0  2015-08-18  tuq  添加文件头部注释
* 
* 
******************************************************************************/

#ifndef _RANSAC_H_
#define _RANSAC_H_

#include <set>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <limits>





#include "CVRANSAC_ParameterEstimator.h"

/**
* This class implements the RAndom SAmple Consensus (RANSAC) framework,
* a framework for robust parameter estimation.
* Given data containing outliers we estimate the model parameters using sub-sets of
* the original data:
* 1. Choose the minimal subset from the data for computing the exact model parameters.
* 2. See how much of the input data agrees with the computed parameters.
* 3. Goto step 1. This can be done up to (N choose m) times, where m is the number of
*    data objects required for an exact estimate and N is the total number of data objects.
* 4. Take the largest subset of objects which agreed on the parameters and compute a
*    least squares fit using them.
* 
* This is based on:
* Fischler M.A., Bolles R.C., 
* ``Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography'', 
* Communications of the ACM, Vol. 24(6), 1981.
*
* Hartely R., Zisserman A., "Multiple View Geometry in Computer Vision", 2001.
*
* The class template parameters are T - objects used for the parameter estimation 
*                                      (e.g. Point2D in line estimation, 
*                                            std::pair<Point2D,Point2D> in homography estimation).
*                                   S - type of parameter (e.g. double).                          
*
* @author: Ziv Yaniv (zivy@isis.georgetown.edu)
*
*/
template<class T, class S>
class CVRANSAC 
{

public:

	static void compute(const std::vector<T> &data, 
		CVRANSAC_ParameterEstimator<T, S> *paramEstimator,
		int nMinimumInlinerSize,
		std::vector<std::vector<S> >&allParameters, 
		double desiredProbalilityForNoOutliers)
	{
		std::vector<S> parametersForOneInstance;
		std::vector<T> remainData = data;
		std::vector<T> inliers, outliers;


		std::vector<bool> vRangeLimit(paramEstimator->numParamters(), false);
		std::vector<S> vRangeSt;
		std::vector<S> vRangeEt;


		while(true)
		{
			parametersForOneInstance.clear();
			inliers.clear();
			outliers.clear();


			bool bFlag = compute(remainData, 
				paramEstimator, 
				nMinimumInlinerSize,
				desiredProbalilityForNoOutliers, 
				parametersForOneInstance, 
				vRangeLimit,
				vRangeSt,
				vRangeEt,
				inliers, outliers);

			if (bFlag)
			{
				allParameters.push_back(parametersForOneInstance);
				remainData = outliers;
			}
			else
			{
				break;
			}
		}
	}

	static void compute(const std::vector<T> &data, 
		CVRANSAC_ParameterEstimator<T, S> *paramEstimator,
		int nMinimumInlinerSize,
		std::vector<std::vector<S> >& allParameters, 
		std::vector<std::vector<T> >& allInliers,
		double desiredProbalilityForNoOutliers)
	{
		std::vector<S> parametersForOneInstance;
		std::vector<T> remainData = data;
		std::vector<T> inliers, outliers;


		std::vector<bool> vRangeLimit(paramEstimator->numParamters(), false);
		std::vector<S> vRangeSt;
		std::vector<S> vRangeEt;


		while(true)
		{
			parametersForOneInstance.clear();
			inliers.clear();
			outliers.clear();


			bool bFlag = compute(remainData, 
				paramEstimator, 
				nMinimumInlinerSize,
				desiredProbalilityForNoOutliers, 
				parametersForOneInstance, 
				vRangeLimit,
				vRangeSt,
				vRangeEt,
				inliers, outliers);

			if (bFlag)
			{
				allParameters.push_back(parametersForOneInstance);
				remainData = outliers;
				allInliers.push_back(inliers);
			}
			else
			{
				break;
			}
		}
	}



	/**
	* Estimate the model parameters using the RANSAC framework.
	* @param parameters A vector which will contain the estimated parameters.
	*                   If there is an error in the input then this vector will be empty.
	*                   Errors are: 1. Less data objects than required for an exact fit.
	*                               2. The given data is in a singular configuration (e.g. trying to fit a circle
	*                                  to a set of colinear points).
	*                               3. The given parameter desiredProbabilityForNoOutliers is not in (0,1)
	* @param paramEstimator An object which can estimate the desired parameters using either an exact fit or a 
	*                       least squares fit.
	* @param data The input from which the parameters will be estimated.
	* @param desiredProbabilityForNoOutliers The probability that at least one of the selected subsets doesn't contain an
	*                                        outlier, must be in (0,1).
	* @return Returns whether success to estimate the model
	*/
	static bool compute(
		const std::vector<T> &data,
		CVRANSAC_ParameterEstimator<T,S> *paramEstimator,
		int nMinimumInlierSize,
		double desiredProbabilityForNoOutliers,
		std::vector<S> &parameters, 
		std::vector<bool> vRangeLimit,
		std::vector<S> vRangeSt,
		std::vector<S> vRangeEt,
		std::vector<T> &inliers,
		std::vector<T> &outliers)
	{
		bool bRet = false;
		int numDataObjects = data.size();// 数据点的个数
		unsigned int numForEstimate = paramEstimator->numForEstimate();// 估计模型需要的点的个数

		//there are less data objects than the minimum required for an exact fit, or
		//desiredProbabilityForNoOutliers is not in (0.0,1.0)
		if (numDataObjects < (int)numForEstimate || 
			desiredProbabilityForNoOutliers>=1.0 || 
			desiredProbabilityForNoOutliers<=0.0)
		{
			return bRet;
		}

		std::vector<const T *> exactEstimateData;
		std::vector<const T *> leastSquaresEstimateData;
		std::vector<S> exactEstimateParameters;
		int  j, k, l, numVotesForBest, numVotesForCur, maxIndex;
		unsigned int i;

		unsigned int numTries;

		//true if data[i] agrees with the best model, otherwise false
		bool *bestVotes = new bool[numDataObjects]; 
		memset(bestVotes, 0, numDataObjects*sizeof(bool));

		//true if data[i] agrees with the current model, otherwise false
		bool *curVotes = new bool[numDataObjects];
		memset(curVotes, 0, numDataObjects*sizeof(bool));

		//true if data[i] is NOT chosen for computing the exact fit, otherwise false
		bool *notChosen = new bool[numDataObjects]; 
		memset(notChosen, 0, numDataObjects*sizeof(bool));

		SubSetIndexComparator subSetIndexComparator(numForEstimate);

		std::set<int *, SubSetIndexComparator > chosenSubSets(subSetIndexComparator);

		int *curSubSetIndexes = NULL;// 存储exact model用的的点的索引

		double numerator = log(1.0-desiredProbabilityForNoOutliers);
		double denominator;
		//allTries is either the correct value or if there is an overflow
		//during the computation it is set to the maximal value
		//for unsigned int
		unsigned int allTries = choose(numDataObjects,numForEstimate);

		parameters.clear();
		srand((unsigned)time(NULL)); //seed random number generator

		numVotesForBest = 0; //initialize with 0 so that the first computation which gives any type of fit will be set to best
		numTries = allTries; //initialize with the number of all possible subsets

		for(i=0; i<numTries; i++) 
		{
			//randomly select data for exact model fit ('numForEstimate' objects).
			std::fill(notChosen,notChosen+numDataObjects, true); 
			curSubSetIndexes = new int[numForEstimate];

			exactEstimateData.clear();

			maxIndex = numDataObjects-1; 
			for(l=0; l< (int)numForEstimate; l++) 
			{
				//selectedIndex is in [0,maxIndex]
				int selectedIndex = (int)(((float)rand()/(float)RAND_MAX)*maxIndex + 0.5);

				// 选离selectedIndex最近的一个可用点（未被选用的，防止选重复的点）
				for(j=-1,k=0; k<numDataObjects && j<selectedIndex; k++) 
				{
					if(notChosen[k])
						j++;
				}
				k--;
				exactEstimateData.push_back(&(data[k]));
				notChosen[k] = false;
				maxIndex--;
			}


			//get the indexes of the chosen objects so we can check that this sub-set hasn't been
			//chosen already
			for(l=0, j=0; j<numDataObjects; j++) 
			{
				if(!notChosen[j]) 
				{
					curSubSetIndexes[l] = j+1;// 为什么加1？
					l++;
				}
			}

			//check that the sub-set just chosen is unique
			std::pair< typename std::set<int *, SubSetIndexComparator >::iterator, bool> res = chosenSubSets.insert(curSubSetIndexes);

			if(res.second == true) //first time we chose this sub set
			{ 
				//use the selected data for an exact model parameter fit
				paramEstimator->estimate(exactEstimateData,exactEstimateParameters);
				//selected data is a singular configuration (e.g. three colinear points for 
				//a circle fit)
				if(exactEstimateParameters.size() == 0)
					continue;

				bool con = false;
				for (int kk=0; kk<(int)exactEstimateParameters.size(); kk++)
				{
					if (vRangeLimit[kk] && !(exactEstimateParameters[kk] >= vRangeSt[kk] && exactEstimateParameters[kk] <= vRangeEt[kk]))
					{
						con = true;
						break;
					}
				}
				if (con)
				{
					continue;
				}

				//see how many agree on this estimate
				numVotesForCur = 0;
				std::fill(curVotes,curVotes+numDataObjects, false);		

				//continue checking data until there is no chance of getting a larger consensus set 
				//or all the data has been checked 
				int nAgreeWeight = 0;
				for(j=0; j<numDataObjects && numVotesForBest-numVotesForCur<numDataObjects-j+1; j++) 
				{
					nAgreeWeight = paramEstimator->agree(exactEstimateParameters, data[j]);
					if(nAgreeWeight > 0) 
					{
						curVotes[j] = true;
						numVotesForCur += nAgreeWeight;
					}
				}                           
				
				//if (numVotesForCur >= nMinimumInlierSize)
				{
					if(numVotesForCur > numVotesForBest)//found a larger consensus set?
					{
						numVotesForBest = numVotesForCur;				
						std::copy(curVotes, curVotes+numDataObjects, bestVotes);
						
						//all data objects are inliers, terminate the search
						if(numVotesForBest == numDataObjects)
							i=numTries;                
						else //update the estimate of outliers and the number of iterations we need				           		  				
						{  
							denominator = log(1.0- pow((double)numVotesForCur/(double)numDataObjects, (double)(numForEstimate)));
							numTries = (unsigned int)(numerator/denominator + 0.5);
							
							//there are cases when the probablistic number of tries is greater than all possible sub-sets
							numTries = numTries<allTries ? numTries : allTries;
						}
					}
				}
			}
			else //this sub set already appeared, release memory
			{  
				delete [] curSubSetIndexes;			
			}
		}

		//release the memory
		typename std::set<int *, SubSetIndexComparator >::iterator it = chosenSubSets.begin();
		typename std::set<int *, SubSetIndexComparator >::iterator chosenSubSetsEnd = chosenSubSets.end();
		while(it!=chosenSubSetsEnd) {
			delete [] (*it);
			it++;
		}
		chosenSubSets.clear();

		//compute the least squares estimate using the largest sub set
		outliers.clear();
		if(numVotesForBest >= nMinimumInlierSize) 
		{
			bRet = true;

			for (j = 0; j < numDataObjects; j++)
			{
				if (bestVotes[j])
				{
					leastSquaresEstimateData.push_back(&(data[j]));
					inliers.push_back(data[j]);
				}
				else
					outliers.push_back(data[j]);
			}
			paramEstimator->leastSquaresEstimate(leastSquaresEstimateData,parameters);
		}
		else
		{
			for (j = 0; j < numDataObjects; j++)
			{
				outliers.push_back(data[j]);
			}
		}


		delete [] bestVotes;
		delete [] curVotes;
		delete [] notChosen;

		return bRet;
		//return (double)numVotesForBest/(double)numDataObjects;
	}

private:

	/**
	* Compute n choose m  [ n!/(m!*(n-m)!)]. 
	* If choose(n,m)>std::numeric_limits<unsigned int>::max(), or there is an
	* overflow during the computations then we return 
	* std::numeric_limits<unsigned int>::max(), otherwise the correct value
	* is returned.
	*/
	static unsigned int choose(unsigned int n, unsigned int m)
	{
		double denominatorEnd, numeratorStart, numerator,denominator, i, result; 
		//perform smallest number of multiplications
		if((n-m) > m) {
			numeratorStart = n-m+1;
			denominatorEnd = m;
		}
		else {
			numeratorStart = m+1;
			denominatorEnd = n-m;
		}

		for(i=numeratorStart, numerator=1; i<=n; i++)
			numerator*=i;
		for(i=1, denominator=1; i<=denominatorEnd; i++)
			denominator*=i;	
		result = numerator/denominator;





		//check for overflow both in computation and in result	  
		/*if(denominator>std::numeric_limits<double>::max() || 
			numerator>std::numeric_limits<double>::max() || 
			static_cast<double>(std::numeric_limits<unsigned int>::max())<result )	
			return std::numeric_limits<unsigned int>::max();
		else 
			return static_cast<unsigned int>(result);*/  


		if( denominator > std::numeric_limits<double>::max()|| 
			numerator > std::numeric_limits<double>::max() ||
			static_cast<double>(std::numeric_limits<unsigned int>::max()) < result )
		{
			return std::numeric_limits<unsigned int>::max();
		}
		else 
		{
			return static_cast<unsigned int>(result);   
		}
	}


	static void computeAllChoices(CVRANSAC_ParameterEstimator<T,S> *paramEstimator, std::vector<T> &data,
		bool *bestVotes, bool *curVotes, int &numVotesForBest, int startIndex, int k, int arrIndex, int *arr)
	{
		//we have a new choice of indexes
		if(k==0) {
			estimate(paramEstimator, data, bestVotes, curVotes, numVotesForBest, arr);
			return;
		}
		//continue to recursivly generate the choice of indexes
		int endIndex = data.size()-k;
		for(int i=startIndex; i<=endIndex; i++) {
			arr[arrIndex] = i;
			computeAllChoices(paramEstimator, data, bestVotes, curVotes, numVotesForBest,
				i+1, k-1, arrIndex+1, arr);
		}

	}



	static void estimate(CVRANSAC_ParameterEstimator<T,S> *paramEstimator, std::vector<T> &data,
		bool *bestVotes, bool *curVotes, int &numVotesForBest, int *arr)
	{
		std::vector<T *> exactEstimateData;
		std::vector<S> exactEstimateParameters;
		unsigned int numDataObjects;
		unsigned int numVotesForCur;
		unsigned int j;

		numDataObjects = data.size();
		std::fill(curVotes,curVotes+numDataObjects, false);
		numVotesForCur=0;

		unsigned int numForEstimate = paramEstimator->numForEstimate();

		for(j=0; j<numForEstimate; j++)
			exactEstimateData.push_back(&(data[arr[j]]));
		paramEstimator->estimate(exactEstimateData,exactEstimateParameters);
		//singular data configuration
		if(exactEstimateParameters.size()==0)
			return;

		int nAgreeWeight = 0;
		for(j=0; j<numDataObjects; j++)
		{
			nAgreeWeight = paramEstimator->agree(exactEstimateParameters, data[j]);
			if(nAgreeWeight > 0) 
			{
				curVotes[j] = true;
				numVotesForCur += nAgreeWeight;
			}
		}
		if(numVotesForCur > numVotesForBest) 
		{
			numVotesForBest = numVotesForCur;
			std::copy(curVotes, curVotes+numDataObjects, bestVotes);		
		}
	}

	class SubSetIndexComparator {
	private:
		int length;
	public:
		SubSetIndexComparator(int arrayLength) : length(arrayLength){}
		bool operator()(const int *arr1, const int *arr2) const {
			for(int i=0; i<this->length; i++) {
				if(arr1[i] < arr2[i])
					return true;
				else if(arr1[i] > arr2[i]) 
					return false;
			}
			return false;			
		}
	};
};


#endif //_RANSAC_H_
