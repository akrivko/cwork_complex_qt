#ifndef _RIGHT_PART_
#define _RIGHT_PART_


#include "vector_matrix.h"
#include <math.h>
#include "WhiteNoiseGenerator.h"


class RightPart{
public:	
	RightPart(){};
	virtual vector<float> rightPart(float time_, vector<float> state_){};
	virtual void setParametrsForWhiteNoiseGenerator(float step_, float beginTime_){};
	virtual void setParametrsDistribution(float mu_, float sigma_){};
};


class RightPartOfSatellite: public RightPart{
public:	
	vector<float> rightPart(float time_, vector<float> state_){
		// float mu = 5165861650560.0;
		float mu = 398600.436;
		float omegaEarth = 7.2921158553*1e-5;

		vector<float> resultVector(6);

		float radius3 = pow(state_(0)*state_(0) + state_(1)*state_(1) + state_(2)*state_(2), 3/2.0);
		
		resultVector(0) = state_(3) ;
		resultVector(1) = state_(4) ;
		resultVector(2) = state_(5);

		resultVector(3) = -mu*state_(0)/radius3; // - (- 2*state_(4)*omegaEarth - 
		                  // omegaEarth*omegaEarth*state_(0));
		resultVector(4) = -mu*state_(1)/radius3 ;//- 
		                  // (+ 2*state_(3)*omegaEarth - 
		                  // omegaEarth*omegaEarth*state_(1));
		resultVector(5) = - mu*state_(2)/radius3;

		return resultVector;
	};

};


class RightPartOfConsumer: public RightPart{
public:	
    vector<float> rightPart(float time_, vector<float> state_){
		//float mu = 5165861650560.0;
		float mu = 398600.436;
		float omegaEarth = 7.2921158553*1e-5;

		vector<float> resultVector(6);

		float radius3 = pow(state_(0)*state_(0) + state_(1)*state_(1) + state_(2)*state_(2), 3/2.0);
		
		resultVector(0) = state_(3) ;
		resultVector(1) = state_(4) ;
		resultVector(2) = state_(5);

		resultVector(3) = -mu*state_(0)/radius3;
		 // -          (- 2*state_(4)*omegaEarth - 
		 //                  omegaEarth*omegaEarth*state_(0));
		resultVector(4) = -mu*state_(1)/radius3 ;
		// - 
		//                   (+ 2*state_(3)*omegaEarth - 
		//                   omegaEarth*omegaEarth*state_(1));
		resultVector(5) = - mu*state_(2)/radius3;

		return resultVector;
    }

};


class RightPartOfFormingFilter: public RightPart{
public:	
	vector<float> rightPart(float time_, vector<float> state_){
		vector<float> resultVector(1);
		
        resultVector(0) = -1.0*_mu*state_(0) + pow(2*_sigma*_sigma,0.5)*_whiteNoise->getNoise()/1000;
		
		return resultVector;
	};
	void setParametrsForWhiteNoiseGenerator(float step_, float beginTime_){
        _whiteNoise = new WhiteNoiseGenerator(0);
	};
	void setParametrsDistribution(float mu_, float sigma_){
		_mu = mu_;
		_sigma = sigma_;
	};

protected:
	float _mu;
	float _sigma;
	WhiteNoiseGenerator* _whiteNoise;

};



#endif
