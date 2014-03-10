#ifndef _RIGHT_PART_
#define _RIGHT_PART_

#include "vector_matrix.h"
#include <math.h>
#include "WhiteNoiseGenerator.h"

const int num_comp = 22;

class RightPart{
public:	
    RightPart(){}
    virtual vector<double> rightPart(double time_, vector<double> state_){}
    virtual void setParametrsForWhiteNoiseGenerator(double step_, double beginTime_){}
    virtual void setParametrsDistribution(double mu_, double sigma_){}
};


class RightPartOfSatellite: public RightPart{
public:	
    vector<double> rightPart(double time_, vector<double> state_){
        double mu = 398600.436e+9;

        vector<double> resultVector(state_.size());

        double radius3 = pow(state_(0)*state_(0) + state_(1)*state_(1) + state_(2)*state_(2), 3/2.0);
		
		resultVector(0) = state_(3) ;
		resultVector(1) = state_(4) ;
		resultVector(2) = state_(5);

        resultVector(3) = -mu*state_(0)/radius3;
        resultVector(4) = -mu*state_(1)/radius3 ;
		resultVector(5) = - mu*state_(2)/radius3;

		return resultVector;
	};

};


class RightPartOfConsumer: public RightPart{
public:	
    vector<double> rightPart(double time_, vector<double> state_){
        double mu = 398600.436e+9;

        vector<double> resultVector(state_.size());

        double radius3 = pow(state_(0)*state_(0) + state_(1)*state_(1) + state_(2)*state_(2), 3/2.0);
		
        for (int i=0; i<state_.size(); i++){
            resultVector(i) = 0;
        }

        resultVector(0) = state_(3);
        resultVector(1) = state_(4);
		resultVector(2) = state_(5);

		resultVector(3) = -mu*state_(0)/radius3;
        resultVector(4) = -mu*state_(1)/radius3;
        resultVector(5) = -mu*state_(2)/radius3;

		return resultVector;
    }

};


class RightPartOfFormingFilter: public RightPart{
public:	
    vector<double> rightPart(double time_, vector<double> state_){
        vector<double> resultVector(1);
		
        resultVector(0) = -1.0*_mu*state_(0) + pow(2*_sigma*_sigma,0.5)*_whiteNoise->getNoise();
		
		return resultVector;
	};
    void setParametrsForWhiteNoiseGenerator(double step_, double beginTime_){
        _whiteNoise = new WhiteNoiseGenerator(0);
	};
    void setParametrsDistribution(double mu_, double sigma_){
		_mu = mu_;
		_sigma = sigma_;
	};

protected:
    double _mu;
    double _sigma;
	WhiteNoiseGenerator* _whiteNoise;

};



#endif
