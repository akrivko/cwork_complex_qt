#ifndef _SYSTEM_SIMULATE_NAVIGATION_SIGNALS_
#define _SYSTEM_SIMULATE_NAVIGATION_SIGNALS_


#include "vector_matrix.h"
#include <math.h>
#include "WhiteNoiseGenerator.h"


class SystemSimulateNavigationSignals{
public:
    SystemSimulateNavigationSignals(){
        _formingFilterForDistance = new FormingFilter;
        _formingFilterForDerivativeDistance = new FormingFilter;
    };


    void setParametrs(float time_, float step_){
        vector<float> state(1);
        state(0) = 0;

        float mu1 = 0/1000.0;
        float sigma1 = 15/1000.0;
        float mu2 = 0/1000.0;
        float sigma2 = 0.1/1000.0;

        _whiteNoiseForDistance = new WhiteNoiseGenerator(0);
        _whiteNoiseForDerivativeDistance = new WhiteNoiseGenerator(0);

        _formingFilterForDistance->setInitializeParametrs(time_, state);
        _formingFilterForDistance->setIntegrationMethod(step_, 1);
        _formingFilterForDistance->setParametrsDistributionFormingFilter(mu1, sigma1);
        _formingFilterForDistance->setParametrsForWhiteNoise(step_, time_);

        _formingFilterForDerivativeDistance->setInitializeParametrs(time_, state);
        _formingFilterForDerivativeDistance->setIntegrationMethod(step_, 1);
        _formingFilterForDerivativeDistance->setParametrsDistributionFormingFilter(mu2, sigma2);
        _formingFilterForDerivativeDistance->setParametrsForWhiteNoise(step_, time_);
    };


    float computeDistance(float time_, vector<float> stateConsumer_, vector<float> stateSatellite_){
        float distance;
        vector<float> deltaXYZConsumerSatellite(3);

        for (int i = 0; i < 3; ++i){
            deltaXYZConsumerSatellite(i) = stateConsumer_(i)-stateSatellite_(i);
        };

        distance = pow(inner_prod(deltaXYZConsumerSatellite, deltaXYZConsumerSatellite),0.5);

        return distance;
    }

    float computeReferenceDistance(float time_, vector<float> stateReferenceConsumer_, vector<float> stateReferenceSatellite_){
        return computeDistance(time_, stateReferenceConsumer_, stateReferenceSatellite_);
    }


    float computeTrueDistance(float time_, vector<float> stateConsumer_, vector<float> stateSatellite_){
        float distance = 0;
        float muChrIon = 0; float sigmaChrIon = 10/1000.0;
        float deltaChrIon = muChrIon + sigmaChrIon*(-0.7);//_whiteNoiseForDistance->getNoise();
        float eta = vector<float>(_formingFilterForDistance->getNextState())(0);

        distance += computeDistance(time_, stateConsumer_, stateSatellite_);
        distance += eta + deltaChrIon;

        return distance;
    }


    float computeDerivativeDistance(float time_, vector<float> stateConsumer_, vector<float> stateSatellite_){
        float derivativeDistance = 0;

        vector<float> deltaVxVyVzConsumerSatellite(3);

        for (int i = 0; i < 3; ++i){
            deltaVxVyVzConsumerSatellite(i) = fabs(stateConsumer_(i+3)-stateSatellite_(i+3));
        };

        vector<float> unitVectorForDistance(3);
        unitVectorForDistance(0) = fabs(stateSatellite_(0) - stateConsumer_(0));
        unitVectorForDistance(1) = fabs(stateSatellite_(1) - stateConsumer_(1));
        unitVectorForDistance(2) = fabs(stateSatellite_(2) - stateConsumer_(2));

        float distance = computeDistance(time_, stateConsumer_, stateSatellite_);
        unitVectorForDistance = 1.0/distance * unitVectorForDistance;

        derivativeDistance = inner_prod(deltaVxVyVzConsumerSatellite, unitVectorForDistance);

        return derivativeDistance;
    }


    float computeReferenceDerivativeDistance(float time_, vector<float> stateReferenceConsumer_, vector<float> stateReferenceSatellite_){
        return computeDerivativeDistance(time_, stateReferenceConsumer_, stateReferenceSatellite_);
    }


    float computeTrueDerivativeDistance(float time_, vector<float> stateConsumer_, vector<float> stateSatellite_){
        float derivativeDistance = 0;

        float muSys = 0; float sigmaSys =0.3/1000.0;
        float deltaSys = muSys + sigmaSys*0.2;//_whiteNoiseForDerivativeDistance->getNoise();
        float eta = vector<float>(_formingFilterForDerivativeDistance->getNextState())(0);

        derivativeDistance += computeDerivativeDistance(time_, stateConsumer_, stateSatellite_);
        derivativeDistance += eta + deltaSys;

        return derivativeDistance;
    }


protected:
    FormingFilter* _formingFilterForDistance;
    FormingFilter* _formingFilterForDerivativeDistance;
    WhiteNoiseGenerator* _whiteNoiseForDistance;
    WhiteNoiseGenerator* _whiteNoiseForDerivativeDistance;
};


#endif
