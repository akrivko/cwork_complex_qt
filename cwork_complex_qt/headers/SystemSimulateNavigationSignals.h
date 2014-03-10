#ifndef _SYSTEM_SIMULATE_NAVIGATION_SIGNALS_
#define _SYSTEM_SIMULATE_NAVIGATION_SIGNALS_


#include "vector_matrix.h"
#include <math.h>
#include "WhiteNoiseGenerator.h"
#include "Model.h"


class SystemSimulateNavigationSignals{
public:
    SystemSimulateNavigationSignals(){
        _formingFilterForDistance = new FormingFilter;
        _formingFilterForDerivativeDistance = new FormingFilter;
    }


    void setParametrs(double time_, double step_){
        vector<double> state(1);


        double mu1 = 0.0;
        double sigma1 = 15.0;
        double mu2 = 0.0;
        double sigma2 = 0.1;

        _whiteNoiseForDistance = new WhiteNoiseGenerator(0);
        _whiteNoiseForDerivativeDistance = new WhiteNoiseGenerator(0);

        state(0) = _whiteNoiseForDistance->getNoise();

        _formingFilterForDistance->setInitializeParametrs(time_, state);
        _formingFilterForDistance->setIntegrationMethod(step_, 2);
        _formingFilterForDistance->setParametrsDistributionFormingFilter(mu1, sigma1);
        _formingFilterForDistance->setParametrsForWhiteNoise(step_, time_);

        _formingFilterForDerivativeDistance->setInitializeParametrs(time_, state);
        _formingFilterForDerivativeDistance->setIntegrationMethod(step_, 2);
        _formingFilterForDerivativeDistance->setParametrsDistributionFormingFilter(mu2, sigma2);
        _formingFilterForDerivativeDistance->setParametrsForWhiteNoise(step_, time_);
    };


    double computeDistance(double time_, vector<double> stateConsumer_, vector<double> stateSatellite_){
        double distance;
        vector<double> deltaXYZConsumerSatellite(3);

        for (int i = 0; i < 3; ++i){
            deltaXYZConsumerSatellite(i) = stateConsumer_(i)-stateSatellite_(i);
        };

        distance = pow(inner_prod(deltaXYZConsumerSatellite, deltaXYZConsumerSatellite),0.5);

        return distance;
    }

    double computeReferenceDistance(double time_, vector<double> stateReferenceConsumer_, vector<double> stateReferenceSatellite_){
        return computeDistance(time_, stateReferenceConsumer_, stateReferenceSatellite_);
    }


    double computeTrueDistance(double time_, vector<double> stateConsumer_, vector<double> stateSatellite_){
        double distance;
        double sigmaChrIon = 10.0;
        double deltaChrIon = sigmaChrIon*_whiteNoiseForDistance->getNoise();
        double eta = vector<double>(_formingFilterForDistance->getNextState())(0);

        distance = computeDistance(time_, stateConsumer_, stateSatellite_);
        distance += eta + deltaChrIon;

        return distance;
    }


    double computeDerivativeDistance(double time_, vector<double> stateConsumer_, vector<double> stateSatellite_){
        double derivativeDistance = 0;

        vector<double> deltaVxVyVzConsumerSatellite(3);

        for (int i = 0; i < 3; ++i){
            deltaVxVyVzConsumerSatellite(i) = (stateSatellite_(i+3)-stateConsumer_(i+3));
        };

        vector<double> unitVectorForDistance(3);
        unitVectorForDistance(0) = (stateSatellite_(0) - stateConsumer_(0));
        unitVectorForDistance(1) = (stateSatellite_(1) - stateConsumer_(1));
        unitVectorForDistance(2) = (stateSatellite_(2) - stateConsumer_(2));

        double distance = computeDistance(time_, stateConsumer_, stateSatellite_);
        unitVectorForDistance = 1.0/distance * unitVectorForDistance;

        derivativeDistance = inner_prod(deltaVxVyVzConsumerSatellite, unitVectorForDistance);

        return derivativeDistance;
    }


    double computeReferenceDerivativeDistance(double time_, vector<double> stateReferenceConsumer_, vector<double> stateReferenceSatellite_){
        return computeDerivativeDistance(time_, stateReferenceConsumer_, stateReferenceSatellite_);
    }


    double computeTrueDerivativeDistance(double time_, vector<double> stateConsumer_, vector<double> stateSatellite_){
        double derivativeDistance;

        double sigmaSys =0.3;
        double deltaSys = sigmaSys*_whiteNoiseForDerivativeDistance->getNoise();//0.2;//
        double eta = vector<double>(_formingFilterForDerivativeDistance->getNextState())(0);

        derivativeDistance = computeDerivativeDistance(time_, stateConsumer_, stateSatellite_);
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
