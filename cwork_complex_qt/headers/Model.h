#ifndef _MODEL_
#define _MODEL_

#include "IntegrationMethod.h"
#include "RightPart.h"
#include "vector_matrix.h"
#include <math.h>
#include "KalmanFilter.h"


class Model{
public:
    void setInitializeParametrs(float time_,
                                vector<float> state_){
        _state = state_;
        _time = time_;
    };

    void setIntegrationMethod(float step_,
                              int typeIntegrationMethod_){
        switch ( typeIntegrationMethod_ ) {
            case 1:
              _integration = new EilerMethod;
              break;
            case 2:
              _integration = new RungeKuttaMethod;
              break;
        }
        _integration->setIniatilizeParametrs(step_, _time, _state);
        _integration->setRightPartOfModel(_rightPart);
    };

    vector<float> getCurrentState(){
        return _state;
    };

    void computeNextState(){
        _state = _integration->computeState(_time, _state);
        _time += _integration->getStep();
    };

    vector<float> getNextState(){
        computeNextState();
        return _state;
    };

protected:
    vector<float> _state;
    float _time;
    RightPart* _rightPart;
    IntegrationMethod* _integration;
};


class  Satellite: public Model{
public:
    Satellite(){
        _rightPart = new RightPartOfSatellite;
    };

    void setReferenceState(vector<float> referenceState_){
        _referenceState = referenceState_;
    };

    void setIntegrationMethod(float step_,
                              int typeIntegrationMethod_){
        switch ( typeIntegrationMethod_ ) {
            case 1:
              _integration = new EilerMethod;
              _integrationReference = new EilerMethod;
              break;
            case 2:
              _integration = new RungeKuttaMethod;
              _integrationReference = new RungeKuttaMethod;
              break;
        }
        _integration->setIniatilizeParametrs(step_, _time, _state);
        _integration->setRightPartOfModel(_rightPart);
        _integrationReference->setIniatilizeParametrs(step_, _time, _state);
        _integrationReference->setRightPartOfModel(_rightPart);
    };


    vector<float> getCurrentReferenceState(){
        return _referenceState;
    };

    void computeNextReferenceState(){
        _referenceState = _integrationReference->computeState(_time, _referenceState);
        //_time += _integration->getStep();	 исправить, т.к. время уже суммируется в истинной траектории
    };

    vector<float> getNextReferenceState(){
        computeNextReferenceState();
        return _referenceState;
    };

protected:
    vector<float> _referenceState;
    IntegrationMethod* _integrationReference;
};


class  Consumer: public Model {
public:
    Consumer(){
        _rightPart = new RightPartOfConsumer;

        vector<float> deltaX(6);
        for (int i = 0; i < 6; ++i)
        {
            deltaX(i) = 0;
        }
        _deltaStateEstimateFK = deltaX;
        kalmanFilter = new KalmanFilter(deltaX);
    }

    void setReferenceState(vector<float> referenceState_){
        _referenceState = referenceState_;
        _estimateState = _state;
    };

    void setIntegrationMethod(float step_,
                              int typeIntegrationMethod_){
        switch ( typeIntegrationMethod_ ) {
            case 1:
              _integration = new EilerMethod;
              _integrationReference = new EilerMethod;
              break;
            case 2:
              _integration = new RungeKuttaMethod;
              _integrationReference = new RungeKuttaMethod;
              break;
        }
        _integration->setIniatilizeParametrs(step_, _time, _state);
        _integration->setRightPartOfModel(_rightPart);
        _integrationReference->setIniatilizeParametrs(step_, _time, _state);
        _integrationReference->setRightPartOfModel(_rightPart);
    };


    vector<float> getCurrentReferenceState(){
        return _referenceState;
    };

    void computeNextReferenceState(){
        _referenceState = _integrationReference->computeState(_time, _referenceState);
        //_time += _integration->getStep();	 исправить, т.к. время уже суммируется в истинной траектории
    };

    vector<float> getNextReferenceState(){
        computeNextReferenceState();
        return _referenceState;
    };

    void computeNextEstimateState(){
        vector<float> State(6);
        State = _estimateState + _deltaStateEstimateFK;
        _estimateState = State;
    }

    vector<float> getNextEstimateState(){
        computeNextEstimateState();
        return _estimateState;
    }

    vector<float> getCurrentEstimateState(){
        return _estimateState;
    }

    void computeEstimateDeltaState(std::vector<float> deltaPseudoDistance,
                                   std::vector<float> deltaDerivativePseudoDistance,
                                   std::vector< vector<float> > statesSatellites){
        float step = 10;
        vector<float> deltaX(6);
        int numSat = deltaPseudoDistance.size();
        vector<float> deltaY(numSat*2);


        for (int i = 0; i < 6; ++i)
        {
            deltaX(i) = 0;
        }

        for (int i = 0; i < numSat; ++i)
        {
            deltaY(i) = deltaPseudoDistance[i];
            deltaY(i+numSat) = deltaDerivativePseudoDistance[i];
        }

        identity_matrix<float> I(6);
        matrix<float> F(6,6);
        F = I + step*A(_referenceState);   ///что сюда подставлять, опорную или истинную, сейчас опорная


        float xSat, ySat, zSat, vxSat, vySat, vzSat;
        float xCon, yCon, zCon, vxCon, vyCon, vzCon;


        xCon = _referenceState(0);
        yCon = _referenceState(1);
        zCon = _referenceState(2);
        vxCon = _referenceState(3);
        vyCon = _referenceState(4);
        vzCon = _referenceState(5);

        float radius;


        matrix<float> H(numSat*2,6);
        for (int i = 0; i < numSat; ++i)
        {
            xSat = vector<float>(statesSatellites[i])(0);
            ySat = vector<float>(statesSatellites[i])(1);
            zSat = vector<float>(statesSatellites[i])(2);
            vxSat = vector<float>(statesSatellites[i])(3);
            vySat = vector<float>(statesSatellites[i])(4);
            vzSat = vector<float>(statesSatellites[i])(5);

            radius = sqrt((xSat-xCon)*(xSat-xCon)+(ySat-yCon)*(ySat-yCon)+(zSat-zCon)*(zSat-zCon));

            H(i,0) = -((xSat-xCon)/radius);
            H(i,1) = -((ySat-yCon)/radius);
            H(i,2) = -((zSat-zCon)/radius);
            H(i,3) = 0;
            H(i,4) = 0;
            H(i,5) = 0;

            H(i+numSat,0) = 0;
            H(i+numSat,1) = 0;
            H(i+numSat,2) = 0;
            H(i+numSat,3) = (xSat-xCon)/radius;
            H(i+numSat,4) = (ySat-yCon)/radius;
            H(i+numSat,5) = (zSat-zCon)/radius;

        }

        matrix<float> D(numSat*2, numSat*2);
        for (int i = 0; i < numSat*2; ++i)
        {
            for (int j = 0; j < numSat*2; ++j)
                {
                    D(i,j) = 0;
                }
        }
        for (int i = 0; i < numSat; ++i)
        {
            D(i,i) = pow(10/1000.0, 2);
        }
        for (int i = numSat; i < numSat*2; ++i)
        {
            D(i,i) = pow(0.3/1000.0, 2);
        }

        _deltaStateEstimateFK =  kalmanFilter->estimateDeltaX(deltaX,//_deltaStateEstimateFK,//
                                                deltaY, F,
                                                H, D);
    };

     vector <float> getDeltaStateEstimateFK(){
        return _deltaStateEstimateFK;
    };

     matrix<float> getEstP(){
         return kalmanFilter->getEstP();
     }


private:
    KalmanFilter* kalmanFilter;
    vector<float> _deltaStateEstimateFK;
    vector<float> _referenceState;
    vector<float> _estimateState;
    IntegrationMethod* _integrationReference;

    matrix<float> A(vector<float> state_){
        float mu = 398600.436;
        matrix<float> A(6,6);
        float x = state_(0);
        float y = state_(1);
        float z = state_(2);
        float r = pow(x*x+y*y+z*z,1/2.0);

        A(0,0) = 0; A(0,1) = 0; A(0,2) = 0; A(0,3) = 1; A(0,4) = 0; A(0,5) = 0;
        A(1,0) = 0; A(1,1) = 0; A(1,2) = 0; A(1,3) = 0; A(1,4) = 1; A(1,5) = 0;
        A(2,0) = 0; A(2,1) = 0; A(2,2) = 0; A(2,3) = 0; A(2,4) = 0; A(2,5) = 1;
        A(3,0) = -mu/(pow(r,3))*(1-3*pow(x/r,2));
            A(3,1) = 3*mu*x*y/pow(r,5);
                A(3,2) = 3*mu*x*z/pow(r,5);
                    A(3,3) = 0; A(3,4) = 0; A(3,5) = 0;
        A(4,0) = 3*mu*x*y/pow(r,5);
            A(4,1) = -mu/(pow(r,3))*(1-3*pow(y/r,2));
                A(4,2) = 3*mu*y*z/pow(r,5);
                    A(4,3) = 0; A(4,4) = 0; A(4,5) = 0;
        A(5,0) = 3*mu*x*z/pow(r,5);
            A(5,1) = 3*mu*z*y/pow(r,5);
                A(5,2) = -mu/(pow(r,3))*(1-3*pow(z/r,2));
                    A(5,3) = 0; A(5,4) = 0; A(5,5) = 0;
        return A;
    };
};




class  FormingFilter: public Model {
public:
    FormingFilter(){
        _rightPart = new RightPartOfFormingFilter;
    };
    void setParametrsDistributionFormingFilter(float mu_, float sigma_){
        _rightPart->setParametrsDistribution(mu_, sigma_);
    };
    void setParametrsForWhiteNoise(float step_, float beginTime_){
        _rightPart->setParametrsForWhiteNoiseGenerator(step_, beginTime_);
    };

};


#endif
