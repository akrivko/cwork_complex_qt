#ifndef _MODEL_
#define _MODEL_

#include "IntegrationMethod.h"
#include "RightPart.h"
#include "vector_matrix.h"
#include <math.h>
#include "KalmanFilter.h"

class Model{
public:
    void setInitializeParametrs(double time_,
                                vector<double> state_){
        _state = state_;
        _time = time_;
    };

    void setIntegrationMethod(double step_,
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

    vector<double> getCurrentState(){
        return _state;
    };

    void computeNextState(){
        _state = _integration->computeState(_time, _state);
        _time += _integration->getStep();
    };

    vector<double> getNextState(){
        computeNextState();
        return _state;
    };

protected:
    vector<double> _state;
    double _time;
    RightPart* _rightPart;
    IntegrationMethod* _integration;
};



class  Satellite: public Model{
public:
    Satellite(){
        _rightPart = new RightPartOfSatellite;
    }

    void setReferenceState(vector<double> referenceState_){
        _referenceState = referenceState_;
    };

    void setIntegrationMethod(double step_,
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
        _integrationReference->setIniatilizeParametrs(step_, _time, _referenceState);
        _integrationReference->setRightPartOfModel(_rightPart);
    };


    vector<double> getCurrentReferenceState(){
        return _referenceState;
    };

    void computeNextReferenceState(){
        _referenceState = _integrationReference->computeState(_time, _referenceState);
        /*ToDO
         *добавить время
         */
    };

    vector<double> getNextReferenceState(){
        computeNextReferenceState();
        return _referenceState;
    };

protected:
    vector<double> _referenceState;
    IntegrationMethod* _integrationReference;
};



class  Consumer: public Satellite {
public:
    Consumer(){
        _rightPart = new RightPartOfConsumer;

        matrix<double> initP(num_comp,num_comp);
        for (int i = 0; i < num_comp; ++i){
            for (int j = 0; j < num_comp; ++j){
                initP(i,j) = 0;
            }
        }
        for (int i = 0; i < 3; ++i)
        {
            initP(i,i) = 1e8;
            initP(i+3,i+3) = 1e5;
        }
        for (int i = 6; i < num_comp; ++i)
        {
            initP(i,i) = 1000;
        }

        kalmanFilter = new KalmanFilter(initP);

        vector<double> initDeltaX(num_comp);
        for (int i = 0; i < num_comp; ++i)
        {
            initDeltaX(i) = 0;
        }

        _deltaStateEstimateFK = initDeltaX;
    }


    void setReferenceState(vector<double> referenceState_){
        _referenceState = referenceState_;
        refEstimate = _referenceState;
    }


    void computeEstimateDeltaState(std::vector<double> deltaPseudoDistance,
                                   std::vector<double> deltaDerivativePseudoDistance,
                                   std::vector< vector<double> > statesSatellites){
        double step = 10;

        std::cout<<"/****** "<< _referenceState(6)*(10.0*10.0+15.0*15.0+100*100)/(10.0*10.0+15.0*15.0)<<"****/" <<std::endl;
        int numSat = deltaPseudoDistance.size();
        vector<double> deltaY(numSat);

        vector<double> deltaX(num_comp);

        //deltaX = _deltaStateEstimateFK;

        for (int i = 0; i < num_comp; ++i)
        {
            deltaX(i) = 0;
        }

        for (int i = 0; i < numSat; ++i)
        {
            deltaY(i) = deltaPseudoDistance[i];
            //std::cout<<"............."<<deltaY(i)<<"..........."<<std::endl;
        }

        matrix<double> I(num_comp, num_comp);
        for (int i = 0; i < num_comp; ++i){
            for (int j = 0; j < num_comp; ++j){
                I(i,j) = 0;
            }
        }
        for (int i = 0; i < num_comp; ++i){
            I(i,i) = 1;
        }

        matrix<double> F(num_comp,num_comp);
        F = I + step*A(refEstimate);

        double xSat, ySat, zSat, vxSat, vySat, vzSat;
        double xCon, yCon, zCon, vxCon, vyCon, vzCon;

        xCon = _referenceState(0);
        yCon = _referenceState(1);
        zCon = _referenceState(2);
        vxCon = _referenceState(3);
        vyCon = _referenceState(4);
        vzCon = _referenceState(5);

        double radius;

        matrix<double> H(numSat,num_comp);
        for (int i = 0; i < numSat; ++i)
        {
            xSat =  vector<double>(statesSatellites[i])(0);
            ySat =  vector<double>(statesSatellites[i])(1);
            zSat =  vector<double>(statesSatellites[i])(2);
            vxSat = vector<double>(statesSatellites[i])(3);
            vySat = vector<double>(statesSatellites[i])(4);
            vzSat = vector<double>(statesSatellites[i])(5);

            radius = sqrt((xSat-xCon)*(xSat-xCon)+(ySat-yCon)*(ySat-yCon)+(zSat-zCon)*(zSat-zCon));

            H(i,0) = -(xSat-xCon)/radius;
            H(i,1) = -(ySat-yCon)/radius;
            H(i,2) = -(zSat-zCon)/radius;

            H(i,3) = 0;
            H(i,4) = 0;
            H(i,5) = 0;

            for (int j = 6; j < num_comp; ++j)
            {
                H(i,j) = 0;
            }
            H(i, i+6) = 1;
        }

        matrix<double> D(numSat, numSat);
        for (int i = 0; i < numSat; ++i)
        {
            for (int j = 0; j < numSat; ++j)
                {
                    D(i,j) = 0;
                }
        }
        for (int i = 0; i < numSat; ++i)
        {
            D(i,i) = (10.0*10.0+15.0*15.0+100*100);
        }

        kalmanFilter->estimateP(F, H, D);
        _deltaStateEstimateFK =  kalmanFilter->estimateDeltaX(deltaX,
                                                              deltaY, F,
                                                              H, D);
//        deltaX = _deltaStateEstimateFK;

//        for (int i = 0; i < 6; ++i)
//        {
//            deltaX(i) = 0;
//        }

//        for (int i = 0; i < numSat; ++i)
//        {
//            deltaY(i) = deltaPseudoDistance[i]-_deltaStateEstimateFK(i+6);
//        }

//        _deltaStateEstimateFK =  kalmanFilter->estimateDeltaX(deltaX,
//                                                              deltaY, F,
//                                                              H, D);

        //_referenceState = _referenceState + _deltaStateEstimateFK;
        for (int i = 0; i < 6; ++i)
        {
            _referenceState(i) = _referenceState(i) + _deltaStateEstimateFK(i);
        }

        for (int i = 6; i < num_comp; ++i)
        {
            _referenceState(i) = _deltaStateEstimateFK(i);
        }


        refEstimate = _referenceState;

    }

     vector <double> getDeltaStateEstimateFK(){
        return _deltaStateEstimateFK;
    }

     matrix<double> getEstP(){
         return kalmanFilter->getEstP();
     }


private:
    KalmanFilter* kalmanFilter;
    vector<double> _deltaStateEstimateFK;
    vector<float> refEstimate;

    matrix<double> A(vector<double> state_){
        double mu = 398600.436e+9;
        matrix<double> A(num_comp,num_comp);
        double x = state_(0);
        double y = state_(1);
        double z = state_(2);
        double r = pow(x*x+y*y+z*z, 1/2.0);

        for (int i=0; i<num_comp; ++i){
            for (int j=0; j<num_comp; ++j){
                A(i,j) = 0;
            }
        }

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
    }
};


class  FormingFilter: public Model {
public:
    FormingFilter(){
        _rightPart = new RightPartOfFormingFilter;
    }
    void setParametrsDistributionFormingFilter(double mu_, double sigma_){
        _rightPart->setParametrsDistribution(mu_, sigma_);
    }
    void setParametrsForWhiteNoise(double step_, double beginTime_){
        _rightPart->setParametrsForWhiteNoiseGenerator(step_, beginTime_);
    }

};


#endif
