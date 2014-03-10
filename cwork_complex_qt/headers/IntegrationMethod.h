#ifndef _INTEGRATION_METHOD_
#define _INTEGRATION_METHOD_


#include "RightPart.h"
#include "vector_matrix.h"


class IntegrationMethod{
public:	
	IntegrationMethod(){

	};
    void setIniatilizeParametrs(double step_,
                                double time_,
                                vector<double> state_){
		_step = step_;
		_time = time_;
		_state = state_;
	};
    void setStep(double step_){
		_step = step_;
	};
    void setTime(double time_){
		_time = time_;
	};
    void setState(vector<double> state_){
		_state = state_;
	};
	void setRightPartOfModel(RightPart* rightPart_){
		_rightPartOfModel = rightPart_;
	};
    double getStep(){
		return _step;
	};
    virtual vector<double> computeState(double time_, vector<double> state_){};
	

protected:	
    double _step;
    double _time;
    vector<double> _state;
	RightPart* _rightPartOfModel;
};

class EilerMethod: public IntegrationMethod{
public:
    vector<double> computeState(double time_, vector<double> state_){
        vector<double> state(state_.size());
        state = state_ + _step*_rightPartOfModel->rightPart(time_, state_);
        return state;
	};
};

class RungeKuttaMethod: public IntegrationMethod{
public:
    vector<double> computeState(double time_, vector<double> state_){
        vector<double> k(state_.size());
        vector<double> k1(state_.size());
        vector<double> k2(state_.size());
        vector<double> k3(state_.size());
        vector<double> k4(state_.size());
        vector<double> state(state_.size());
		k1 = _rightPartOfModel->rightPart(time_, state_);
		k2 = _rightPartOfModel->rightPart(time_+0.5*_step, state_+0.5*_step*k1);
		k3 = _rightPartOfModel->rightPart(time_+0.5*_step, state_+0.5*_step*k2);
		k4 = _rightPartOfModel->rightPart(time_+_step, state_+_step*k1);
		k = k1 + 2*(k2+k3) + k4;
        state = state_ + k*_step/6;


        return state;
	};
};


#endif
