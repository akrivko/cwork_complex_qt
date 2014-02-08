#ifndef _INTEGRATION_METHOD_
#define _INTEGRATION_METHOD_


#include "RightPart.h"
#include "vector_matrix.h"


class IntegrationMethod{
public:	
	IntegrationMethod(){

	};
	void setIniatilizeParametrs(float step_,
								float time_,
								vector<float> state_){
		_step = step_;
		_time = time_;
		_state = state_;
	};
	void setStep(float step_){
		_step = step_;
	};
	void setTime(float time_){
		_time = time_;
	};
	void setState(vector<float> state_){
		_state = state_;
	};
	void setRightPartOfModel(RightPart* rightPart_){
		_rightPartOfModel = rightPart_;
	};
	float getStep(){
		return _step;
	};
	virtual vector<float> computeState(float time_, vector<float> state_){};
	

protected:	
	float _step;
	float _time;
	vector<float> _state;
	RightPart* _rightPartOfModel;
};

class EilerMethod: public IntegrationMethod{
public:
    vector<float> computeState(float time_, vector<float> state_){
        vector<float> state(state_.size());
        state = state_ + _step*_rightPartOfModel->rightPart(time_, state_);
        return state;
	};
};

class RungeKuttaMethod: public IntegrationMethod{
public:
	vector<float> computeState(float time_, vector<float> state_){
		vector<float> k(state_.size());
		vector<float> k1(state_.size());
		vector<float> k2(state_.size());
		vector<float> k3(state_.size());
		vector<float> k4(state_.size());
		k1 = _rightPartOfModel->rightPart(time_, state_);
		k2 = _rightPartOfModel->rightPart(time_+0.5*_step, state_+0.5*_step*k1);
		k3 = _rightPartOfModel->rightPart(time_+0.5*_step, state_+0.5*_step*k2);
		k4 = _rightPartOfModel->rightPart(time_+_step, state_+_step*k1);
		k = k1 + 2*(k2+k3) + k4;
		_state += k*_step/6;


		return _state;
	};
};


#endif
