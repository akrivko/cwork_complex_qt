#ifndef _WHITE_NOISE_GENERATOR_
#define _WHITE_NOISE_GENERATOR_


#include "vector_matrix.h"
#include <math.h>
#include <ctime> 


class WhiteNoiseGenerator{
public:	
    WhiteNoiseGenerator(float time_){
		srand(time(0));
    }

	float getNoise(){
		float x = 0;
		float y = 0;
		while (x==0 && y ==0){
			x = rand()/float(RAND_MAX);
			y = rand()/float(RAND_MAX);
		};		
        return cos(2*3.14157*x)*pow(-2*log(y),0.5);
    }
	
};


#endif



// class WhiteNoiseGenerator{
// public:	
// 	WhiteNoiseGenerator(float deltaT_, float time_){
// 		_currentTime = time_;
// 		_deltaT = deltaT_;
// 		_currentValue = 0;
// 		_sigma = pow(1/_deltaT, 0.5);
// 		srand(time(0));
// 	};
// 	float getValue(float time_){
// 		if (_currentTime<=time_ && time_<_currentTime+_deltaT){
// 			return _currentValue;
// 		}
// 		else{
// 			float x = 0;
// 			float y = 0;
// 			while (x==0 && y ==0){
// 				x = rand()/float(RAND_MAX);
// 				y = rand()/float(RAND_MAX);
// 			};
// 			_currentTime = time_;
// 			_currentValue = (cos(2*3.14157*x)*pow(-2*log(y),0.5))*_sigma;
// 			return _currentValue;
// 		};
// 	};

// protected:
// 	float _deltaT;
// 	float _sigma;
// 	float _currentValue;
// 	float _currentTime;
	
// };



//		if (_currentTime<=time_ && time_<_currentTime+_deltaT){
//			return _currentValue;
//		}
//		else{
//			float x = 0;
//			float y = 0;
//			while (x==0 && y ==0){
//				x = rand()/float(RAND_MAX);
//				y = rand()/float(RAND_MAX);
//			};
//			_currentTime = time_;
//            _currentValue = (cos(2*3.14157*x)*pow(-2*log(y),0.5));// * pow(1/_deltaT, 0.5);
//			return _currentValue;
//		};




/*
#ifndef _WHITE_NOISE_GENERATOR_
#define _WHITE_NOISE_GENERATOR_


#include "vector_matrix.h"
#include <math.h>
#include <ctime>


class WhiteNoiseGenerator{
public:
    WhiteNoiseGenerator(float deltaT_, float time_){
        _currentTime = time_;
        _deltaT = deltaT_;
        _currentValue = 0;
        srand(time(0));
    };

    float getValue(float time_){
//		if (_currentTime<=time_ && time_<_currentTime+_deltaT){
//			return _currentValue;
//		}
//		else{
//			float x = 0;
//			float y = 0;
//			while (x==0 && y ==0){
//				x = rand()/float(RAND_MAX);
//				y = rand()/float(RAND_MAX);
//			};
//			_currentTime = time_;
//            _currentValue = (cos(2*3.14157*x)*pow(-2*log(y),0.5));// * pow(1/_deltaT, 0.5);
//			return _currentValue;
//		};
        return getNoise();
    };

    float getNoise(){
        float x = 0;
        float y = 0;
        while (x==0 && y ==0){
            x = rand()/float(RAND_MAX);
            y = rand()/float(RAND_MAX);
        };
        return cos(2*3.14157*x)*pow(-2*log(y),0.5);
    };

protected:
    float _deltaT;
    float _currentValue;
    float _currentTime;

};


#endif



// class WhiteNoiseGenerator{
// public:
// 	WhiteNoiseGenerator(float deltaT_, float time_){
// 		_currentTime = time_;
// 		_deltaT = deltaT_;
// 		_currentValue = 0;
// 		_sigma = pow(1/_deltaT, 0.5);
// 		srand(time(0));
// 	};
// 	float getValue(float time_){
// 		if (_currentTime<=time_ && time_<_currentTime+_deltaT){
// 			return _currentValue;
// 		}
// 		else{
// 			float x = 0;
// 			float y = 0;
// 			while (x==0 && y ==0){
// 				x = rand()/float(RAND_MAX);
// 				y = rand()/float(RAND_MAX);
// 			};
// 			_currentTime = time_;
// 			_currentValue = (cos(2*3.14157*x)*pow(-2*log(y),0.5))*_sigma;
// 			return _currentValue;
// 		};
// 	};

// protected:
// 	float _deltaT;
// 	float _sigma;
// 	float _currentValue;
// 	float _currentTime;

// };

 */
