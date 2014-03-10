#ifndef _WHITE_NOISE_GENERATOR_
#define _WHITE_NOISE_GENERATOR_


#include "vector_matrix.h"
#include <math.h>
#include <ctime> 
#include <chrono>
#include <random>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

 // I don't seed it on purpouse (it's not relevant)





class WhiteNoiseGenerator{
public:	



    WhiteNoiseGenerator(double time_){
		srand(time(0));
    }

    double getNoise(){
//        double x = 0;
//        double y = 0;
//		while (x==0 && y ==0){
//            x = rand()/double(RAND_MAX);
//            y = rand()/double(RAND_MAX);
//		};
//        return cos(2*3.14157*x)*pow(-2*log(y),0.5);
        using namespace boost;
        static mt19937 rng(static_cast<unsigned> (std::time(0)));
        // select Gaussian probability distribution
        normal_distribution<double> norm_dist(0, 1);

        // bind random number generator to distribution, forming a function
        variate_generator<mt19937&, normal_distribution<double> >  normal_sampler(rng, norm_dist);

        // sample from the distribution
        return normal_sampler();
    }
	
};


#endif



// class WhiteNoiseGenerator{
// public:	
// 	WhiteNoiseGenerator(double deltaT_, double time_){
// 		_currentTime = time_;
// 		_deltaT = deltaT_;
// 		_currentValue = 0;
// 		_sigma = pow(1/_deltaT, 0.5);
// 		srand(time(0));
// 	};
// 	double getValue(double time_){
// 		if (_currentTime<=time_ && time_<_currentTime+_deltaT){
// 			return _currentValue;
// 		}
// 		else{
// 			double x = 0;
// 			double y = 0;
// 			while (x==0 && y ==0){
// 				x = rand()/double(RAND_MAX);
// 				y = rand()/double(RAND_MAX);
// 			};
// 			_currentTime = time_;
// 			_currentValue = (cos(2*3.14157*x)*pow(-2*log(y),0.5))*_sigma;
// 			return _currentValue;
// 		};
// 	};

// protected:
// 	double _deltaT;
// 	double _sigma;
// 	double _currentValue;
// 	double _currentTime;
	
// };



//		if (_currentTime<=time_ && time_<_currentTime+_deltaT){
//			return _currentValue;
//		}
//		else{
//			double x = 0;
//			double y = 0;
//			while (x==0 && y ==0){
//				x = rand()/double(RAND_MAX);
//				y = rand()/double(RAND_MAX);
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
    WhiteNoiseGenerator(double deltaT_, double time_){
        _currentTime = time_;
        _deltaT = deltaT_;
        _currentValue = 0;
        srand(time(0));
    };

    double getValue(double time_){
//		if (_currentTime<=time_ && time_<_currentTime+_deltaT){
//			return _currentValue;
//		}
//		else{
//			double x = 0;
//			double y = 0;
//			while (x==0 && y ==0){
//				x = rand()/double(RAND_MAX);
//				y = rand()/double(RAND_MAX);
//			};
//			_currentTime = time_;
//            _currentValue = (cos(2*3.14157*x)*pow(-2*log(y),0.5));// * pow(1/_deltaT, 0.5);
//			return _currentValue;
//		};
        return getNoise();
    };

    double getNoise(){
        double x = 0;
        double y = 0;
        while (x==0 && y ==0){
            x = rand()/double(RAND_MAX);
            y = rand()/double(RAND_MAX);
        };
        return cos(2*3.14157*x)*pow(-2*log(y),0.5);
    };

protected:
    double _deltaT;
    double _currentValue;
    double _currentTime;

};


#endif



// class WhiteNoiseGenerator{
// public:
// 	WhiteNoiseGenerator(double deltaT_, double time_){
// 		_currentTime = time_;
// 		_deltaT = deltaT_;
// 		_currentValue = 0;
// 		_sigma = pow(1/_deltaT, 0.5);
// 		srand(time(0));
// 	};
// 	double getValue(double time_){
// 		if (_currentTime<=time_ && time_<_currentTime+_deltaT){
// 			return _currentValue;
// 		}
// 		else{
// 			double x = 0;
// 			double y = 0;
// 			while (x==0 && y ==0){
// 				x = rand()/double(RAND_MAX);
// 				y = rand()/double(RAND_MAX);
// 			};
// 			_currentTime = time_;
// 			_currentValue = (cos(2*3.14157*x)*pow(-2*log(y),0.5))*_sigma;
// 			return _currentValue;
// 		};
// 	};

// protected:
// 	double _deltaT;
// 	double _sigma;
// 	double _currentValue;
// 	double _currentTime;

// };

 */
