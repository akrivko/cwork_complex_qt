#ifndef _CONSTELLATION_
#define _CONSTELLATION_

#include <fstream>
#include <iostream>
#include "WhiteNoiseGenerator.h"
#include "Model.h"


class Constellation{
public:
	Constellation(int countSatellites_){
		_countSatellites = countSatellites_;
		for (int i = 0; i < countSatellites_; ++i)
		{
			_satellites.push_back(Satellite());
		}		
    }

	void InitializeSatellites(float stepIntegration_){
        WhiteNoiseGenerator whiteNoise(0);
		std::ifstream F1;
		std::ifstream F2;

		float sigmaxyz;
		float sigmaVxyz;
		
		if (_countSatellites == 31)
		{
			sigmaxyz = 10/1000.0;
			sigmaVxyz = 0.1/1000.0;
			F1.open("src/ephes_gps_1.txt", std::ios::in);
			F2.open("src/ephes_gps_2.txt", std::ios::in);
		};
		if (_countSatellites == 24)
		{
			sigmaxyz = 13/1000.0;
			sigmaVxyz = 0.3/1000.0;
			F1.open("src/ephes_glonass_1.txt", std::ios::in);
			F2.open("src/ephes_glonass_2.txt", std::ios::in);
		};
		
		float omegaEarth = 7.2921158553*1e-5; //рад в сек
		float theta = omegaEarth*6*3600;
	
		vector<float> stateSatellite(6);		

		float x1,y1,z1,t1;
		float x2,y2,z2,t2;
		float xAbs1,yAbs1,zAbs1;
		float xAbs2,yAbs2,zAbs2;
		float vx, vy, vz;

		for (int i = 0; i < _countSatellites; ++i)
		{	
			
			F1>>x1;
			F1.eof();
			F1>>y1;
			F1.eof();
			F1>>z1;
			F1.eof();
						
			F2>>x2;
			F2.eof();
			F2>>y2;
			F2.eof();
			F2>>z2;	
            F2.eof();

			
			float dt = 10.0;
			
			vx = (x2-x1)/dt;
			vy = (y2-y1)/dt;
			vz = (z2-z1)/dt;

			xAbs1 = x1*cos(theta)-y1*sin(theta);
			yAbs1 = x1*sin(theta)+y1*cos(theta);
			zAbs1 = z1;

			xAbs2 = x2*cos(theta)-y2*sin(theta);
			yAbs2 = x2*sin(theta)+y2*cos(theta);
			zAbs2 = z2;


			stateSatellite(0) = (xAbs1+xAbs2)/2.0;
			stateSatellite(1) = (yAbs1+yAbs2)/2.0;
			stateSatellite(2) = (zAbs1+zAbs2)/2.0;			

			stateSatellite(3) = vx*cos(theta)-vy*sin(theta)-omegaEarth*yAbs1;
			stateSatellite(4) = vx*sin(theta)-vy*cos(theta)+omegaEarth*xAbs1;
			stateSatellite(5) = vz;

			
            _satellites[i].setReferenceState(stateSatellite);

            stateSatellite(0) += sigmaxyz*whiteNoise.getNoise();
			stateSatellite(1) += sigmaxyz*whiteNoise.getNoise();
			stateSatellite(2) += sigmaxyz*whiteNoise.getNoise();			

			stateSatellite(3) += sigmaVxyz*whiteNoise.getNoise();
			stateSatellite(4) += sigmaVxyz*whiteNoise.getNoise();
            stateSatellite(5) += sigmaVxyz*whiteNoise.getNoise();

			_satellites[i].setInitializeParametrs(0, stateSatellite); //передавать время
            _satellites[i].setIntegrationMethod(stepIntegration_, 2); //RungeKutta
		};
    }

	void computeNextState(){
		for (int i = 0; i < _countSatellites; ++i)
		{
			_satellites[i].computeNextReferenceState();
			_satellites[i].computeNextState();			
		};
    }

	vector<float> getStateSatellite(int numSatellite){
		vector<float> stateSatellite(6);
		stateSatellite = _satellites[numSatellite].getCurrentState();
		return stateSatellite;
    }

	vector<float> getReferenceStateSatellite(int numSatellite){
		vector<float> stateSatellite(6);
		stateSatellite = _satellites[numSatellite].getCurrentReferenceState();
		return stateSatellite;
    }

	int getCountSatellites(){
		return _countSatellites;
    }

protected:
	int _countSatellites;
	std::vector<Satellite> _satellites;
	
};


#endif
