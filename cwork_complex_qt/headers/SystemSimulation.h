#ifndef _SYSTEM_SIMULATION_
#define _SYSTEM_SIMULATION_


#include "vector_matrix.h"
#include <math.h>
#include "headers/SystemSimulateNavigationSignals.h"
#include "headers/Model.h"
#include "headers/Constellation.h"
#include "headers/WhiteNoiseGenerator.h"


class SystemSimulation{
public:
    SystemSimulation(){
        _step = 0.1;
        _simulationTime = 1000;
        _beginTime = 0;

        _gps = new Constellation(31);
        _gps->InitializeSatellites(_step);

        _glonass = new Constellation(24);
        _glonass->InitializeSatellites(_step);


        _consumer = new Consumer;

        vector<double> stateReference(6);
        vector<double> stateTrue(6);

        stateReference(0) = 6871e3;
        stateReference(1) = 0.0;
        stateReference(2) = 0.0;
        stateReference(3) = 0.0;
        stateReference(4) = 3811.78;
        stateReference(5) = 6594.11;


        double sigmaxyz = 100.0;
        double sigmaVxyz = 1.0;
        WhiteNoiseGenerator whiteNoise(0);
        stateTrue(0) = 6871e3  + sigmaxyz*whiteNoise.getNoise();
        stateTrue(1) = 0.0 + sigmaxyz*whiteNoise.getNoise();
        stateTrue(2) = 0.0 + sigmaxyz*whiteNoise.getNoise();
        stateTrue(3) = 0.0 + sigmaVxyz*whiteNoise.getNoise();
        stateTrue(4) = 3811.78 + sigmaVxyz*whiteNoise.getNoise();
        stateTrue(5) = 6594.11 + sigmaVxyz*whiteNoise.getNoise();


        _consumer->setInitializeParametrs(_beginTime, stateTrue);
        _consumer->setReferenceState(stateReference);
        _consumer->setIntegrationMethod(_step, 2);


        for (int i = 0; i < 31; ++i)
        {
            _SSNSGps.push_back(SystemSimulateNavigationSignals());
            _SSNSGps[i].setParametrs(_beginTime, _step);
        }
        for (int i = 0; i < 24; ++i)
        {
            _SSNSGlonass.push_back(SystemSimulateNavigationSignals());
            _SSNSGlonass[i].setParametrs(_beginTime, _step);
        }
    }


    std::vector<int> spotVisibleSatellites(vector<double> state_consumer, int numberConstellation){
        std::vector<int> visibleSatellites;
        Constellation* currentConstellation;
        if (numberConstellation == 1){
            currentConstellation = _gps;
        }
        if (numberConstellation == 2){
            currentConstellation = _glonass;
        }

        int countSatellites = currentConstellation->getCountSatellites();
        double gamma;
        double alpha;
        vector<double> xyz_consumer(3);
        vector<double> xyzSatellite(3);
        vector<double> xyz_consumerMinusSatellite(3);
        xyz_consumer(0) = state_consumer(0);
        xyz_consumer(1) = state_consumer(1);
        xyz_consumer(2) = state_consumer(2);
        double radius_consumer;
        double radius_consumerSatellite;
        double radiusEarth = 6371.0*1000;
        for (int i = 0; i < countSatellites; ++i)
        {
            xyzSatellite(0) = vector<double>(currentConstellation->getStateSatellite(i))(0);
            xyzSatellite(1) = vector<double>(currentConstellation->getStateSatellite(i))(1);
            xyzSatellite(2) = vector<double>(currentConstellation->getStateSatellite(i))(2);
            for (int j = 0; j < 3; ++j)
            {
                xyz_consumerMinusSatellite(j) = xyz_consumer(j) - xyzSatellite(j);
            }

            radius_consumer = pow(inner_prod(xyz_consumer, xyz_consumer), 0.5);
            radius_consumerSatellite = pow(inner_prod(xyz_consumerMinusSatellite, xyz_consumerMinusSatellite), 0.5);
            gamma = acos(inner_prod(xyz_consumer, xyz_consumerMinusSatellite)/(radius_consumer*radius_consumerSatellite));

            alpha = asin(radiusEarth/radius_consumer);

            if (gamma>=alpha){
                visibleSatellites.push_back(i);
            };
        };
        return visibleSatellites;
    }

    void update(){
        _gps->computeNextState();
        _glonass->computeNextState();

        _consumer->computeNextState();
        _consumer->computeNextReferenceState();
    }


    void simulate(){
        double time = _beginTime;
        std::vector<int> numbersOfVisibleSatellitesGps;
        std::vector<int> numbersOfVisibleSatellitesGlonass;
        int numGps = 8, numGlonass = 8;
        std::vector<double> deltaPseudoDistance;
        std::vector<double> deltaDerivativePseudoDistance;
        std::vector< vector<double> > statesSatellites (0, vector<double>(6));

        std::vector<double> t(0);

        std::vector<double> xr(0);
        std::vector<double> yr(0);
        std::vector<double> zr(0);
        std::vector<double> vxr(0);
        std::vector<double> vyr(0);
        std::vector<double> vzr(0);
        std::vector<double> xlim(0);
        std::vector<double> ylim(0);
        std::vector<double> zlim(0);
        std::vector<double> vxlim(0);
        std::vector<double> vylim(0);
        std::vector<double> vzlim(0);
        std::vector<double> xlim2(0);
        std::vector<double> ylim2(0);
        std::vector<double> zlim2(0);
        std::vector<double> vxlim2(0);
        std::vector<double> vylim2(0);
        std::vector<double> vzlim2(0);

        vector<double> referenceStateConsumer(6);
        vector<double> StateConsumer(6);
        vector<double> deltaStateFK(6);

        matrix<double> estP(6,6);
        int temp = 0 ;

        int numLimit;

        while(time<=_simulationTime) {

            update();

            //пока только GPS
            numbersOfVisibleSatellitesGps =     spotVisibleSatellites(_consumer->getCurrentState(), 1);
            numbersOfVisibleSatellitesGlonass = spotVisibleSatellites(_consumer->getCurrentState(), 2);

            //            numLimit = 10;//numbersOfVisibleSatellites.size();

            if (temp%100 == 0){
                for (int i = 0; i < numGps; ++i)
                {
                    deltaPseudoDistance.push_back(
                            _SSNSGps[numbersOfVisibleSatellitesGps[i]].computeTrueDistance(
                            time,
                            _consumer->getCurrentState(),
                            _gps->getStateSatellite(numbersOfVisibleSatellitesGps[i]))
                            -
                            _SSNSGps[numbersOfVisibleSatellitesGps[i]].computeReferenceDistance(
                            time,_consumer->getCurrentReferenceState(), _gps->getReferenceStateSatellite(numbersOfVisibleSatellitesGps[i]))
                            );

                    deltaDerivativePseudoDistance.push_back(
                            _SSNSGps[numbersOfVisibleSatellitesGps[i]].computeTrueDerivativeDistance(
                            time,_consumer->getCurrentState(), _gps->getStateSatellite(numbersOfVisibleSatellitesGps[i]))
                            -
                            _SSNSGps[numbersOfVisibleSatellitesGps[i]].computeReferenceDerivativeDistance(
                            time,_consumer->getCurrentReferenceState(), _gps->getReferenceStateSatellite(numbersOfVisibleSatellitesGps[i]))
                            );
                    statesSatellites.push_back(_gps->getReferenceStateSatellite(numbersOfVisibleSatellitesGps[i]));

                };


                for (int i = 0; i < numGlonass; ++i)
                {
                    deltaPseudoDistance.push_back(
                            _SSNSGlonass[numbersOfVisibleSatellitesGlonass[i]].computeTrueDistance(
                            time,
                            _consumer->getCurrentState(),
                            _gps->getStateSatellite(numbersOfVisibleSatellitesGlonass[i]))
                            -
                            _SSNSGlonass[numbersOfVisibleSatellitesGlonass[i]].computeReferenceDistance(
                            time,_consumer->getCurrentReferenceState(), _gps->getReferenceStateSatellite(numbersOfVisibleSatellitesGlonass[i]))
                            );

                    deltaDerivativePseudoDistance.push_back(
                            _SSNSGlonass[numbersOfVisibleSatellitesGlonass[i]].computeTrueDerivativeDistance(
                            time,_consumer->getCurrentState(), _gps->getStateSatellite(numbersOfVisibleSatellitesGlonass[i]))
                            -
                            _SSNSGlonass[numbersOfVisibleSatellitesGlonass[i]].computeReferenceDerivativeDistance(
                            time,_consumer->getCurrentReferenceState(), _gps->getReferenceStateSatellite(numbersOfVisibleSatellitesGlonass[i]))
                            );
                    statesSatellites.push_back(_gps->getReferenceStateSatellite(numbersOfVisibleSatellitesGlonass[i]));

                };

                _consumer->computeEstimateDeltaState(deltaPseudoDistance,  deltaDerivativePseudoDistance, statesSatellites);
                temp = 0;

                estP = _consumer->getEstP();

                referenceStateConsumer = _consumer->getCurrentReferenceState();
                StateConsumer = _consumer->getCurrentState();

                deltaStateFK = StateConsumer - referenceStateConsumer;

                xr.push_back(deltaStateFK(0));
                yr.push_back(deltaStateFK(1));
                zr.push_back(deltaStateFK(2));
                vxr.push_back(deltaStateFK(3));
                vyr.push_back(deltaStateFK(4));
                vzr.push_back(deltaStateFK(5));

                xlim.push_back(3*sqrt(estP(0,0)));
                ylim.push_back(3*sqrt(estP(1,1)));
                zlim.push_back(3*sqrt(estP(2,2)));

                vxlim.push_back(3*sqrt(estP(3,3)));
                vylim.push_back(3*sqrt(estP(4,4)));
                vzlim.push_back(3*sqrt(estP(5,5)));

                xlim2.push_back(-3*sqrt(estP(0,0)));
                ylim2.push_back(-3*sqrt(estP(1,1)));
                zlim2.push_back(-3*sqrt(estP(2,2)));

                vxlim2.push_back(-3*sqrt(estP(3,3)));
                vylim2.push_back(-3*sqrt(estP(4,4)));
                vzlim2.push_back(-3*sqrt(estP(5,5)));
                t.push_back(time);
            }
            temp++;

            std::cout << time << std::endl;
            time += _step;


            deltaPseudoDistance.clear();
            deltaDerivativePseudoDistance.clear();
            statesSatellites.clear();

        }

        DrawGraphic drawG;
        drawG.drawXY(t,xr, xlim, xlim2);
        drawG.drawXY(t,yr, ylim, ylim2);
        drawG.drawXY(t,zr, zlim, zlim2);
        drawG.drawXY(t,vxr, vxlim, vxlim2);
        drawG.drawXY(t,vyr, vylim, vylim2);
        drawG.drawXY(t,vzr, vzlim, vzlim2);
    }

protected:
    double _simulationTime;
    double _beginTime;
    double _step;
    Constellation* _gps;
    Constellation* _glonass;
    Consumer* _consumer;
    std::vector<SystemSimulateNavigationSignals> _SSNSGps;
    std::vector<SystemSimulateNavigationSignals> _SSNSGlonass;

};

#endif
