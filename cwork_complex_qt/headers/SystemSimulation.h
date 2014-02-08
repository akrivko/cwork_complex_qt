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
        _step = 10;
        _simulationTime = 1000;
        _beginTime = 0;

        _gps = new Constellation(31);
        _gps->InitializeSatellites(_step);

        _glonass = new Constellation(24);
        _glonass->InitializeSatellites(_step);


        _consumer = new Consumer;

        vector<float> stateReference(6);
        vector<float> stateTrue(6);

        stateReference(0) = 6.871e3;
        stateReference(1) = 0.0;
        stateReference(2) = 0.0;
        stateReference(3) = 0.0;
        stateReference(4) = 3.808;
        stateReference(5) = 6.596;


        float sigmaxyz = 100/1000.0;
        float sigmaVxyz = 1/1000.0;
        WhiteNoiseGenerator whiteNoise(0);
        stateTrue(0) = 6.871e3 + sigmaxyz*whiteNoise.getNoise();
        stateTrue(1) = 0.0 + sigmaxyz*whiteNoise.getNoise();
        stateTrue(2) = 0.0 + sigmaxyz*whiteNoise.getNoise();
        stateTrue(3) = 0.0 + sigmaVxyz*whiteNoise.getNoise();
        stateTrue(4) = 3.808 + sigmaVxyz*whiteNoise.getNoise();
        stateTrue(5) = 6.596 + sigmaVxyz*whiteNoise.getNoise();

//        float dif;
//        for (int i=0; i<6; ++i){
//          dif = stateTrue(i) - stateReference(i);
//        }


        _consumer->setIntegrationMethod(_step, 1);
        _consumer->setInitializeParametrs(_beginTime, stateTrue);
        _consumer->setReferenceState(stateReference);

        _systemSimulateNavigationSignals = new SystemSimulateNavigationSignals;
        _systemSimulateNavigationSignals->setParametrs(_beginTime, _step);
    };


    std::vector<int> spotVisibleSatellites(vector<float> state_consumer, int numberConstellation){
        std::vector<int> visibleSatellites;
        Constellation* currentConstellation;
        if (numberConstellation == 1){
            currentConstellation = _gps;
        }
        if (numberConstellation == 2){
            currentConstellation = _glonass;
        }

        int countSatellites = currentConstellation->getCountSatellites();
        float gamma;
        float alpha;
        vector<float> xyz_consumer(3);
        vector<float> xyzSatellite(3);
        vector<float> xyz_consumerMinusSatellite(3);
        xyz_consumer(0) = state_consumer(0);
        xyz_consumer(1) = state_consumer(1);
        xyz_consumer(2) = state_consumer(2);
        float radius_consumer;
        float radius_consumerSatellite;
        float radiusEarth = 6371;
        for (int i = 0; i < countSatellites; ++i)
        {
            xyzSatellite(0) = vector<float>(currentConstellation->getStateSatellite(i))(0);
            xyzSatellite(1) = vector<float>(currentConstellation->getStateSatellite(i))(1);
            xyzSatellite(2) = vector<float>(currentConstellation->getStateSatellite(i))(2);
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
        float time = _beginTime;
        std::vector<int> numbersOfVisibleSatellites;
        std::vector<float> deltaPseudoDistance;
        std::vector<float> deltaDerivativePseudoDistance;
        std::vector< vector<float> > statesSatellites (0, vector<float>(6));

        std::vector<float> t(0);
        std::vector<float> x(0);
        std::vector<float> y(0);
        std::vector<float> z(0);

        std::vector<float> xr(0);
        std::vector<float> yr(0);
        std::vector<float> zr(0);

        std::vector<float> xlim(0);
        std::vector<float> ylim(0);
        std::vector<float> zlim(0);

        vector<float> referenceStateConsumer(6);
        vector<float> StateConsumer(6);
        matrix<float> estP(6,6);


        int numLimit;

        while(time<=_simulationTime) {

            update();

            //пока только GPS
            numbersOfVisibleSatellites = spotVisibleSatellites(_consumer->getCurrentState(), 1);

            numLimit = 8;//numbersOfVisibleSatellites.size();


            for (int i = 0; i <  numLimit; ++i)
            {
                 deltaPseudoDistance.push_back(
                    _systemSimulateNavigationSignals->computeTrueDistance(
                        time,_consumer->getCurrentState(), _gps->getStateSatellite(numbersOfVisibleSatellites[i]))
                    -
                    _systemSimulateNavigationSignals->computeReferenceDistance(
                        time,_consumer->getCurrentReferenceState(), _gps->getReferenceStateSatellite(numbersOfVisibleSatellites[i]))
                    );

                 deltaDerivativePseudoDistance.push_back(
                    _systemSimulateNavigationSignals->computeTrueDerivativeDistance(
                        time,_consumer->getCurrentState(), _gps->getStateSatellite(numbersOfVisibleSatellites[i]))
                    -
                    _systemSimulateNavigationSignals->computeReferenceDerivativeDistance(
                        time,_consumer->getCurrentReferenceState(), _gps->getReferenceStateSatellite(numbersOfVisibleSatellites[i]))
                    );
                 statesSatellites.push_back(_gps->getReferenceStateSatellite(numbersOfVisibleSatellites[i]));
            };

            _consumer->computeEstimateDeltaState(deltaPseudoDistance,  deltaDerivativePseudoDistance, statesSatellites);
            _consumer->computeNextEstimateState();
            estP = _consumer->getEstP();
            referenceStateConsumer = _consumer->getCurrentState();
            StateConsumer = _consumer->getCurrentEstimateState();
            xr.push_back(referenceStateConsumer(0)-StateConsumer(0));
            yr.push_back(referenceStateConsumer(1)-StateConsumer(1));
            zr.push_back(referenceStateConsumer(2)-StateConsumer(2));

            xlim.push_back(3*sqrt(estP(0,0)));
            ylim.push_back(3*sqrt(estP(1,1)));
            zlim.push_back(3*sqrt(estP(2,2)));

            vector<float> deltaStateFK(6);
            deltaStateFK = _consumer->getDeltaStateEstimateFK();
            t.push_back(time);
            x.push_back(deltaStateFK(0));//StateConsumer(0));
            y.push_back(deltaStateFK(1));
            z.push_back(deltaStateFK(2));

            std::cout<<numLimit<<std::endl;

            std::cout<<time<<std::endl;
            time += _step;

            deltaPseudoDistance.clear();
            deltaDerivativePseudoDistance.clear();
            statesSatellites.clear();

        }
        DrawGraphic drawG;
        drawG.drawXY(t,x, xlim);
        drawG.drawXY(t,y, ylim);
        drawG.drawXY(t,z, zlim);
        drawG.drawXY(x,y);

    }

protected:
    float _simulationTime;
    float _beginTime;
    float _step;
    Constellation* _gps;
    Constellation* _glonass;
    Consumer* _consumer;
    SystemSimulateNavigationSignals* _systemSimulateNavigationSignals;

};

#endif
