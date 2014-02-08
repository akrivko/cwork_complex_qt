#include <QCoreApplication>

#include <iostream>
#include <fstream>
#include <vector>
#include "headers/Model.h"
#include "headers/vector_matrix.h"
#include "headers/DrawGraphics.h"
#include "headers/Constellation.h"

#include "headers/WhiteNoiseGenerator.h"
#include "headers/SystemSimulateNavigationSignals.h"
#include "headers/SystemSimulation.h"
#include "headers/KalmanFilter.h"


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    SystemSimulation* ss;

    ss = new SystemSimulation();

    ss->simulate();


/*
    WhiteNoiseGenerator WN(0);
    std::vector<float> t(0);
    std::vector<float> x(0);
    for (int i=0; i<1000; ++i){
        t.push_back(i);
        x.push_back(WN.getNoise());
    }

    DrawGraphic drawG;

    drawG.drawXY(t,x);
*/


/*
     float step = 10;
     float time=0;

     vector<float> state(1);
     state(0) = 0;

     FormingFilter ff;
     ff.setInitializeParametrs(time, state);
     ff.setIntegrationMethod(step, 1);
     ff.setParametrsDistributionFormingFilter(0, 10);
     ff.setParametrsForWhiteNoise(step, time);
     WhiteNoiseGenerator WN(0);
     std::vector<float> t(0);
     std::vector<float> x(0);

     while (time<500)
     {
        state = ff.getNextState();
        t.push_back(time);
        x.push_back(state(0));
        time+=step;
     };


     DrawGraphic drawG;
     drawG.drawXY(t,x);
*/



/*
    Consumer* consumer;
    consumer = new Consumer();

    float step = 1;
    vector<float> state0(6), state(6);
    state0(0) = 6871;
    state0(1) = 0;
    state0(2) = 0;
    state0(3) = 0;
    state0(4) = 3.8;
    state0(5) = 6.5;

    consumer->setInitializeParametrs(0,state0);
    consumer->setIntegrationMethod(step, 1);
    state = consumer->getNextState();

    std::vector<float> t(0);
    std::vector<float> x(0);
    std::vector<float> y(0);
    std::vector<float> z(0);

    float time = 0;

    while (time<6000)
    {
        t.push_back(time);
        state = consumer->getNextState();
        x.push_back(state(0));
        y.push_back(state(1));
        z.push_back(state(2));
        time+=step;
    };

    DrawGraphic drawG;
    drawG.drawXYZ(x, y, z);
*/


    return a.exec();
}
