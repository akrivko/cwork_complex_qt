#include <QCoreApplication>

#include <iostream>
#include <fstream>
#include <vector>
#include "headers/vector_matrix.h"
#include "headers/SystemSimulation.h"
#include "headers/Model.h"
#include "headers/DrawGraphics.h"
#include "headers/Constellation.h"
#include "headers/WhiteNoiseGenerator.h"
#include "headers/SystemSimulateNavigationSignals.h"
#include "headers/KalmanFilter.h"
#include <QFile>
#include <QTextStream>


vector<double> fromKeplerToCartesian(double a, double e, double i, double Omega, double omega, double M){

    double mu = 398600.436e+9;

    int E = M;
    /*ToDo
     *расчет E для M != 0
     */

    vector<double> P(3), Q(3);

    P(0) = cos(omega)*cos(Omega) - sin(omega)*cos(i)*sin(Omega);
    P(1) = cos(omega)*sin(Omega) + sin(omega)*cos(i)*cos(Omega);
    P(2) = sin(omega)*sin(i);

    Q(0) = -sin(omega)*cos(Omega) - cos(omega)*cos(i)*sin(Omega);
    Q(1) = -sin(omega)*sin(Omega) + cos(omega)*cos(i)*cos(Omega);
    Q(2) = sin(i)*cos(omega);

    vector<double> R(3), V(3);

    R = a*(cos(E)-e)*P + a*sqrt(1-e*e)*sin(E)*Q;

    V = 1/(1-e*cos(E))*sqrt(mu/a)*(-e*sin(E)*P+sqrt(1-e*e)*cos(E)*Q);

    vector<double> res(6);
    res(0) = R(0);
    res(1) = R(1);
    res(2) = R(2);

    res(3) = V(0);
    res(4) = V(1);
    res(5) = V(2);

    return res;

}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    std::cout << "Modelling time, c" << std::endl;
    double time;
    std::cin >> time;

    SystemSimulation* ss;
    ss = new SystemSimulation(time);
    ss->simulate();

    return a.exec();
}

/*

*/


/*  vector<double> state(6);

  state = fromKeplerToCartesian(6371e3+500e3, 0, 3.14/3.0, 0, 0, 0);

  std::cout<<state;

*/

/*   vector<double> s(3);
  s(0) = 1;
  s(1) = 2;
  s(2) = 3;

  matrix<double> m(3,3), mi(3,3), mii(3,3);
  m(0,0) = 693; m(0,1) = 357; m(0,2) = 657;
  m(1,0) = 8; m(1,1) = 1085; m(1,2) = 234;
  m(2,0) = 19; m(2,1) = 349; m(2,2) = 34;

  InvertMatrix(m, mi);
  InvertMatrix(mi, mii);
  std::cout<<mi;
  std::cout<<mii;
  std::cout<<m-mii;
*/
/*    matrix<double> m1(2,2), m2(2,2) ;
  m1(0,0) = 3; m1(0,1) = 7;
  m1(1,0) = 8; m1(1,1) = 10;

  m2(0,0) = 6; m2(0,1) = 2;
  m2(1,0) = 0; m2(1,1) = 1;
  //InvertMatrix(m1, m2);
  std::cout<<trans(m1);
*/

//    std::vector<float> num(0);
//    num.push_back(1);
//    num.push_back(2);
//    num.push_back(3);

//    std::cout<<num[2];
//    std::cout<<num[1];




//test constellation
/*
    double step = 1;
    Constellation* _gps;
    _gps = new Constellation(24);
    _gps->InitializeSatellites(step);

    vector<double> s(6);

    std::vector<double> t(0);
    std::vector<double> x(0);
    std::vector<double> y(0);
    std::vector<double> z(0);

    double time = 0;
    int i=0;

double R;
    while (time<3600*12*2)
    {
        //for (int j=0; j<31; ++j){
        s = _gps->getStateSatellite(17);// - _gps->getReferenceStateSatellite(17);
        if (i%100 == 0){
            t.push_back(time);
            R = sqrt(s(0)*s(0)+s(1)*s(1)+s(2)*s(2));
            x.push_back(R);
            y.push_back(s(1));
            z.push_back(s(2));

        };
        //}
        //for (int j=0; j<10; ++j){
        _gps->computeNextState();
        //}
        time+=step;
        i++;
    };

    DrawGraphic drawG;
    drawG.drawXY(t, x);
    drawG.drawXY(t, y);
    drawG.drawXY(t, z);
    drawG.drawXYZ(x, y, z);
//*/




 /*
    WhiteNoiseGenerator WN(0);
    std::vector<double> t(0);
    std::vector<double> x(0);
    for (int i=0; i<1000; ++i){
        t.push_back(i);
        x.push_back(WN.getNoise());
    }

    DrawGraphic drawG;

    drawG.drawXY(t,x);
*/


/*
     double step = 0.1;
     double time=0;

     vector<double> state(1), state2(1);
     state(0) = 0;

     FormingFilter ff;
     ff.setInitializeParametrs(time, state);
     ff.setIntegrationMethod(step, 1);
     ff.setParametrsDistributionFormingFilter(0, 10);
     ff.setParametrsForWhiteNoise(step, time);
     FormingFilter ff2;
     ff2.setInitializeParametrs(time, state);
     ff2.setIntegrationMethod(step, 1);
     ff2.setParametrsDistributionFormingFilter(0, 10);
     ff2.setParametrsForWhiteNoise(step, time);
     std::vector<double> t(0);
     std::vector<double> x(0);
     std::vector<double> x2(0);

     while (time<500)
     {
        state = ff.getNextState();
        state2 = ff2.getNextState();

        t.push_back(time);
        x.push_back(state(0));
        x2.push_back(state2(0));
        time+=step;
     };


     DrawGraphic drawG;
     drawG.drawXY(t,x, x2);

/*



    Consumer* consumer;
    consumer = new Consumer();

    double step = 1;
    vector<double> state(6), stateRef(6);
    state(0) = 6871e3;
    state(1) = 0;
    state(2) = 0;
    state(3) = 0;
    state(4) = 3811.78;
    state(5) = 6594.11;


    consumer->setInitializeParametrs(0,state);
    consumer->setIntegrationMethod(step, 2);



    double sigmaxyz = 100.0;
    double sigmaVxyz = 1.0;
    WhiteNoiseGenerator whiteNoise(0);
    state(0) = 6871e3  + sigmaxyz*whiteNoise.getNoise();
    state(1) = 0.0 + sigmaxyz*whiteNoise.getNoise();
    state(2) = 0.0 + sigmaxyz*whiteNoise.getNoise();
    state(3) = 0.0 + sigmaVxyz*whiteNoise.getNoise();
    state(4) = 3811.78 + sigmaVxyz*whiteNoise.getNoise();
    state(5) = 6594.11 + sigmaVxyz*whiteNoise.getNoise();

    consumer->setReferenceState(state);

    vector<double> s(6);

    std::vector<double> t(0);
    std::vector<double> x(0), x2(0);
    std::vector<double> y(0), y2(0);
    std::vector<double> z(0), z2(0);

    double time = 0;

    QFile log("log.dat");

    if(!log.open(QIODevice::Text | QFile::WriteOnly))
    {
        return 100;
    }

    QTextStream stream(&log);
    double R;
    while (time<6000)
    {
        t.push_back(time);
        state = consumer->getNextState();
        stateRef = consumer->getNextReferenceState();
        s = stateRef-state;
        x.push_back(s(0));
        y.push_back(state(1));

        R = sqrt(state(0)*state(0)+state(1)*state(1)+state(2)*state(2));

        z.push_back(state(2));
        x2.push_back(stateRef(0));
        y2.push_back(stateRef(1));
        z2.push_back(stateRef(2));

//        stream<<QString("%1 %2 %3 %4 %5 %6 %7\n")
//                .arg(state(0),0,'g',10)
//                .arg(state(1),0,'g',10)
//                .arg(state(2),0,'g',10)
//                .arg(stateRef(0),0,'g',10)
//                .arg(stateRef(1),0,'g',10)
//                .arg(stateRef(2),0,'g',10)
//                .arg(time,0,'g',10);
//        stream.flush();
        time += step;
    };

    log.close();

    DrawGraphic drawG;
    drawG.drawXYZ(x, y, z);
    drawG.drawXY(t, x);
    drawG.drawXY(t, y);
    drawG.drawXY(t, z);

/**//**/
