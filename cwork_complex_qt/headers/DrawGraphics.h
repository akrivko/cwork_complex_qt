#ifndef _DRAW_GRAPHIC_H_
#define _DRAW_GRAPHIC_H_

#include "source\gnuplot_i.hpp"

class DrawGraphic{
public:    
    
    DrawGraphic(){        
        Gnuplot::set_GNUPlotPath("C:/Programming/gnuplot/bin/");
    };

    void drawXY(std::vector<double> x, std::vector<double> y){
        Gnuplot gPlot;
        gPlot.plot_xy(x,y,"points");
        gPlot.set_smooth().plot_xy(x,y,"cspline");
        int a;
        std::cin >> a;
    };

    void drawXY(std::vector<double> x, std::vector<double> y1, std::vector<double> y2){
        Gnuplot gPlot;
        gPlot.plot_xy(x,y1,"points");
        gPlot.plot_xy(x,y2,"points");        
        gPlot.set_smooth().plot_xy(x,y1,"cspline");
        gPlot.set_smooth().plot_xy(x,y2,"cspline");
        int a;
        std::cin >> a;
    };

    void drawXY(std::vector<double> x, std::vector<double> y1, std::vector<double> y2, std::vector<double> y3){
        Gnuplot gPlot;
        gPlot.set_smooth().plot_xy(x,y1,"cspline");
        gPlot.set_smooth().plot_xy(x,y2,"cspline");
        gPlot.set_smooth().plot_xy(x,y3,"cspline");
        int a;
        std::cin >> a;
    };
    
    void drawXYZ(std::vector<double> x, std::vector<double> y, std::vector<double> z){
        Gnuplot gPlot;
        gPlot.plot_xyz(x,y,z,"points"); 
        int a;
        std::cin >> a;
    };

    void drawXYZ(std::vector<double> x, std::vector<double> y, std::vector<double> z, std::vector<double> x2, std::vector<double> y2, std::vector<double> z2){
        Gnuplot gPlot;
        gPlot.plot_xyz(x,y,z,"liness");
        gPlot.plot_xyz(x2,y2,z2,"lines");
        int a;
        std::cin >> a;
    };
};



#endif
