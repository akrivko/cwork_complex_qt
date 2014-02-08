#ifndef _DRAW_GRAPHIC_H_
#define _DRAW_GRAPHIC_H_

#include "source\gnuplot_i.hpp"

class DrawGraphic{
public:    
    
    DrawGraphic(){        
        Gnuplot::set_GNUPlotPath("C:/Programming/gnuplot/bin/");
    };

    void drawXY(std::vector<float> x, std::vector<float> y){        
        Gnuplot gPlot;
        gPlot.plot_xy(x,y,"points");
       // gPlot.set_smooth().plot_xy(x,y,"cspline");        
        int a;
        std::cin >> a;
    };

    void drawXY(std::vector<float> x, std::vector<float> y1, std::vector<float> y2){        
        Gnuplot gPlot;
        gPlot.plot_xy(x,y1,"points");
        gPlot.plot_xy(x,y2,"points");        
        int a;
        std::cin >> a;
    };
    
    void drawXYZ(std::vector<float> x, std::vector<float> y, std::vector<float> z){        
        Gnuplot gPlot;
        gPlot.plot_xyz(x,y,z,"points"); 
        int a;
        std::cin >> a;
    };
};

#endif