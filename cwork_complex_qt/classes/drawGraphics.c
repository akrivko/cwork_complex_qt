#include <iostream>
#include "gnuplot_i.hpp" //Gnuplot class handles POSIX-Pipe-communikation with Gnuplot


class DrawGraphic{
public:
    
    DrawGraphic(){
        //gPlot = Gnuplot("");
        Gnuplot::set_GNUPlotPath("C:/Programming/gnuplot/bin/");
    };
    void drawGraphic(std::vector<double> x, std::vector<double> y){        
        Gnuplot gPlot;
        gPlot.plot_xy(x,y,"points");
        gPlot.set_smooth().plot_xy(x,y,"cspline");
        //gPlot.unset_smooth();
        int a;
        std::cin >> a;
    };
};





int main(int argc, char* argv[])
{


    DrawGraphic drawG;
    

        std::vector<double> x, y;

        for (int i = 0; i < 50; i++)  // fill double arrays x, y, z
        {
            x.push_back((double)i);    
            y.push_back((double)i*i); 
        }

        drawG.drawGraphic(x,y);

        int a;
        std::cin >> a;


    return 0;
}