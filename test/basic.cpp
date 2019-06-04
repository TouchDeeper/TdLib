#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include "TdLibrary/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() 
{
    // Prepare data.
    int n = 5000;
    std::vector<double> x(n), y(n), z(n), w(n,2);

    for(int i=0; i<n; i++) {
        x.at(i) = -2 + 4.0 / double(n) * i;
        y.at(i) = x[i] * x[i] - 10 * cos(2.0 * M_PI * x[i]);
//        z.at(i) = log(i);
    }
    
    // Set the size of output image = 1200x780 pixels
    plt::figure_size(1200, 780);

    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(x, y);

    // Plot a red dashed line from given x and y data.
    plt::plot(x, w,"r--");

    // Plot a line whose name will show up as "log(x)" in the legend.
    plt::named_plot("y(x)", x, y);

    // Set x-axis to interval [0,1000000]
    plt::xlim(-2, 2);

    // Add graph title
    plt::title("Sample figure");

    // Enable legend.
    plt::legend();

    // save figure
    const char* filename = "./basic.png";
    std::cout << "Saving result to " << filename << std::endl;;
    plt::save(filename);
}
