#include "bin.h"
#include <iostream>
#include "segment.h"
#include <cmath>
typedef std::pair<double, double> lineParam;

struct point
{
    /* data */
    double x, y;
    point(double x, double y):x(x),y(y){}
};

int main()
{
    point p1(0, -1), p2(1, 0), p3(1, 1);
    lineParam line;
    line.first = (p2.y - p1.y) / (p2.x - p1.x);
    line.second = p1.y - line.first * p1.x;

    std::cout << "params :" << line.first << "  " << line.second << std::endl; 

    double dist = std::abs(line.first * p3.x - p3.y + line.second) /
                        sqrt(1 + line.first * line.first);
    std::cout << "dist = " << dist << "\n1 / sqrt(2) = " << 1 / sqrt(2) << std::endl;


    printf("60 / 180 = %f \n", 60. / 180);
    printf("cos(60 / 180 * M_PI) %f\n", cos(60. / 180 * M_PI));
    std::cout << "test code finished !" << std::endl;   
    return 0;
}