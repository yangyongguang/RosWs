#include "bin.h"
#include <iostream>
#include "segment.h"
#include <cmath>
#include <list>
typedef std::pair<double, double> lineParam;

struct point
{
    /* data */
    double x, y;
    point(double x, double y):x(x),y(y){}
};

int main()
{
    // point p1(0, -1), p2(1, 0), p3(1, 1);
    // lineParam line;
    // line.first = (p2.y - p1.y) / (p2.x - p1.x);
    // line.second = p1.y - line.first * p1.x;

    // std::cout << "params :" << line.first << "  " << line.second << std::endl; 

    // double dist = std::abs(line.first * p3.x - p3.y + line.second) /
    //                     sqrt(1 + line.first * line.first);
    // std::cout << "dist = " << dist << "\n1 / sqrt(2) = " << 1 / sqrt(2) << std::endl;


    // printf("60 / 180 = %f \n", 60. / 180);
    // printf("cos(60 / 180 * M_PI) %f\n", cos(60. / 180 * M_PI));
    // std::cout << "test code finished !" << std::endl;   

    // while(1)
    // { 
    //     int idxRes = 0;
    //     double d;
    //     std::cin >> d;
    //     double hSensor = 1.73;
    //     double theta_start = 65.1277;
    //     double angle_resolution = 0.41;
    //     printf("atan2(d, hSensor) %f\n", atan2(d, hSensor) * 180 / M_PI);
    //     idxRes = (atan2(d, hSensor) * 180 / M_PI - theta_start) / angle_resolution;
    //     printf("idxRes %d\n", idxRes);
    // }
    std::list<int> l = {1,2,3,4,5,6,7,8,9};
    for (auto it = l.begin(); it != l.end();)
    {
        if (*it == 3)
            l.erase(it++);
        else
            ++it;
    }

    for (auto it = l.begin(); it != l.end(); ++it)
    {
        printf("%d ", *it);
    }
    printf("\n");
    return 0;
}

