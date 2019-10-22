#include <brasenham_line_algorithm.h>

//https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm

void squares_line_low(float x0, float y0, float x1, float y1, std::vector<geometry_msgs::Point32> &point)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float yi = 1;
    float D;
    float y,x;
    geometry_msgs::Point32 point_aux;

    if(dy < 0)
    {
        yi = -1;
        dy = -dy;
    }
    D = 2*dy - dx;
    y = y0;

    point_aux.z = 0;
    for(x=x0;x<x1;x++)
    {
        //plot(x,y);
        point_aux.x = x;
        point_aux.y = y;
        point.push_back(point_aux);
        if(D > 0)
        {
            y = y + yi;
            D = D - 2*dx;
        }
        D = D + 2*dy;
    }
}

void squares_line_high(float x0, float y0, float x1, float y1, std::vector<geometry_msgs::Point32> &point)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float xi = 1;
    float D;
    float y,x;
    geometry_msgs::Point32 point_aux;

    if(dx < 0)
    {
        xi = -1;
        dx = -dx;
    }
    D = 2*dx - dy;
    x = x0;

    for(y = y0; y < y1; y++)
    {
        //plot(x,y);
        point_aux.x = x;
        point_aux.y = y;
        point.push_back(point_aux);
        if(D > 0)
        {
            x = x + xi;
            D = D - 2*dy;
        }
        D = D + 2*dx;
    }
}

void squares_line(float x0, float y0, float x1, float y1, std::vector<geometry_msgs::Point32> &point)
{
    if(abs(y1 - y0) < abs(x1 - x0))
    {
        if(x0 > x1)
        {
            squares_line_low(x1, y1, x0, y0, point);
        }
        else
        {
            squares_line_low(x0, y0, x1, y1, point);            
        }
    }
    else
    {
        if(y0 > y1)
        {
            squares_line_high(x1, y1, x0, y0, point);            
        }
        else
        {
            squares_line_high(x0, y0, x1, y1, point);            
        }
    }
}
