#include "mle.h"

float get_likelihood(float x, float m, float s)
{
    static const float inv_sqrt_2pi = 0.3989422804014327;
    float a = (x - m) / s;

    // return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);

    // Asi es como figura en el paper (o lo que entendi)
    // La media deberia ser 0, y x la distancia entre el punto y el feature mas proximo
    return std::exp(-a);
}

void get_occupancy_likelihood(std::vector<int8_t> &likelihood, geometry_msgs::Point32 point, float resolution, float std_x, float std_y)
{
    float Px1x2,Px2x3,Px3x4;    // x likelihood values
    float Py1y2,Py2y3,Py3y4;    // y likelihood values

    float x1,x2,x3,x4;          // x positions of square lines
    float y1,y2,y3,y4;          // y positions of square lines

    x2 = (int(point.x / resolution) * resolution);
    x1 = x2 - resolution;
    x3 = x2 + resolution;
    x4 = x3 + resolution;

    y2 = (int(point.y / resolution) * resolution);
    y1 = y2 - resolution;
    y3 = y2 + resolution;
    y4 = y3 + resolution;

    // Grid Point Occupation - 9 grid cells around the point using Gaussian assumption
    // x-coordinate occupancy likelihoods (Gaussian assumption)
    Px1x2 = gaussian_blur_integral(x1,x2,point.x,std_x);
    Px2x3 = gaussian_blur_integral(x2,x3,point.x,std_x);
    Px3x4 = gaussian_blur_integral(x3,x4,point.x,std_x);

    // y-coordinate occupancy likelihoods (Gaussian assumption)
    Py1y2 = gaussian_blur_integral(y1,y2,point.y,std_y);
    Py2y3 = gaussian_blur_integral(y2,y3,point.y,std_y);
    Py3y4 = gaussian_blur_integral(y3,y4,point.y,std_y);

    // Compute the likelihood of the 9 grid cells around the laser point
    likelihood.push_back(100 * (1 - int8_t(Px1x2 * Py3y4)));    // 0
    likelihood.push_back(100 * (1 - int8_t(Px2x3 * Py3y4)));    // 1
    likelihood.push_back(100 * (1 - int8_t(Px3x4 * Py3y4)));    // 2

    likelihood.push_back(100 * (1 - int8_t(Px1x2 * Py2y3)));    // 3
    likelihood.push_back(100 * (1 - int8_t(Px2x3 * Py2y3)));    // 4
    likelihood.push_back(100 * (1 - int8_t(Px3x4 * Py2y3)));    // 5

    likelihood.push_back(100 * (1 - int8_t(Px1x2 * Py1y2)));    // 6
    likelihood.push_back(100 * (1 - int8_t(Px2x3 * Py1y2)));    // 7
    likelihood.push_back(100 * (1 - int8_t(Px3x4 * Py1y2)));    // 8
}

float gaussian_blur_integral(float a, float b, float c, float std)
{
    // Qian2019 - P.7
    // erf: error function
    return (-0.5 * (erf((c - b)/ (sqrt(2)*std)) - erf((c - a)/ (sqrt(2)*std))));
}