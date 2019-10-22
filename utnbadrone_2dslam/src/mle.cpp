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