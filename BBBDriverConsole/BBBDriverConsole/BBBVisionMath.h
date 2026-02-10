#pragma once

#include <vector>
#include <cstdint>

namespace BBB
{
    class VisionMath
    {
    public:
        // devolvemos v clamped entre 0 y 1
        static float Clamp01(float v);

        // convertimos grados a radianes
        static float DegToRad(float deg);

        // calculamos altura sobre el suelo usando geometria
        // sistema camara X derecha Y abajo Z delante
        // altura camara en metros y pitch hacia abajo en grados
        static float HeightAboveGroundM(float Xc, float Yc, float Zc, float camHeightM, float pitchDownDeg);

        // pasamos HSV a RGB para pintar por distancia
        static void HsvToRgb(float hDeg, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b);

        // pintamos segun Z con un heatmap
        static void DepthToHeatRGB(float z, float zMin, float zMax, uint8_t& r, uint8_t& g, uint8_t& b);

        // calculamos percentil q 0 a 1
        // ojo modifica el vector porque lo ordenamos
        static float Percentile(std::vector<float>& v, float q);
    };
}
