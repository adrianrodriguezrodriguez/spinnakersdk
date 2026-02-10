#include "BBBVisionMath.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace BBB
{
    float VisionMath::Clamp01(float v)
    {
        if (v < 0.0f) return 0.0f;
        if (v > 1.0f) return 1.0f;
        return v;
    }

    float VisionMath::DegToRad(float deg)
    {
        return deg * 3.14159265358979323846f / 180.0f;
    }

    float VisionMath::HeightAboveGroundM(float Xc, float Yc, float Zc, float camHeightM, float pitchDownDeg)
    {
        // convertimos a eje arriba
        float yUp = -Yc;

        // deshacemos pitch rotando alrededor de X
        float p = DegToRad(pitchDownDeg);
        float cp = std::cos(p);
        float sp = std::sin(p);

        // rotacion en plano yUp z
        float yUpWorld = cp * yUp - sp * Zc;

        // altura sobre suelo
        return camHeightM + yUpWorld;
    }

    void VisionMath::HsvToRgb(float hDeg, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b)
    {
        hDeg = std::fmod(hDeg, 360.0f);
        if (hDeg < 0.0f) hDeg += 360.0f;

        float c = v * s;
        float x = c * (1.0f - std::fabs(std::fmod(hDeg / 60.0f, 2.0f) - 1.0f));
        float m = v - c;

        float rp = 0, gp = 0, bp = 0;

        if (hDeg < 60.0f) { rp = c; gp = x; bp = 0; }
        else if (hDeg < 120.0f) { rp = x; gp = c; bp = 0; }
        else if (hDeg < 180.0f) { rp = 0; gp = c; bp = x; }
        else if (hDeg < 240.0f) { rp = 0; gp = x; bp = c; }
        else if (hDeg < 300.0f) { rp = x; gp = 0; bp = c; }
        else { rp = c; gp = 0; bp = x; }

        int R = (int)std::lround((rp + m) * 255.0f);
        int G = (int)std::lround((gp + m) * 255.0f);
        int B = (int)std::lround((bp + m) * 255.0f);

        r = (uint8_t)std::clamp(R, 0, 255);
        g = (uint8_t)std::clamp(G, 0, 255);
        b = (uint8_t)std::clamp(B, 0, 255);
    }

    void VisionMath::DepthToHeatRGB(float z, float zMin, float zMax, uint8_t& r, uint8_t& g, uint8_t& b)
    {
        float denom = (zMax - zMin);
        if (denom < 1e-6f) denom = 1.0f;

        float t = (z - zMin) / denom;
        t = Clamp01(t);

        float hue = (1.0f - t) * 240.0f;
        HsvToRgb(hue, 1.0f, 1.0f, r, g, b);
    }

    float VisionMath::Percentile(std::vector<float>& v, float q)
    {
        if (v.empty()) return std::numeric_limits<float>::quiet_NaN();

        q = std::clamp(q, 0.0f, 1.0f);

        std::sort(v.begin(), v.end());
        float idx = q * (float)(v.size() - 1);

        size_t i0 = (size_t)idx;
        size_t i1 = (std::min)(i0 + 1, v.size() - 1);

        float t = idx - (float)i0;
        return v[i0] * (1.f - t) + v[i1] * t;
    }
}
