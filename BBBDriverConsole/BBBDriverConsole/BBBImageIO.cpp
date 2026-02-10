#include "BBBImageIO.h"

#include <fstream>
#include <cstdint>

namespace BBB
{
    bool ImageIO::SavePGM8(const Spinnaker::ImagePtr& img, const std::string& filePath)
    {
        const int w = (int)img->GetWidth();
        const int h = (int)img->GetHeight();
        const uint8_t* data = (const uint8_t*)img->GetData();
        if (!data) return false;

        const int stride = (int)img->GetStride();

        std::ofstream f(filePath, std::ios::binary);
        if (!f.is_open()) return false;

        f << "P5\n" << w << " " << h << "\n255\n";
        for (int y = 0; y < h; ++y)
            f.write((const char*)(data + y * stride), w);

        return true;
    }

    bool ImageIO::SavePGM16_BE(const Spinnaker::ImagePtr& img, const std::string& filePath)
    {
        const int w = (int)img->GetWidth();
        const int h = (int)img->GetHeight();
        const uint16_t* data = (const uint16_t*)img->GetData();
        if (!data) return false;

        const int strideU16 = (int)(img->GetStride() / sizeof(uint16_t));

        std::ofstream f(filePath, std::ios::binary);
        if (!f.is_open()) return false;

        f << "P5\n" << w << " " << h << "\n65535\n";

        for (int y = 0; y < h; ++y)
        {
            const uint16_t* row = data + y * strideU16;
            for (int x = 0; x < w; ++x)
            {
                uint16_t v = row[x];
                unsigned char be[2] = {
                    (unsigned char)(v >> 8),
                    (unsigned char)(v & 0xFF)
                };
                f.write((char*)be, 2);
            }
        }

        return true;
    }
}
