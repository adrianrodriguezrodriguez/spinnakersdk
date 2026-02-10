#pragma once

#include <string>

#include "Spinnaker.h"

namespace BBB
{
    class ImageIO
    {
    public:
        // guardamos PGM 8 bits
        static bool SavePGM8(const Spinnaker::ImagePtr& img, const std::string& filePath);

        // guardamos PGM 16 bits big endian
        static bool SavePGM16_BE(const Spinnaker::ImagePtr& img, const std::string& filePath);
    };
}
