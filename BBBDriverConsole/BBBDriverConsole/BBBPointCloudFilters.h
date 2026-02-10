#pragma once

#include <vector>
#include <cstdint>

namespace BBB
{
    // punto con color
    struct Pt
    {
        float x = 0, y = 0, z = 0;
        uint8_t r = 0, g = 0, b = 0;
    };

    // vector 3 para plano suelo
    struct V3
    {
        float x = 0, y = 0, z = 0;
    };

    // plano ax + by + cz + d = 0
    struct Plane
    {
        float a = 0, b = 0, c = 0, d = 0;
    };

    class CloudFilters
    {
    public:
        // voxel downsample promediando por celda
        static std::vector<Pt> VoxelDownsample(const std::vector<Pt>& in, float leaf);

        // quitamos puntos aislados por radio y vecinos
        static std::vector<Pt> RadiusOutlierRemoval(const std::vector<Pt>& in, float radius, int minNeighbors);

        // nos quedamos con el cluster mas grande en grid
        static std::vector<Pt> KeepLargestCluster(const std::vector<Pt>& in, float cellSize);

        // ransac de plano del suelo usando candidatos de la parte baja
        static bool FitGroundPlaneRANSAC(const std::vector<V3>& candidates, int iters, float thrM, float pitchDeg, Plane& bestPlane);

        // quitamos puntos que queden por debajo del plano del suelo
        static void RemoveGroundByPlane(std::vector<Pt>& pts, const Plane& ground, float marginM);
    };
}
