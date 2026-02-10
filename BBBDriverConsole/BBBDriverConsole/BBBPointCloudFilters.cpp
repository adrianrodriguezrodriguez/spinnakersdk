#include "BBBPointCloudFilters.h"

#include <unordered_map>
#include <queue>
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace BBB
{
    // clave 3D para voxel y grids
    struct Key3
    {
        int x = 0, y = 0, z = 0;

        bool operator==(const Key3& o) const
        {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    // hash de Key3
    struct Key3Hash
    {
        size_t operator()(const Key3& k) const
        {
            size_t h1 = std::hash<int>()(k.x);
            size_t h2 = std::hash<int>()(k.y);
            size_t h3 = std::hash<int>()(k.z);
            return h1 ^ (h2 * 0x9e3779b1u) ^ (h3 * 0x85ebca6bu);
        }
    };

    // calculamos celda para un punto
    static Key3 CellKey(float x, float y, float z, float cell)
    {
        return Key3{
            (int)std::floor(x / cell),
            (int)std::floor(y / cell),
            (int)std::floor(z / cell)
        };
    }

    std::vector<Pt> CloudFilters::VoxelDownsample(const std::vector<Pt>& in, float leaf)
    {
        if (leaf <= 1e-6f) return in;

        struct Acc
        {
            double sx = 0, sy = 0, sz = 0;
            double sr = 0, sg = 0, sb = 0;
            int n = 0;
        };

        std::unordered_map<Key3, Acc, Key3Hash> m;
        m.reserve(in.size());

        for (const auto& p : in)
        {
            Key3 k = CellKey(p.x, p.y, p.z, leaf);
            auto& a = m[k];

            a.sx += p.x;
            a.sy += p.y;
            a.sz += p.z;
            a.sr += p.r;
            a.sg += p.g;
            a.sb += p.b;
            a.n += 1;
        }

        std::vector<Pt> out;
        out.reserve(m.size());

        for (auto& it : m)
        {
            const Acc& a = it.second;
            if (a.n <= 0) continue;

            Pt p;
            p.x = (float)(a.sx / a.n);
            p.y = (float)(a.sy / a.n);
            p.z = (float)(a.sz / a.n);
            p.r = (uint8_t)std::clamp((int)std::lround(a.sr / a.n), 0, 255);
            p.g = (uint8_t)std::clamp((int)std::lround(a.sg / a.n), 0, 255);
            p.b = (uint8_t)std::clamp((int)std::lround(a.sb / a.n), 0, 255);

            out.push_back(p);
        }

        return out;
    }

    std::vector<Pt> CloudFilters::RadiusOutlierRemoval(const std::vector<Pt>& in, float radius, int minNeighbors)
    {
        if (in.empty()) return in;
        if (radius <= 1e-6f) return in;
        if (minNeighbors <= 1) return in;

        const float cell = radius;
        const float r2 = radius * radius;

        std::unordered_map<Key3, std::vector<int>, Key3Hash> grid;
        grid.reserve(in.size());

        for (int i = 0; i < (int)in.size(); ++i)
        {
            Key3 k = CellKey(in[i].x, in[i].y, in[i].z, cell);
            grid[k].push_back(i);
        }

        std::vector<Pt> out;
        out.reserve(in.size());

        for (int i = 0; i < (int)in.size(); ++i)
        {
            const Pt& p = in[i];
            Key3 ck = CellKey(p.x, p.y, p.z, cell);

            int neighbors = 0;

            for (int dz = -1; dz <= 1; ++dz)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        Key3 nk{ ck.x + dx, ck.y + dy, ck.z + dz };
                        auto it = grid.find(nk);
                        if (it == grid.end()) continue;

                        const auto& lst = it->second;
                        for (int j : lst)
                        {
                            if (j == i) continue;
                            const Pt& q = in[j];

                            float dx2 = p.x - q.x;
                            float dy2 = p.y - q.y;
                            float dz2 = p.z - q.z;
                            float d2 = dx2 * dx2 + dy2 * dy2 + dz2 * dz2;

                            if (d2 <= r2)
                            {
                                neighbors++;
                                if (neighbors >= minNeighbors) break;
                            }
                        }

                        if (neighbors >= minNeighbors) break;
                    }

            if (neighbors >= minNeighbors)
                out.push_back(p);
        }

        return out;
    }

    std::vector<Pt> CloudFilters::KeepLargestCluster(const std::vector<Pt>& in, float cellSize)
    {
        if (in.empty()) return in;
        if (cellSize <= 1e-6f) return in;

        std::unordered_map<Key3, std::vector<int>, Key3Hash> cells;
        cells.reserve(in.size());

        for (int i = 0; i < (int)in.size(); ++i)
        {
            Key3 k = CellKey(in[i].x, in[i].y, in[i].z, cellSize);
            cells[k].push_back(i);
        }

        std::unordered_map<Key3, int, Key3Hash> visited;
        visited.reserve(cells.size());

        int bestCount = -1;
        std::vector<Key3> bestKeys;

        std::queue<Key3> q;

        for (auto& it : cells)
        {
            const Key3 start = it.first;
            if (visited.find(start) != visited.end()) continue;

            std::vector<Key3> compKeys;
            int compCount = 0;

            visited[start] = 1;
            q.push(start);

            while (!q.empty())
            {
                Key3 cur = q.front();
                q.pop();

                auto itc = cells.find(cur);
                if (itc == cells.end()) continue;

                compKeys.push_back(cur);
                compCount += (int)itc->second.size();

                for (int dz = -1; dz <= 1; ++dz)
                    for (int dy = -1; dy <= 1; ++dy)
                        for (int dx = -1; dx <= 1; ++dx)
                        {
                            if (dx == 0 && dy == 0 && dz == 0) continue;

                            Key3 nb{ cur.x + dx, cur.y + dy, cur.z + dz };
                            if (cells.find(nb) == cells.end()) continue;
                            if (visited.find(nb) != visited.end()) continue;

                            visited[nb] = 1;
                            q.push(nb);
                        }
            }

            if (compCount > bestCount)
            {
                bestCount = compCount;
                bestKeys.swap(compKeys);
            }
        }

        if (bestCount <= 0) return in;

        std::unordered_map<Key3, int, Key3Hash> keep;
        keep.reserve(bestKeys.size());
        for (const auto& k : bestKeys) keep[k] = 1;

        std::vector<Pt> out;
        out.reserve(bestCount);

        for (auto& it : cells)
        {
            if (keep.find(it.first) == keep.end()) continue;
            for (int idx : it.second)
                out.push_back(in[idx]);
        }

        return out;
    }

    // restamos vectores
    static V3 Sub(const V3& a, const V3& b)
    {
        return V3{ a.x - b.x, a.y - b.y, a.z - b.z };
    }

    // producto cruz
    static V3 Cross(const V3& a, const V3& b)
    {
        return V3{
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    // producto punto
    static float Dot(const V3& a, const V3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    // norma
    static float Norm(const V3& a)
    {
        return std::sqrt(Dot(a, a));
    }

    // normalizamos
    static V3 Normalize(const V3& a)
    {
        float n = Norm(a);
        if (n < 1e-9f) return V3{ 0, 0, 0 };
        return V3{ a.x / n, a.y / n, a.z / n };
    }

    // creamos plano desde 3 puntos
    static bool PlaneFrom3Pts(const V3& p0, const V3& p1, const V3& p2, Plane& out)
    {
        V3 u = Sub(p1, p0);
        V3 v = Sub(p2, p0);
        V3 n = Cross(u, v);
        float nn = Norm(n);
        if (nn < 1e-8f) return false;

        n = Normalize(n);

        out.a = n.x;
        out.b = n.y;
        out.c = n.z;
        out.d = -(out.a * p0.x + out.b * p0.y + out.c * p0.z);

        return true;
    }

    // distancia firmada al plano
    static float SignedDist(const Plane& pl, const V3& p)
    {
        return pl.a * p.x + pl.b * p.y + pl.c * p.z + pl.d;
    }

    bool CloudFilters::FitGroundPlaneRANSAC(const std::vector<V3>& candidates, int iters, float thrM, float /*pitchDeg*/, Plane& bestPlane)
    {
        if ((int)candidates.size() < 80) return false;

        float bestScore = -1.0f;
        Plane best{};

        static bool seeded = false;
        if (!seeded) { seeded = true; std::srand(12345); }

        for (int k = 0; k < iters; ++k)
        {
            int i0 = std::rand() % (int)candidates.size();
            int i1 = std::rand() % (int)candidates.size();
            int i2 = std::rand() % (int)candidates.size();

            if (i0 == i1 || i0 == i2 || i1 == i2) continue;

            Plane pl;
            if (!PlaneFrom3Pts(candidates[i0], candidates[i1], candidates[i2], pl)) continue;

            float nlen = std::sqrt(pl.a * pl.a + pl.b * pl.b + pl.c * pl.c);
            if (nlen < 1e-9f) continue;

            pl.a /= nlen; pl.b /= nlen; pl.c /= nlen; pl.d /= nlen;

            int inliers = 0;
            for (const auto& p : candidates)
            {
                float dist = std::fabs(SignedDist(pl, p));
                if (dist <= thrM) inliers++;
            }

            float prior = 0.5f + std::fabs(pl.b);
            float score = (float)inliers * prior;

            if (score > bestScore)
            {
                bestScore = score;
                best = pl;
            }
        }

        if (bestScore <= 0.0f) return false;

        bestPlane = best;
        return true;
    }

    void CloudFilters::RemoveGroundByPlane(std::vector<Pt>& pts, const Plane& ground, float marginM)
    {
        int pos = 0;
        int neg = 0;

        for (int i = 0; i < (int)pts.size(); ++i)
        {
            V3 p{ pts[i].x, pts[i].y, pts[i].z };
            float sd = SignedDist(ground, p);
            if (sd >= 0.0f) pos++;
            else neg++;
        }

        bool keepPositive = (pos >= neg);

        std::vector<Pt> out;
        out.reserve(pts.size());

        for (const auto& q : pts)
        {
            V3 p{ q.x, q.y, q.z };
            float sd = SignedDist(ground, p);

            if (keepPositive)
            {
                if (sd > marginM) out.push_back(q);
            }
            else
            {
                if (sd < -marginM) out.push_back(q);
            }
        }

        pts.swap(out);
    }
}
