#include "BBBDriver.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>
#include <unordered_map>
#include <queue>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;

// ARR helpers matematicos y color
static float Clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}
static float DegToRad(float deg)
{
    return deg * 3.14159265358979323846f / 180.0f;
}

// ARR sistema camara X derecha Y abajo Z delante
static float HeightAboveGroundM(float Xc, float Yc, float Zc, float camHeightM, float pitchDownDeg)
{
    float yUp = -Yc;

    float p = DegToRad(pitchDownDeg);
    float cp = std::cos(p);
    float sp = std::sin(p);

    float yUpWorld = cp * yUp - sp * Zc;
    return camHeightM + yUpWorld;
}

static void HsvToRgb(float hDeg, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b)
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

static void DepthToHeatRGB(float z, float zMin, float zMax, uint8_t& r, uint8_t& g, uint8_t& b)
{
    float denom = (zMax - zMin);
    if (denom < 1e-6f) denom = 1.0f;

    float t = (z - zMin) / denom;
    t = Clamp01(t);

    float hue = (1.0f - t) * 240.0f;
    HsvToRgb(hue, 1.0f, 1.0f, r, g, b);
}

// ARR percentil robusto
static float Percentile(std::vector<float>& v, float q)
{
    if (v.empty()) return std::numeric_limits<float>::quiet_NaN();
    std::sort(v.begin(), v.end());
    float idx = q * (v.size() - 1);
    size_t i0 = (size_t)idx;
    size_t i1 = (std::min)(i0 + 1, v.size() - 1);
    float t = idx - (float)i0;
    return v[i0] * (1.f - t) + v[i1] * t;
}

struct Pt
{
    float x, y, z;
    uint8_t r, g, b;
};

struct Key3
{
    int x, y, z;
    bool operator==(const Key3& o) const { return x == o.x && y == o.y && z == o.z; }
};

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

static Key3 CellKey(float x, float y, float z, float cell)
{
    return Key3{ (int)std::floor(x / cell), (int)std::floor(y / cell), (int)std::floor(z / cell) };
}

// ARR voxel sencillo
static std::vector<Pt> VoxelDownsample(const std::vector<Pt>& in, float leaf)
{
    if (leaf <= 1e-6f) return in;

    struct Acc { double sx = 0, sy = 0, sz = 0, sr = 0, sg = 0, sb = 0; int n = 0; };
    std::unordered_map<Key3, Acc, Key3Hash> m;
    m.reserve(in.size());

    for (const auto& p : in)
    {
        Key3 k = CellKey(p.x, p.y, p.z, leaf);
        auto& a = m[k];
        a.sx += p.x; a.sy += p.y; a.sz += p.z;
        a.sr += p.r; a.sg += p.g; a.sb += p.b;
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

// ARR outlier por radio con grid
static std::vector<Pt> RadiusOutlierRemoval(const std::vector<Pt>& in, float radius, int minNeighbors)
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

                    for (int j : it->second)
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

        if (neighbors >= minNeighbors) out.push_back(p);
    }

    return out;
}

// ARR cluster mayor con grid BFS
static std::vector<Pt> KeepLargestCluster(const std::vector<Pt>& in, float cellSize)
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
        for (int idx : it.second) out.push_back(in[idx]);
    }

    return out;
}

BBBDriver::~BBBDriver()
{
    Close();
}

// TELEDYNE usamos nodos GenICam para setear enumeraciones
bool BBBDriver::SetEnumAsString(INodeMap& nodeMap, const char* name, const char* value)
{
    CEnumerationPtr node = nodeMap.GetNode(name);
    if (!IsReadable(node) || !IsWritable(node)) return false;

    CEnumEntryPtr entry = node->GetEntryByName(value);
    if (!IsReadable(entry)) return false;

    node->SetIntValue(entry->GetValue());
    return true;
}

bool BBBDriver::TrySetEnumAny(INodeMap& nodeMap, const char* name, const char* const* values, int nValues)
{
    for (int i = 0; i < nValues; ++i)
        if (SetEnumAsString(nodeMap, name, values[i]))
            return true;
    return false;
}

// TELEDYNE leemos nodos float y bool oficiales
bool BBBDriver::GetFloatNode(INodeMap& nodeMap, const char* name, float& out)
{
    CFloatPtr node = nodeMap.GetNode(name);
    if (!IsReadable(node)) return false;
    out = (float)node->GetValue();
    return true;
}

bool BBBDriver::GetBoolNode(INodeMap& nodeMap, const char* name, bool& out)
{
    CBooleanPtr node = nodeMap.GetNode(name);
    if (!IsReadable(node)) return false;
    out = node->GetValue();
    return true;
}

// ARR baseline mm o m a metros
float BBBDriver::BaselineToMeters(float baselineMaybeMm)
{
    float b = baselineMaybeMm;
    if (b > 1.0f) b *= 0.001f;
    return b;
}

// TELEDYNE validamos payloads oficiales de rectified y disparity
bool BBBDriver::ValidateSetHasRectDisp(const ImageList& set)
{
    ImagePtr disp = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
    ImagePtr rect = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1);

    if (!disp || !rect) return false;
    if (disp->IsIncomplete() || rect->IsIncomplete()) return false;
    if (!disp->GetData() || !rect->GetData()) return false;
    return true;
}

// ARR clamp roi en porcentajes
void BBBDriver::ClampRoiXY(const BBBParams& p, int w, int h, int& x0, int& x1, int& y0, int& y1)
{
    int ax = (std::max)(0, (std::min)(100, p.roiMinXPct));
    int bx = (std::max)(0, (std::min)(100, p.roiMaxXPct));
    if (ax > bx) std::swap(ax, bx);
    if (bx - ax < 5) bx = (std::min)(100, ax + 5);

    int ay = (std::max)(0, (std::min)(100, p.roiMinYPct));
    int by = (std::max)(0, (std::min)(100, p.roiMaxYPct));
    if (ay > by) std::swap(ay, by);
    if (by - ay < 5) by = (std::min)(100, ay + 5);

    x0 = w * ax / 100;
    x1 = w * bx / 100;
    y0 = h * ay / 100;
    y1 = h * by / 100;

    x0 = (std::max)(0, (std::min)(w - 1, x0));
    x1 = (std::max)(1, (std::min)(w, x1));
    y0 = (std::max)(0, (std::min)(h - 1, y0));
    y1 = (std::max)(1, (std::min)(h, y1));
}

// TELEDYNE abrimos camara por serial usando CameraList
bool BBBDriver::OpenBySerial(CameraList& cams, const std::string& serial)
{
    Close();
    if (serial.empty()) return false;
    if (cams.GetSize() == 0) return false;

    // IMPORTANTE
    // Antes teniamos un bug gordo
    // Cada vez que buscabamos una camara por serial inicializabamos TODAS y luego haciamos DeInit() a las que no eran
    // En multi-camara eso desinicializa la camara que ya habiamos abierto en otro BBBDriver -> BeginAcquisition [-1002]
    // Solucion
    // Leemos el serial por TLDevice sin tocar Init/DeInit, y solo inicializamos la camara objetivo

    for (unsigned int i = 0; i < cams.GetSize(); ++i)
    {
        CameraPtr c = cams.GetByIndex(i);
        std::string s;

        try
        {
            s = c->TLDevice.DeviceSerialNumber.ToString().c_str();
        }
        catch (...) { continue; }

        if (s != serial) continue;

        try
        {
            c->Init();
        }
        catch (Spinnaker::Exception& e)
        {
            std::cout << "Init fallo " << e.what() << "\n";
            return false;
        }

        bool isStereo = false;
        try
        {
            isStereo = ImageUtilityStereo::IsStereoCamera(c);
        }
        catch (...) { isStereo = false; }

        if (!isStereo)
        {
            try { c->DeInit(); }
            catch (...) {}
            return false;
        }

        cam = c;
        return true;
    }

    return false;
}

// TELEDYNE abrimos primera estereo saltando serial
bool BBBDriver::OpenFirstStereoSkip(CameraList& cams, const std::string& serialToSkip)
{
    Close();
    if (cams.GetSize() == 0) return false;

    for (unsigned int i = 0; i < cams.GetSize(); ++i)
    {
        CameraPtr c = cams.GetByIndex(i);

        std::string s;
        try { s = c->TLDevice.DeviceSerialNumber.ToString().c_str(); }
        catch (...) { s.clear(); }

        if (!serialToSkip.empty() && s == serialToSkip)
            continue;

        try
        {
            c->Init();
        }
        catch (...) { continue; }

        bool okStereo = false;
        try { okStereo = ImageUtilityStereo::IsStereoCamera(c); }
        catch (...) { okStereo = false; }

        if (okStereo)
        {
            cam = c;
            return true;
        }

        try { c->DeInit(); }
        catch (...) {}
    }

    return false;
}

void BBBDriver::Close()
{
    try
    {
        if (cam)
        {
            StopAcquisition();
            // TELEDYNE DeInit oficial
            cam->DeInit();
            cam = nullptr;
        }
    }
    catch (...) {}
}

// TELEDYNE control GVCP heartbeat en nodos GenICam
bool BBBDriver::DisableGVCPHeartbeat(bool disable)
{
    if (!cam) return false;

    try
    {
        INodeMap& nodeMap = cam->GetNodeMap();
        CBooleanPtr hb = nodeMap.GetNode("GevGVCPHeartbeatDisable");
        if (IsWritable(hb))
        {
            hb->SetValue(disable);
            return true;
        }
    }
    catch (...) {}
    return false;
}

// TELEDYNE configuramos componentes oficiales Rectified y Disparity
bool BBBDriver::ConfigureStreams_Rectified1_Disparity()
{
    if (!cam) return false;

    try
    {
        INodeMap& nodeMap = cam->GetNodeMap();

        CEnumerationPtr sourceSel = nodeMap.GetNode("SourceSelector");
        CEnumerationPtr compSel = nodeMap.GetNode("ComponentSelector");
        CBooleanPtr compEnable = nodeMap.GetNode("ComponentEnable");

        if (!IsReadable(sourceSel) || !IsWritable(sourceSel))
        {
            std::cout << "SourceSelector no accesible\n";
            return false;
        }
        if (!IsReadable(compSel) || !IsWritable(compSel))
        {
            std::cout << "ComponentSelector no accesible\n";
            return false;
        }
        if (!IsReadable(compEnable) || !IsWritable(compEnable))
        {
            std::cout << "ComponentEnable no accesible\n";
            return false;
        }

        const char* sensors[] = { "Sensor1", "Sensor0" };
        if (!TrySetEnumAny(nodeMap, "SourceSelector", sensors, 2))
        {
            std::cout << "No pude setear SourceSelector Sensor1/Sensor0\n";
            return false;
        }

        NodeList_t entries;
        compSel->GetEntries(entries);
        for (auto& n : entries)
        {
            CEnumEntryPtr e = (CEnumEntryPtr)n;
            if (!IsReadable(e)) continue;
            compSel->SetIntValue(e->GetValue());
            compEnable->SetValue(false);
        }

        const char* rectNames[] = { "Rectified" };
        if (!TrySetEnumAny(nodeMap, "ComponentSelector", rectNames, 1))
        {
            std::cout << "No existe ComponentSelector=Rectified\n";
            return false;
        }
        compEnable->SetValue(true);

        const char* dispNames[] = { "Disparity" };
        if (!TrySetEnumAny(nodeMap, "ComponentSelector", dispNames, 1))
        {
            std::cout << "No existe ComponentSelector=Disparity\n";
            return false;
        }
        compEnable->SetValue(true);

        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "ConfigureStreams fallo " << e.what() << "\n";
        return false;
    }
}

// TELEDYNE trigger software con nodos oficiales
bool BBBDriver::ConfigureSoftwareTrigger()
{
    if (!cam) return false;

    try
    {
        INodeMap& nodeMap = cam->GetNodeMap();

        SetEnumAsString(nodeMap, "AcquisitionMode", "Continuous");

        if (!SetEnumAsString(nodeMap, "TriggerMode", "Off")) { std::cout << "TriggerMode Off FAIL\n"; return false; }
        if (!SetEnumAsString(nodeMap, "TriggerSelector", "FrameStart")) { std::cout << "TriggerSelector FrameStart FAIL\n"; return false; }
        if (!SetEnumAsString(nodeMap, "TriggerSource", "Software")) { std::cout << "TriggerSource Software FAIL\n"; return false; }
        if (!SetEnumAsString(nodeMap, "TriggerMode", "On")) { std::cout << "TriggerMode On FAIL\n"; return false; }

        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "ConfigureSoftwareTrigger fallo " << e.what() << "\n";
        return false;
    }
}

// TELEDYNE stream buffer mode oficial
bool BBBDriver::ConfigureStreamBuffersNewestOnly()
{
    if (!cam) return false;

    try
    {
        INodeMap& tl = cam->GetTLStreamNodeMap();
        const char* modes[] = { "NewestOnly", "OldestFirst" };
        return TrySetEnumAny(tl, "StreamBufferHandlingMode", modes, 2);
    }
    catch (...) {}
    return false;
}

// TELEDYNE Scan3D params con nodos oficiales
bool BBBDriver::ReadScan3DParams(Scan3DParams& out)
{
    if (!cam) return false;

    try
    {
        INodeMap& nodeMap = cam->GetNodeMap();

        if (!GetFloatNode(nodeMap, "Scan3dCoordinateScale", out.scale)) return false;
        if (!GetFloatNode(nodeMap, "Scan3dCoordinateOffset", out.offset)) return false;
        if (!GetFloatNode(nodeMap, "Scan3dFocalLength", out.focal)) return false;
        if (!GetFloatNode(nodeMap, "Scan3dBaseline", out.baseline)) return false;
        if (!GetFloatNode(nodeMap, "Scan3dPrincipalPointU", out.principalU)) return false;
        if (!GetFloatNode(nodeMap, "Scan3dPrincipalPointV", out.principalV)) return false;
        if (!GetBoolNode(nodeMap, "Scan3dInvalidDataFlag", out.invalidFlag)) return false;
        if (!GetFloatNode(nodeMap, "Scan3dInvalidDataValue", out.invalidValue)) return false;

        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "ReadScan3DParams fallo " << e.what() << "\n";
        return false;
    }
}

// TELEDYNE BeginAcquisition y EndAcquisition oficiales
bool BBBDriver::StartAcquisition()
{
    if (!cam) return false;
    if (acquiring) return true;

    try
    {
        ConfigureStreamBuffersNewestOnly();
        cam->BeginAcquisition();
        acquiring = true;
        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "BeginAcquisition fallo " << e.what() << "\n";
        acquiring = false;
        return false;
    }
}

void BBBDriver::StopAcquisition()
{
    if (!cam || !acquiring) return;
    try { cam->EndAcquisition(); }
    catch (...) {}
    acquiring = false;
}

// TELEDYNE TriggerSoftware y GetNextImageSync oficiales
bool BBBDriver::CaptureOnceSync(ImageList& outSet, uint64_t timeoutMs)
{
    if (!cam) return false;
    if (!StartAcquisition()) return false;

    try
    {
        INodeMap& nodeMap = cam->GetNodeMap();

        CCommandPtr sw = nodeMap.GetNode("TriggerSoftware");
        if (IsWritable(sw)) sw->Execute();

        outSet = cam->GetNextImageSync(timeoutMs);

        if (!ValidateSetHasRectDisp(outSet)) return false;
        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "GetNextImageSync fallo " << e.what() << "\n";
        return false;
    }
}

// ARR guardado PGM simple
static bool SavePGM8(const ImagePtr& img, const std::string& filePath)
{
    const int w = (int)img->GetWidth();
    const int h = (int)img->GetHeight();
    const uint8_t* data = (const uint8_t*)img->GetData();
    if (!data) return false;

    const int stride = (int)img->GetStride();

    std::ofstream f(filePath, std::ios::binary);
    if (!f.is_open()) return false;

    f << "P5\n" << w << " " << h << "\n255\n";
    for (int y = 0; y < h; ++y) f.write((const char*)(data + y * stride), w);

    return true;
}

static bool SavePGM16_BE(const ImagePtr& img, const std::string& filePath)
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
            unsigned char be[2] = { (unsigned char)(v >> 8), (unsigned char)(v & 0xFF) };
            f.write((char*)be, 2);
        }
    }

    return true;
}

bool BBBDriver::SaveDisparityPGM(const ImageList& set, const std::string& filePath)
{
    ImagePtr disp = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
    if (!disp) return false;
    if (disp->IsIncomplete()) return false;
    if (!disp->GetData()) return false;

    try
    {
        const unsigned int bpp = disp->GetBitsPerPixel();
        if (bpp <= 8) return SavePGM8(disp, filePath);
        return SavePGM16_BE(disp, filePath);
    }
    catch (...) { return false; }
}

// TELEDYNE ImagePtr Save es oficial
bool BBBDriver::SaveRectifiedPNG(const ImageList& set, const std::string& filePath)
{
    ImagePtr rect = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1);
    if (!rect || rect->IsIncomplete()) return false;

    try
    {
        rect->Save(filePath.c_str());
        return true;
    }
    catch (...) { return false; }
}

// ARR el resto de funciones queda igual que tu version anterior
// ARR aqui irian SavePointCloudPLY_Filtered GetDistanceCentralPointM GetDistanceToBultoM_Debug SetExposureUs SetGainDb


bool BBBDriver::SavePointCloudPLY_Filtered(
    const ImageList& set,
    const Scan3DParams& s3d,
    const BBBParams& p,
    const BBBCameraMount& mount,
    const std::string& filePath)
{
    ImagePtr disp = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
    ImagePtr rect = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1);

    if (!disp || disp->IsIncomplete() || !disp->GetData()) return false;

    const int w = (int)disp->GetWidth();
    const int h = (int)disp->GetHeight();

    float baselineM = BaselineToMeters(s3d.baseline);
    const float focal = s3d.focal;
    if (focal <= 1e-6f || baselineM <= 1e-9f) return false;

    // Aplicamos speckle del SDK sobre disparity
    if (p.applySpeckleFilter)
    {
        try
        {
            ImageUtilityStereo::FilterSpecklesFromImage(
                disp,
                p.maxSpeckleSize,
                p.speckleThreshold,
                s3d.scale,
                s3d.invalidValue
            );
        }
        catch (...) {}
    }

    const uint8_t* rectData = nullptr;
    int rectStride = 0;
    if (rect && !rect->IsIncomplete() && rect->GetData() && rect->GetBitsPerPixel() == 8)
    {
        rectData = (const uint8_t*)rect->GetData();
        rectStride = (int)rect->GetStride();
    }

    const unsigned int bpp = disp->GetBitsPerPixel();
    const int step = (std::max)(1, p.decimationFactor);

    const uint8_t* d8 = (const uint8_t*)disp->GetData();
    const uint16_t* d16 = (const uint16_t*)disp->GetData();
    const int strideBytes = (int)disp->GetStride();
    const int strideU16 = strideBytes / (int)sizeof(uint16_t);

    int x0, x1, y0, y1;
    ClampRoiXY(p, w, h, x0, x1, y0, y1);

    auto IsInvalidRaw = [&](uint16_t raw) -> bool
        {
            if (raw == 0) return true;
            if (s3d.invalidFlag)
            {
                uint16_t inv = (uint16_t)(s3d.invalidValue);
                if (raw == inv) return true;
            }
            return false;
        };

    auto ReadRawAt = [&](int x, int y) -> uint16_t
        {
            if (bpp <= 8) return (uint16_t)d8[y * strideBytes + x];
            return d16[y * strideU16 + x];
        };

    auto MedianRaw3x3 = [&](int x, int y) -> uint16_t
        {
            if (!p.applyMedian3x3) return ReadRawAt(x, y);

            uint16_t vals[9];
            int n = 0;

            for (int dy = -1; dy <= 1; ++dy)
            {
                int yy = y + dy;
                if (yy < 0 || yy >= h) continue;

                for (int dx = -1; dx <= 1; ++dx)
                {
                    int xx = x + dx;
                    if (xx < 0 || xx >= w) continue;

                    uint16_t r = ReadRawAt(xx, yy);
                    if (IsInvalidRaw(r)) continue;
                    vals[n++] = r;
                }
            }

            if (n == 0) return 0;
            std::sort(vals, vals + n);
            return vals[n / 2];
        };

    auto RawToDisparity = [&](uint16_t raw) -> float
        {
            return (float)raw * s3d.scale + s3d.offset;
        };

    auto DisparityToZ = [&](float dispVal) -> float
        {
            return (focal * baselineM) / dispVal;
        };

    std::vector<Pt> pts;
    pts.reserve(((x1 - x0) / step) * ((y1 - y0) / step));

    float zHardMax = p.hardMaxZM;
    float zMaxUse = std::min(p.maxRangeM, zHardMax);

    for (int y = y0; y < y1; y += step)
    {
        for (int x = x0; x < x1; x += step)
        {
            uint16_t raw = MedianRaw3x3(x, y);
            if (IsInvalidRaw(raw)) continue;

            float dispVal = RawToDisparity(raw);
            if (dispVal <= 1e-6f) continue;

            float z = DisparityToZ(dispVal);
            if (!std::isfinite(z)) continue;

            if (z > zHardMax) continue;
            if (z < p.minRangeM || z > zMaxUse) continue;

            float X = ((float)x - s3d.principalU) * z / focal;
            float Y = ((float)y - s3d.principalV) * z / focal;

            // filtro geometrico suelo (si está activo)
            if (p.enableGroundPlaneFilter)
            {
                float hAG = HeightAboveGroundM(X, Y, z, mount.alturaCamaraM, mount.pitchDeg);
                if (!std::isfinite(hAG)) continue;
                if (hAG < p.groundMinHeightM) continue;
            }

            uint8_t R = 180, G = 180, B = 180;

            if (p.colorMode == 2)
            {
                DepthToHeatRGB(z, p.minRangeM, zMaxUse, R, G, B);
            }
            else if (p.colorMode == 1 && rectData && rectStride > 0)
            {
                uint8_t g = rectData[y * rectStride + x];
                R = g; G = g; B = g;
            }

            Pt q;
            q.x = X; q.y = Y; q.z = z;
            q.r = R; q.g = G; q.b = B;
            pts.push_back(q);
        }
    }

    if (pts.size() < 500)
    {
        std::cout << "Pocos puntos antes de limpiar " << pts.size() << "\n";
        return false;
    }

    std::cout << "Puntos RAW (sin filtrar) " << pts.size() << "\n";

    float zFront = std::numeric_limits<float>::quiet_NaN();
    if (p.enableFrontDepthClamp)
    {
        std::vector<float> zvals;
        zvals.reserve(pts.size());
        for (const auto& q : pts) zvals.push_back(q.z);

        zFront = Percentile(zvals, p.frontFacePercentile);
        if (std::isfinite(zFront))
        {
            float zCut = zFront + p.frontDepthBandM;

            std::vector<Pt> tmp;
            tmp.reserve(pts.size());

            for (const auto& q : pts)
                if (q.z <= zCut) tmp.push_back(q);

            std::cout << "Corte de fondo (profundidad) zFront (frente) " << zFront
                << " m banda " << p.frontDepthBandM
                << " puntos " << pts.size() << " -> " << tmp.size() << "\n";

            pts.swap(tmp);
        }
    }

    if (pts.size() < 400)
    {
        std::cout << "Pocos puntos tras corte fondo " << pts.size() << "\n";
        return false;
    }

    {
        auto tmp = VoxelDownsample(pts, p.voxelLeafM);
        std::cout << "Puntos voxel " << pts.size() << " -> " << tmp.size() << "\n";
        pts.swap(tmp);
    }

    {
        auto tmp = RadiusOutlierRemoval(pts, p.outlierRadiusM, p.outlierMinNeighbors);
        std::cout << "Puntos outlier " << pts.size() << " -> " << tmp.size() << "\n";
        pts.swap(tmp);
    }

    if (p.keepLargestCluster)
    {
        auto tmp = KeepLargestCluster(pts, p.outlierRadiusM);
        std::cout << "Puntos cluster " << pts.size() << " -> " << tmp.size() << "\n";
        pts.swap(tmp);
    }

    if (pts.size() < 300)
    {
        std::cout << "Pocos puntos despues de limpiar " << pts.size() << "\n";
        return false;
    }

    // Medidas en consola
    {
        std::vector<float> xs, zs, hs;
        xs.reserve(pts.size());
        zs.reserve(pts.size());
        hs.reserve(pts.size());

        for (const auto& q : pts)
        {
            xs.push_back(q.x);
            zs.push_back(q.z);

            float hAG = HeightAboveGroundM(q.x, q.y, q.z, mount.alturaCamaraM, mount.pitchDeg);
            if (std::isfinite(hAG)) hs.push_back(hAG);
        }

        float xMin = +1e9f, xMax = -1e9f;
        float hMin = +1e9f, hMax = -1e9f;
        float zMin = +1e9f, zMax = -1e9f;

        for (const auto& q : pts)
        {
            xMin = std::min(xMin, q.x);
            xMax = std::max(xMax, q.x);
            zMin = std::min(zMin, q.z);
            zMax = std::max(zMax, q.z);
        }

        for (float hv : hs)
        {
            hMin = std::min(hMin, hv);
            hMax = std::max(hMax, hv);
        }

        float qLo = std::clamp(p.dimPercentileLow, 0.0f, 0.49f);
        float qHi = std::clamp(p.dimPercentileHigh, 0.51f, 1.0f);

        float xLo = Percentile(xs, qLo);
        float xHi = Percentile(xs, qHi);

        float hLo = Percentile(hs, qLo);
        float hHi = Percentile(hs, qHi);

        float zLo = Percentile(zs, 0.05f);
        float zHi = Percentile(zs, 0.95f);

        float anchoM = xHi - xLo;
        float altoM = hHi - hLo;

        float zFace = std::isfinite(zFront) ? zFront : Percentile(zs, p.frontFacePercentile);
        float faceAnchoM = std::numeric_limits<float>::quiet_NaN();
        float faceAltoM = std::numeric_limits<float>::quiet_NaN();

        if (std::isfinite(zFace))
        {
            std::vector<float> fxs, fhs;
            fxs.reserve(pts.size() / 3);
            fhs.reserve(pts.size() / 3);

            float zLim = zFace + p.faceSlabM;
            for (const auto& q : pts)
            {
                if (q.z > zLim) continue;
                fxs.push_back(q.x);

                float hAG = HeightAboveGroundM(q.x, q.y, q.z, mount.alturaCamaraM, mount.pitchDeg);
                if (std::isfinite(hAG)) fhs.push_back(hAG);
            }

            if (fxs.size() >= 200 && fhs.size() >= 200)
            {
                float fxLo = Percentile(fxs, qLo);
                float fxHi = Percentile(fxs, qHi);
                float fhLo = Percentile(fhs, qLo);
                float fhHi = Percentile(fhs, qHi);

                faceAnchoM = fxHi - fxLo;
                faceAltoM = fhHi - fhLo;
            }
        }

        std::cout << "BULTO dims "
            << "alto p" << (int)std::lround(qLo * 100) << "-" << (int)std::lround(qHi * 100) << " "
            << altoM << " m " << (int)std::lround(altoM * 1000.0f) << " mm "
            << "ancho p" << (int)std::lround(qLo * 100) << "-" << (int)std::lround(qHi * 100) << " "
            << anchoM << " m " << (int)std::lround(anchoM * 1000.0f) << " mm "
            << "z p5-95 " << zLo << " a " << zHi
            << "\n";

        std::cout << "BULTO debug "
            << "alto min-max " << (hMax - hMin) << " m "
            << "ancho min-max " << (xMax - xMin) << " m "
            << "z min-max " << zMin << " a " << zMax
            << "\n";

        if (std::isfinite(faceAnchoM) && std::isfinite(faceAltoM))
        {
            float areaM2 = faceAnchoM * faceAltoM;
            std::cout << "CARA frontal "
                << "zFront (frente) " << zFace
                << " slab (grosor) " << p.faceSlabM
                << " ancho " << faceAnchoM << " m " << (int)std::lround(faceAnchoM * 1000.0f) << " mm "
                << " alto " << faceAltoM << " m " << (int)std::lround(faceAltoM * 1000.0f) << " mm "
                << " area " << areaM2 << " m2"
                << "\n";
        }
        else
        {
            std::cout << "CARA frontal sin suficientes puntos para medir\n";
        }
    }

    std::ofstream f(filePath, std::ios::binary);
    if (!f.is_open()) return false;

    f << "ply\n";
    f << (p.plyBinary ? "format binary_little_endian 1.0\n" : "format ascii 1.0\n");
    f << "element vertex " << pts.size() << "\n";
    f << "property float x\n";
    f << "property float y\n";
    f << "property float z\n";
    f << "property uchar red\n";
    f << "property uchar green\n";
    f << "property uchar blue\n";
    f << "end_header\n";

    for (const auto& q : pts)
    {
        if (!p.plyBinary)
        {
            f << q.x << " " << q.y << " " << q.z << " "
                << (int)q.r << " " << (int)q.g << " " << (int)q.b << "\n";
        }
        else
        {
            f.write((char*)&q.x, sizeof(float));
            f.write((char*)&q.y, sizeof(float));
            f.write((char*)&q.z, sizeof(float));
            f.write((char*)&q.r, sizeof(uint8_t));
            f.write((char*)&q.g, sizeof(uint8_t));
            f.write((char*)&q.b, sizeof(uint8_t));
        }
    }

    std::cout << "PLY guardado " << filePath
        << " puntos " << pts.size()
        << " rango " << p.minRangeM << " a " << std::min(p.maxRangeM, p.hardMaxZM)
        << " colorMode " << p.colorMode
        << "\n";

    return true;
}

bool BBBDriver::GetDistanceCentralPointM(const ImageList& set, const Scan3DParams& s3d, float& outMeters)
{
    ImagePtr disp = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
    if (!disp || disp->IsIncomplete() || !disp->GetData()) return false;

    const int w = (int)disp->GetWidth();
    const int h = (int)disp->GetHeight();
    const int cx = w / 2;
    const int cy = h / 2;

    const unsigned int bpp = disp->GetBitsPerPixel();
    const uint8_t* d8 = (const uint8_t*)disp->GetData();
    const uint16_t* d16 = (const uint16_t*)disp->GetData();
    const int strideBytes = (int)disp->GetStride();
    const int strideU16 = strideBytes / (int)sizeof(uint16_t);

    uint16_t raw = 0;
    if (bpp <= 8) raw = d8[cy * strideBytes + cx];
    else raw = d16[cy * strideU16 + cx];

    if (raw == 0) return false;
    if (s3d.invalidFlag)
    {
        uint16_t inv = (uint16_t)s3d.invalidValue;
        if (raw == inv) return false;
    }

    float d = (float)raw * s3d.scale + s3d.offset;
    if (d <= 1e-6f) return false;

    float baselineM = BaselineToMeters(s3d.baseline);
    float z = (s3d.focal * baselineM) / d;
    if (!std::isfinite(z)) return false;

    outMeters = z;
    return true;
}

bool BBBDriver::GetDistanceToBultoM_Debug(
    const ImageList& set,
    const Scan3DParams& s3d,
    const BBBParams& p,
    const BBBCameraMount& mount,
    float& outMeters,
    int& outUsedPoints)
{
    outUsedPoints = 0;

    ImagePtr disp = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
    if (!disp || disp->IsIncomplete() || !disp->GetData()) return false;

    const int w = (int)disp->GetWidth();
    const int h = (int)disp->GetHeight();

    int x0, x1, y0, y1;
    ClampRoiXY(p, w, h, x0, x1, y0, y1);

    const unsigned int bpp = disp->GetBitsPerPixel();
    const uint8_t* d8 = (const uint8_t*)disp->GetData();
    const uint16_t* d16 = (const uint16_t*)disp->GetData();
    const int strideBytes = (int)disp->GetStride();
    const int strideU16 = strideBytes / (int)sizeof(uint16_t);

    auto ReadRawAt = [&](int x, int y) -> uint16_t
        {
            if (bpp <= 8) return (uint16_t)d8[y * strideBytes + x];
            return d16[y * strideU16 + x];
        };

    auto IsInvalidRaw = [&](uint16_t raw) -> bool
        {
            if (raw == 0) return true;
            if (s3d.invalidFlag)
            {
                uint16_t inv = (uint16_t)(s3d.invalidValue);
                if (raw == inv) return true;
            }
            return false;
        };

    float baselineM = BaselineToMeters(s3d.baseline);
    const float focal = s3d.focal;

    std::vector<float> depths;
    depths.reserve((x1 - x0) * (y1 - y0));

    float zHardMax = p.hardMaxZM;
    float zMaxUse = std::min(p.maxRangeM, zHardMax);

    for (int y = y0; y < y1; ++y)
    {
        for (int x = x0; x < x1; ++x)
        {
            uint16_t raw = ReadRawAt(x, y);
            if (IsInvalidRaw(raw)) continue;

            float d = (float)raw * s3d.scale + s3d.offset;
            if (d <= 1e-6f) continue;

            float z = (focal * baselineM) / d;
            if (!std::isfinite(z)) continue;

            if (z > zHardMax) continue;
            if (z < p.minRangeM || z > zMaxUse) continue;

            if (p.enableGroundPlaneFilter)
            {
                float X = ((float)x - s3d.principalU) * z / focal;
                float Y = ((float)y - s3d.principalV) * z / focal;

                float hAG = HeightAboveGroundM(X, Y, z, mount.alturaCamaraM, mount.pitchDeg);
                if (!std::isfinite(hAG)) continue;
                if (hAG < p.groundMinHeightM) continue;
            }

            depths.push_back(z);
            outUsedPoints++;
        }
    }

    if (depths.size() < 200) return false;

    std::vector<float> tmp = depths;
    outMeters = Percentile(tmp, p.bultoFacePercentile);
    return std::isfinite(outMeters);
}

bool BBBDriver::SetExposureUs(double exposureUs)
{
    if (!cam) return false;
    try
    {
        cam->ExposureAuto.SetValue(ExposureAuto_Off);
        cam->ExposureTime.SetValue(exposureUs);
        return true;
    }
    catch (...) { return false; }
}

bool BBBDriver::SetGainDb(double gainDb)
{
    if (!cam) return false;
    try
    {
        cam->GainAuto.SetValue(GainAuto_Off);
        cam->Gain.SetValue(gainDb);
        return true;
    }
    catch (...) { return false; }
}
