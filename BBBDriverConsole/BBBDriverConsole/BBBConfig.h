#pragma once
#include <string>
#include <vector>
#include <cstdint>

struct BBBCameraMount
{
    float alturaCamaraM = 3.849f;
    float distHorizArc0M = 0.0f;
    float pitchDeg = 36.45f;
};

struct BBBParams
{
    float minRangeM = 1.0f;
    float maxRangeM = 5.5f;

    int roiMinXPct = 30;
    int roiMaxXPct = 70;
    int roiMinYPct = 10;
    int roiMaxYPct = 85;

    int decimationFactor = 1;

    bool applySpeckleFilter = true;
    int maxSpeckleSize = 900;
    int speckleThreshold = 20;

    bool applyMedian3x3 = true;

    float voxelLeafM = 0.01f;

    float outlierRadiusM = 0.08f;
    int outlierMinNeighbors = 10;

    bool keepLargestCluster = true;

    bool enableGroundPlaneFilter = true;
    float groundBandPct = 0.35f;
    float groundRansacThrM = 0.012f;
    int groundRansacIters = 300;
    float groundCutMarginM = 0.08f;

    bool enableFrontDepthClamp = true;
    float frontFacePercentile = 0.05f;
    float frontDepthBandM = 1.80f;

    float faceSlabM = 0.20f;

    float dimPercentileLow = 0.02f;
    float dimPercentileHigh = 0.98f;

    int colorMode = 2;
    bool plyBinary = true;

    float hardMaxZM = 6.0f;
    float groundMinHeightM = 0.08f;

    float bultoFacePercentile = 0.10f;
};

struct BBBControl
{
    double exposureUs = 800.0;
    double gainDb = 0.0;
};

struct BBBPaths
{
    std::string outputDir = ".";
    std::string dirPNG = "PNG";
    std::string dirPGM = "PGM";
    std::string dirPLY = "PLY";
    uint64_t captureTimeoutMs = 5000;
};

struct CameraConfig
{
    bool enabled = true;

    std::string serial;
    std::string name;

    BBBCameraMount mount;
    BBBParams params;
    BBBControl control;
};

struct BBBAppConfig
{
    BBBPaths paths;

    int maxCameras = 3;
    bool autoAddDetectedCameras = true;

    bool autoNameFromSerial = true;
    std::string namePrefix = "BBB";

    BBBCameraMount defaultMount;
    BBBParams defaultParams;
    BBBControl defaultControl;

    std::vector<CameraConfig> cameras;
};

class BBBConfig
{
public:
    static bool LoadIni(const std::string& iniPath, BBBAppConfig& out);
    static bool SaveIni(const std::string& iniPath, const BBBAppConfig& cfg);

    static bool EnsureDetectedCameras(
        BBBAppConfig& cfg,
        const std::vector<std::string>& detectedStereoSerials,
        bool& outChanged
    );

    static std::string MakeAutoName(const BBBAppConfig& cfg, const std::string& serial, int index1Based);
};
