#pragma once

#include <string>
#include <cstdint>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

struct BBBParams
{
    float minRangeM = 0.5f;
    float maxRangeM = 6.0f;

    int roiMinPct = 35;
    int roiMaxPct = 65;

    int decimationFactor = 1;

    bool applySpeckleFilter = true;
    int maxSpeckleSize = 600;
    int speckleThreshold = 15;

    bool applyMedian3x3 = true;

    bool plyBinary = true;

    float voxelLeafM = 0.01f;
    float outlierRadiusM = 0.04f;
    int outlierMinNeighbors = 4;

    bool keepLargestCluster = true;

    int colorMode = 2;
};


struct Scan3DParams
{
    float scale = 1.0f;
    float offset = 0.0f;
    float focal = 0.0f;
    float baseline = 0.0f;
    float principalU = 0.0f;
    float principalV = 0.0f;
    bool invalidFlag = false;
    float invalidValue = 0.0f;
};

class BBBDriver
{
public:
    BBBDriver();
    ~BBBDriver();

    bool OpenFirstStereo();
    bool OpenBySerial(const std::string& serial);
    void Close();

    bool ConfigureStreams_Rectified1_Disparity();

    bool ConfigureSoftwareTrigger();
    bool DisableGVCPHeartbeat(bool disable);

    bool ReadScan3DParams(Scan3DParams& out);

    bool StartAcquisition();
    void StopAcquisition();

    bool CaptureOnceSync(Spinnaker::ImageList& outSet, uint64_t timeoutMs);

    bool SaveDisparityPGM(const Spinnaker::ImageList& set, const std::string& filePath);
    bool SaveRectifiedPNG(const Spinnaker::ImageList& set, const std::string& filePath);

    bool SavePointCloudPLY_Filtered(
        const Spinnaker::ImageList& set,
        const Scan3DParams& s3d,
        const BBBParams& p,
        const std::string& filePath
    );

    bool GetDistanceCentralPointM(const Spinnaker::ImageList& set, const Scan3DParams& s3d, float& outMeters);

    bool GetDistanceToBultoM(
        const Spinnaker::ImageList& set,
        const Scan3DParams& s3d,
        const BBBParams& p,
        float& outMeters
    );

    bool GetDistanceToBultoM_Debug(
        const Spinnaker::ImageList& set,
        const Scan3DParams& s3d,
        const BBBParams& p,
        float& outMeters,
        int& outUsedPoints
    );

    bool SetExposureUs(double exposureUs);
    bool SetGainDb(double gainDb);

    Spinnaker::CameraPtr GetCamera() const;

private:
    static bool SetEnumAsString(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, const char* value);
    static bool TrySetEnumAny(Spinnaker::GenApi::INodeMap& nodeMap, const char* name,
        const char* const* values, int nValues);

    static bool GetFloatNode(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, float& out);
    static bool GetBoolNode(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, bool& out);

    static void ClampRoi(const BBBParams& p, int w, int h, int& x0, int& x1, int& y0, int& y1);
    static float BaselineToMeters(float baselineMaybeMm);

    static bool ValidateSetHasRectDisp(const Spinnaker::ImageList& set);

private:
    bool acquiring = false;
    Spinnaker::SystemPtr system;
    Spinnaker::CameraPtr cam;
};
