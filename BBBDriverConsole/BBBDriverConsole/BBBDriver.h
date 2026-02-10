#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

#ifdef _WIN32
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#endif

#include <string>
#include <cstdint>

// TELEDYNE usamos Spinnaker y GenApi oficiales
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include "BBBConfig.h"

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
    BBBDriver() = default;
    ~BBBDriver();

    bool OpenBySerial(Spinnaker::CameraList& cams, const std::string& serial);
    bool OpenFirstStereoSkip(Spinnaker::CameraList& cams, const std::string& serialToSkip);

    void Close();

    bool DisableGVCPHeartbeat(bool disable);

    bool ConfigureStreams_Rectified1_Disparity();
    bool ConfigureSoftwareTrigger();
    bool ConfigureStreamBuffersNewestOnly();

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
        const BBBCameraMount& mount,
        const std::string& filePath
    );

    bool GetDistanceCentralPointM(const Spinnaker::ImageList& set, const Scan3DParams& s3d, float& outMeters);

    bool GetDistanceToBultoM_Debug(
        const Spinnaker::ImageList& set,
        const Scan3DParams& s3d,
        const BBBParams& p,
        const BBBCameraMount& mount,
        float& outMeters,
        int& outUsedPoints
    );

    bool SetExposureUs(double exposureUs);
    bool SetGainDb(double gainDb);

    Spinnaker::CameraPtr GetCamera() const { return cam; }

private:
    // TELEDYNE trabajamos con nodos GenICam oficiales
    static bool SetEnumAsString(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, const char* value);
    static bool TrySetEnumAny(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, const char* const* values, int nValues);

    static bool GetFloatNode(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, float& out);
    static bool GetBoolNode(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, bool& out);

    static bool ValidateSetHasRectDisp(const Spinnaker::ImageList& set);
    static void ClampRoiXY(const BBBParams& p, int w, int h, int& x0, int& x1, int& y0, int& y1);
    static float BaselineToMeters(float baselineMaybeMm);

private:
    bool acquiring = false;
    Spinnaker::CameraPtr cam;
};
