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

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

// Parametros que usamos para filtrar y medir el bulto
struct BBBParams
{
    // Rango de Z en metros que aceptamos para nube y medicion
    float minRangeM = 1.0f;
    float maxRangeM = 5.5f;

    // ROI en porcentaje en X para recortar bordes con ruido
    int roiMinXPct = 30;
    int roiMaxXPct = 70;

    // ROI en porcentaje en Y para recortar arriba y abajo
    int roiMinYPct = 10;
    int roiMaxYPct = 85;

    // Decimacion en pixeles para acelerar
    int decimationFactor = 1;

    // Filtro speckle del SDK para quitar puntos sueltos en disparity
    bool applySpeckleFilter = true;
    int maxSpeckleSize = 900;
    int speckleThreshold = 20;

    // Mediana 3x3 en disparity para reducir ruido local
    bool applyMedian3x3 = true;

    // Voxel grid en metros para bajar densidad
    float voxelLeafM = 0.01f;

    // Outlier removal por radio en metros y minimo de vecinos
    float outlierRadiusM = 0.08f;
    int outlierMinNeighbors = 10;

    // Nos quedamos con el cluster mas grande
    bool keepLargestCluster = true;

    // Guardado PLY binario o ascii
    bool plyBinary = true;

    // 0 gris fijo 1 rectifiedGray 2 heatmapZ
    int colorMode = 2;

    // Corte de fondo desde la cara frontal para quitar puntos lejanos
    bool enableFrontDepthClamp = true;
    float frontFacePercentile = 0.05f;
    float frontDepthBandM = 1.80f;

    // Grosor en Z para medir cara frontal
    float faceSlabM = 0.20f;

    // Percentiles para medir ancho y alto sin outliers
    float dimPercentileLow = 0.02f;
    float dimPercentileHigh = 0.98f;

    // Filtro de suelo por plano detectado con RANSAC
    bool enableGroundPlaneFilter = true;

    // Banda inferior que usamos para buscar suelo por porcentaje de rango de Y
    // 0.20 significa que buscamos candidatos en el 20% mas bajo de los puntos
    float groundBandPct = 0.35f;

    // Umbral en metros para considerar inlier del plano
    float groundRansacThrM = 0.012f;

    // Iteraciones de RANSAC
    int groundRansacIters = 300;

    // Margen por encima del plano para cortar suelo
    float groundCutMarginM = 0.08f;

    // Pitch de camara en grados para ayudar al prior
    float cameraPitchDeg = 36.45f;
};

// Parametros Scan3D que leemos del nodemap
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

    // Abrimos la primera camara estereo que encontremos
    bool OpenFirstStereo();

    // Abrimos camara estereo por serial
    bool OpenBySerial(const std::string& serial);

    // Cerramos camara y paramos adquisicion
    void Close();

    // Desactivamos heartbeat para debug si hace falta
    bool DisableGVCPHeartbeat(bool disable);

    // Activamos streams rectified y disparity en sensor1
    bool ConfigureStreams_Rectified1_Disparity();

    // Activamos trigger por software
    bool ConfigureSoftwareTrigger();

    // Configuramos buffers a NewestOnly para no acumular frames viejos
    bool ConfigureStreamBuffersNewestOnly();

    // Leemos parametros Scan3D del firmware
    bool ReadScan3DParams(Scan3DParams& out);

    // Iniciamos adquisicion
    bool StartAcquisition();

    // Paramos adquisicion
    void StopAcquisition();

    // Capturamos set sincronizado con trigger software
    bool CaptureOnceSync(Spinnaker::ImageList& outSet, uint64_t timeoutMs);

    // Guardamos disparity para debug
    bool SaveDisparityPGM(const Spinnaker::ImageList& set, const std::string& filePath);

    // Guardamos rectified para debug
    bool SaveRectifiedPNG(const Spinnaker::ImageList& set, const std::string& filePath);

    // Generamos nube filtrada y la guardamos en PLY
    bool SavePointCloudPLY_Filtered(
        const Spinnaker::ImageList& set,
        const Scan3DParams& s3d,
        const BBBParams& p,
        const std::string& filePath
    );

    // Medimos distancia del punto central
    bool GetDistanceCentralPointM(const Spinnaker::ImageList& set, const Scan3DParams& s3d, float& outMeters);

    // Medimos distancia a cara del bulto con percentil en ROI
    bool GetDistanceToBultoM_Debug(
        const Spinnaker::ImageList& set,
        const Scan3DParams& s3d,
        const BBBParams& p,
        float& outMeters,
        int& outUsedPoints
    );

    // Seteamos exposicion en microsegundos
    bool SetExposureUs(double exposureUs);

    // Seteamos ganancia en dB
    bool SetGainDb(double gainDb);

    // Devolvemos puntero a camara
    Spinnaker::CameraPtr GetCamera() const;

private:
    // Seteamos un enum por string
    static bool SetEnumAsString(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, const char* value);

    // Probamos varios nombres por compatibilidad
    static bool TrySetEnumAny(Spinnaker::GenApi::INodeMap& nodeMap, const char* name,
        const char* const* values, int nValues);

    // Leemos nodo float
    static bool GetFloatNode(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, float& out);

    // Leemos nodo bool
    static bool GetBoolNode(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, bool& out);

    // Validamos que set tiene rectified y disparity
    static bool ValidateSetHasRectDisp(const Spinnaker::ImageList& set);

    // Calculamos ROI en pixeles a partir de porcentajes
    static void ClampRoiXY(const BBBParams& p, int w, int h, int& x0, int& x1, int& y0, int& y1);

    // Convertimos baseline a metros si venia en mm
    static float BaselineToMeters(float baselineMaybeMm);

private:
    bool acquiring = false;
    Spinnaker::SystemPtr system;
    Spinnaker::CameraPtr cam;
};
