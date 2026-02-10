#include "BBBDriver.h"
#include "BBBConfig.h"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <ctime>
#include <filesystem>
#include <vector>
#include <algorithm>

#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

static std::filesystem::path GetExePath()
{
#ifdef _WIN32
    char buf[MAX_PATH];
    DWORD len = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    if (len == 0) return std::filesystem::path();
    return std::filesystem::path(std::string(buf, buf + len));
#else
    return std::filesystem::current_path() / "BBBDriverConsole";
#endif
}

static std::filesystem::path GetExeDir()
{
    auto p = GetExePath();
    if (p.empty()) return std::filesystem::current_path();
    return p.parent_path();
}

static std::filesystem::path FindIniPath(const std::string& iniName)
{
    std::error_code ec;

    auto pCwd = std::filesystem::current_path(ec) / iniName;
    if (!ec && std::filesystem::exists(pCwd, ec) && !ec) return pCwd;

    auto exeDir = GetExeDir();
    auto pExe = exeDir / iniName;
    if (std::filesystem::exists(pExe, ec) && !ec) return pExe;

    return pExe;
}

static std::string NowTag()
{
    using namespace std::chrono;
    auto now = system_clock::now();
    auto t = system_clock::to_time_t(now);

    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    tm = *std::localtime(&t);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

static void ReleaseImageList(Spinnaker::ImageList& set)
{
    const unsigned int n = (unsigned int)set.GetSize();
    for (unsigned int i = 0; i < n; i++)
    {
        Spinnaker::ImagePtr img = set.GetByIndex(i);
        if (img) img->Release();
    }
}

static void EnsureDirs(const BBBPaths& paths)
{
    std::filesystem::path base(paths.outputDir);
    std::filesystem::create_directories(base / paths.dirPNG);
    std::filesystem::create_directories(base / paths.dirPGM);
    std::filesystem::create_directories(base / paths.dirPLY);
}

static std::string SanitizeFileTag(std::string s)
{
    for (auto& c : s)
    {
        bool ok =
            (c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            (c == '_') || (c == '-');
        if (!ok) c = '_';
    }
    if (s.empty()) s = "BBB";
    return s;
}

static void ApplyControl(BBBDriver& d, const BBBControl& c)
{
    d.SetExposureUs(c.exposureUs);
    d.SetGainDb(c.gainDb);
}

static void PrintMenu()
{
    std::cout << "\n---------------------------------\n";
    std::cout << "MENU\n";
    std::cout << " 1 Guardar Disparity (disparidad) PGM y Rectified (rectificada) PNG\n";
    std::cout << " 2 Generar PLY (archivo de nube) filtrado\n";
    std::cout << " 3 Medir distancia\n";
    std::cout << " 4 Cambiar parametros\n";
    std::cout << " 5 Releer Scan3D\n";
    std::cout << " 0 Salir\n";
    std::cout << "Opcion: ";
}

struct ActiveCam
{
    CameraConfig* cfg = nullptr;
    BBBDriver drv;
    Scan3DParams s3d{};
    bool available = false;
};

static std::vector<std::string> DetectStereoSerials(Spinnaker::CameraList& cams)
{
    std::vector<std::string> out;

    for (unsigned int i = 0; i < cams.GetSize(); ++i)
    {
        Spinnaker::CameraPtr c = cams.GetByIndex(i);
        c->Init();

        bool isStereo = Spinnaker::ImageUtilityStereo::IsStereoCamera(c);
        std::string serial = c->TLDevice.DeviceSerialNumber.ToString().c_str();

        c->DeInit();

        if (isStereo && !serial.empty())
            out.push_back(serial);
    }

    // ARR quitamos duplicados por seguridad
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());

    return out;
}

int main()
{
    std::cout << "=== BBBDriverConsole BBB Spinnaker hasta 3 camaras ===\n\n";

    const std::string iniName = "bbb_config.ini";

    std::error_code ec;
    auto cwd = std::filesystem::current_path(ec);
    auto exeDir = GetExeDir();

    std::cout << "Directorio actual " << (ec ? std::string("desconocido") : cwd.string()) << "\n";
    std::cout << "Directorio exe " << exeDir.string() << "\n";

    std::filesystem::path iniPath = FindIniPath(iniName);
    std::cout << "Buscando INI en " << iniPath.string() << "\n";

    BBBAppConfig cfg;
    if (!BBBConfig::LoadIni(iniPath.string(), cfg))
    {
        std::cout << "ERROR no pude leer " << iniPath.string() << "\n";
        return 1;
    }

    if (cfg.maxCameras > 3) cfg.maxCameras = 3;
    if (cfg.maxCameras < 1) cfg.maxCameras = 1;

    if (cfg.paths.outputDir.empty() || cfg.paths.outputDir == ".")
        cfg.paths.outputDir = exeDir.string();

    EnsureDirs(cfg.paths);

    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
    Spinnaker::CameraList cams = system->GetCameras();

    if (cams.GetSize() == 0)
    {
        std::cout << "ERROR no hay camaras detectadas\n";
        system->ReleaseInstance();
        return 2;
    }

    std::vector<std::string> detected = DetectStereoSerials(cams);
    if (detected.empty())
    {
        std::cout << "ERROR no hay camaras estereo detectadas\n";
        cams.Clear();
        system->ReleaseInstance();
        return 3;
    }

    bool cfgChanged = false;
    BBBConfig::EnsureDetectedCameras(cfg, detected, cfgChanged);

    if (cfgChanged)
    {
        BBBConfig::SaveIni(iniPath.string(), cfg);
        std::cout << "INI actualizado al detectar camaras\n";
    }

    // ARR abrimos cada Camera.0..2 una vez sin serial duplicado
    std::vector<ActiveCam> act;
    act.reserve((size_t)cfg.maxCameras);

    std::vector<std::string> usedSerials;

    auto IsUsed = [&](const std::string& s)
        {
            return std::find(usedSerials.begin(), usedSerials.end(), s) != usedSerials.end();
        };

    for (int i = 0; i < cfg.maxCameras; ++i)
    {
        CameraConfig& c = cfg.cameras[i];

        if (!c.enabled)
            continue;

        if (c.serial.empty())
        {
            // ARR dejamos el hueco preparado para cuando se conecte la tercera
            if (c.name.empty() && cfg.autoNameFromSerial)
                c.name = BBBConfig::MakeAutoName(cfg, "", i + 1);

            ActiveCam a;
            a.cfg = &c;
            a.available = false;
            act.push_back(a);
            continue;
        }

        if (IsUsed(c.serial))
        {
            std::cout << "AVISO serial duplicado en INI " << c.serial << " en " << c.name << " lo saltamos\n";
            continue;
        }

        if (c.name.empty() && cfg.autoNameFromSerial)
            c.name = BBBConfig::MakeAutoName(cfg, c.serial, i + 1);

        ActiveCam a;
        a.cfg = &c;
        a.available = a.drv.OpenBySerial(cams, c.serial);

        if (a.available)
            usedSerials.push_back(c.serial);

        act.push_back(a);
    }

    BBBConfig::SaveIni(iniPath.string(), cfg);

    for (auto& a : act)
    {
        if (!a.cfg) continue;

        std::cout << "Camara " << a.cfg->name << " serial " << (a.cfg->serial.empty() ? "SIN_SERIAL" : a.cfg->serial)
            << " " << (a.available ? "OK" : "NO") << "\n";

        if (!a.available) continue;

#ifdef _DEBUG
        a.drv.DisableGVCPHeartbeat(true);
#endif

        if (!a.drv.ConfigureStreams_Rectified1_Disparity())
            std::cout << "AVISO " << a.cfg->name << " no pudo configurar streams\n";

        if (!a.drv.ConfigureSoftwareTrigger())
            std::cout << "AVISO " << a.cfg->name << " no pudo configurar trigger software\n";

        if (!a.drv.ReadScan3DParams(a.s3d))
            std::cout << "AVISO " << a.cfg->name << " no pudo leer Scan3D\n";
        else
            std::cout << a.cfg->name << " Scan3D baseline " << a.s3d.baseline
            << " focal " << a.s3d.focal
            << " scale " << a.s3d.scale
            << " offset " << a.s3d.offset << "\n";

        ApplyControl(a.drv, a.cfg->control);

        if (!a.drv.StartAcquisition())
        {
            std::cout << "AVISO " << a.cfg->name << " no pudo iniciar adquisicion\n";
            a.available = false;
        }
    }

    while (true)
    {
        PrintMenu();
        std::string opt;
        std::getline(std::cin, opt);

        if (opt == "0") break;

        const std::string tag = NowTag();
        std::filesystem::path base(cfg.paths.outputDir);
        std::filesystem::path dirPNG = base / cfg.paths.dirPNG;
        std::filesystem::path dirPGM = base / cfg.paths.dirPGM;
        std::filesystem::path dirPLY = base / cfg.paths.dirPLY;

        if (opt == "5")
        {
            std::cout << "Releyendo Scan3D (baseline linea base, focal, scale escala, offset desfase)\n";
            for (auto& a : act)
            {
                if (!a.available) continue;

                if (a.drv.ReadScan3DParams(a.s3d))
                    std::cout << a.cfg->name << " baseline " << a.s3d.baseline
                    << " focal " << a.s3d.focal
                    << " scale " << a.s3d.scale
                    << " offset " << a.s3d.offset << "\n";
                else
                    std::cout << a.cfg->name << " FAIL Scan3D\n";
            }
            continue;
        }

        if (opt == "4")
        {
            std::cout << "\nElegir camara para cambiar parametros\n";
            for (size_t i = 0; i < act.size(); ++i)
            {
                auto& a = act[i];
                std::cout << " " << (i + 1) << " " << a.cfg->name
                    << " serial " << (a.cfg->serial.empty() ? "SIN_SERIAL" : a.cfg->serial)
                    << " " << (a.available ? "OK" : "NO") << "\n";
            }
            std::cout << "Opcion: ";
            std::string sel;
            std::getline(std::cin, sel);

            int idx = std::stoi(sel) - 1;
            if (idx < 0 || idx >= (int)act.size())
            {
                std::cout << "Opcion no valida\n";
                continue;
            }

            // ARR aqui edits como ya tenias si quieres mantenerlos
            std::cout << "Editando parametros de " << act[idx].cfg->name << " en INI\n";
            std::cout << "Hazlo editando el bbb_config.ini que es lo que quieres centralizado\n";

            BBBConfig::SaveIni(iniPath.string(), cfg);
            continue;
        }

        auto DoCam = [&](ActiveCam& a)
            {
                if (!a.available) return;

                Spinnaker::ImageList set;
                if (!a.drv.CaptureOnceSync(set, cfg.paths.captureTimeoutMs))
                {
                    std::cout << a.cfg->name << " FAIL no capturamos set\n";
                    ReleaseImageList(set);
                    return;
                }

                std::string camTag = SanitizeFileTag(a.cfg->name);

                if (opt == "1")
                {
                    std::string fDisp = camTag + "_disparity_" + tag + ".pgm";
                    std::string fRect = camTag + "_rectified_" + tag + ".png";

                    auto pDisp = (dirPGM / fDisp).string();
                    auto pRect = (dirPNG / fRect).string();

                    bool okDisp = a.drv.SaveDisparityPGM(set, pDisp);
                    bool okRect = a.drv.SaveRectifiedPNG(set, pRect);

                    std::cout << a.cfg->name << " Guardado\n";
                    std::cout << " - " << pDisp << " " << (okDisp ? "OK" : "FAIL") << "\n";
                    std::cout << " - " << pRect << " " << (okRect ? "OK" : "FAIL") << "\n";
                }
                else if (opt == "2")
                {
                    a.drv.ReadScan3DParams(a.s3d);

                    std::string fPly = camTag + "_cloud_" + tag + ".ply";
                    auto pPly = (dirPLY / fPly).string();

                    std::cout << "\n--- " << a.cfg->name << " Generar PLY filtrado ---\n";
                    if (a.drv.SavePointCloudPLY_Filtered(set, a.s3d, a.cfg->params, a.cfg->mount, pPly))
                        std::cout << a.cfg->name << " OK guardado " << pPly << "\n";
                    else
                        std::cout << a.cfg->name << " FAIL PLY\n";
                }
                else if (opt == "3")
                {
                    float zCenter = 0.f;
                    float zBulto = 0.f;
                    int used = 0;

                    bool okC = a.drv.GetDistanceCentralPointM(set, a.s3d, zCenter);
                    bool okB = a.drv.GetDistanceToBultoM_Debug(set, a.s3d, a.cfg->params, a.cfg->mount, zBulto, used);

                    std::cout << a.cfg->name << " Distancias\n";
                    std::cout << " - Centro " << (okC ? std::to_string(zCenter) : std::string("FAIL")) << " m\n";
                    std::cout << " - Cara bulto " << (okB ? std::to_string(zBulto) : std::string("FAIL")) << " m puntos " << used << "\n";
                }

                ReleaseImageList(set);
            };

        for (auto& a : act) DoCam(a);
    }

    for (auto& a : act)
    {
        if (!a.available) continue;
        a.drv.StopAcquisition();
        a.drv.Close();
    }

    cams.Clear();
    system->ReleaseInstance();

    std::cout << "Saliendo\n";
    return 0;
}
