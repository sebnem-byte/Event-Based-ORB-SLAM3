#include "EventLoader.h"
#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <glog/logging.h>

using namespace boost::filesystem;
using namespace std;

namespace EORB_SLAM {

/* ================================ Constructor & Destructor ================================ */

EvEthzLoader::EvEthzLoader(const string &fSettings) {
    path pathSettings = path(fSettings);

    try {
        if (exists(pathSettings) && is_regular_file(pathSettings) &&
            (BaseLoader::checkExtension(pathSettings, ".yaml") || BaseLoader::checkExtension(pathSettings, ".yml"))) {
            
            mPathSettings = fSettings;
            LOG(INFO) << "📂 YAML settings file detected: " << fSettings;

            bool res = this->parseSettings(fSettings);
            if (!res) {
                LOG(ERROR) << "❌ Failed to parse settings file.";
                return;
            }

            LOG(INFO) << "✅ YAML parameters successfully loaded.";

        } else {
            LOG(ERROR) << "❌ Invalid settings file or does not exist: " << fSettings;
        }

    } catch (const boost::filesystem::filesystem_error& ex) {
        LOG(ERROR) << "❌ Filesystem error: " << ex.what();
    }
}

EvEthzLoader::~EvEthzLoader() {
    LOG(INFO) << "🗑️ EvEthzLoader Destructor Called!";
}

/* ================================ YAML Parsing ================================ */

bool EvEthzLoader::parseSettings(const string &settingsFile) {
    LOG(INFO) << "📖 Parsing YAML settings file: " << settingsFile;

    cv::FileStorage fsSettings(settingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        LOG(ERROR) << "❌ Failed to open YAML file: " << settingsFile;
        return false;
    }
    LOG(INFO) << "✅ YAML file opened successfully.";

    // ORB Vocabulary Path
    mPathOrbVoc = MyYamlParser::parseString(fsSettings, "Path.ORBvoc", "");
    if (mPathOrbVoc.empty()) {
        LOG(ERROR) << "❌ ORB Vocabulary path is empty! Check Path.ORBvoc in YAML.";
        return false;
    }
    LOG(INFO) << "✅ ORB Vocabulary Path: " << mPathOrbVoc;

    // Sensor Configuration
    mpConfig = std::make_shared<MySensorConfig>();
    string dsConf = MyYamlParser::parseString(fsSettings, "DS.config", "");
    mpConfig->setConfig(MySensorConfig::mapConfig(dsConf));
    LOG(INFO) << "📷 Sensor configuration: " << dsConf;

    // Camera Parameters
    mpCamParams = std::make_shared<MyCamParams>();
    if (!MyParameters::parseCameraParams(fsSettings, *mpCamParams, false, false)) {
        LOG(ERROR) << "❌ Failed to load camera parameters.";
        return false;
    }
    LOG(INFO) << "✅ Camera parameters loaded.";

    // Event Parameters
    mpEventParams = std::make_shared<EvParams>();
    if (!mpEventParams->parseParams(fsSettings)) {
        LOG(ERROR) << "❌ Failed to parse event parameters.";
        return false;
    }
    LOG(INFO) << "✅ Event parameters loaded.";

    // Viewer Parameters
    mpViewerParams = std::make_shared<MyViewerSettings>();
    if (!MyParameters::parseViewerParams(fsSettings, *mpViewerParams)) {
        LOG(ERROR) << "❌ Failed to load viewer parameters.";
        return false;
    }
    LOG(INFO) << "✅ Viewer parameters loaded.";

    // ORB Extractor Parameters
    mpORBParams = std::make_shared<MixedFtsParams>();
    if (!MyParameters::parseFeatureParams(fsSettings, *mpORBParams)) {
        LOG(ERROR) << "❌ Failed to load ORB parameters.";
        return false;
    }
    LOG(INFO) << "✅ ORB parameters loaded.";

    // IMU Parameters
    mpIMUParams = std::make_shared<MyIMUSettings>();
    if (!MyParameters::parseIMUParams(fsSettings, *mpIMUParams)) {
        LOG(ERROR) << "❌ Failed to load IMU parameters.";
        return false;
    }
    LOG(INFO) << "✅ IMU parameters loaded.";

    return true;
}

/* ================================ Getters ================================ */

std::string  EvEthzLoader::getPathOrbVoc() const {
    return mPathOrbVoc;
}

SensorConfigPtr EvEthzLoader::getConfigStat() const {
    return mpConfig;
}

EvParamsPtr EvEthzLoader::getEventParams() const {
    return mpEventParams;
}

ViewerParamsPtr EvEthzLoader::getViewerParams() const {
    return mpViewerParams;
}

CamParamsPtr EvEthzLoader::getCamParams() const {
    return mpCamParams;
}

std::pair<MyCalibPtr, MyCalibPtr> EvEthzLoader::getPairCamCalibrator() const {
    return mPairCalibrator;
}

ORBParamsPtr EvEthzLoader::getORBParams() const {
    return mpORBParams;
}

IMUParamsPtr EvEthzLoader::getIMUParams() const {
    return mpIMUParams;
}

}  // namespace EORB_SLAM
