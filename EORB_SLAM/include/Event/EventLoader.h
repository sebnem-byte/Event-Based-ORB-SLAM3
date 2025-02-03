#ifndef ORB_SLAM3_EVENTLOADER_H
#define ORB_SLAM3_EVENTLOADER_H

#include "EventData.h"
#include "DataStore.h"
#include "MyCalibrator.h"
#include <memory>

namespace EORB_SLAM {

    using ORBParamsPtr = std::shared_ptr<MixedFtsParams>; // ✅ ORBParamsPtr tanımı

    class EvEthzLoader {
    public:
        explicit EvEthzLoader(const std::string &fSettings);
        ~EvEthzLoader();

        std::string getPathOrbVoc() const;
        SensorConfigPtr getConfigStat() const;
        EvParamsPtr getEventParams() const;
        ViewerParamsPtr getViewerParams() const;
        CamParamsPtr getCamParams() const;
        std::pair<MyCalibPtr, MyCalibPtr> getPairCamCalibrator() const;
        ORBParamsPtr getORBParams() const;  // ✅ ORBParamsPtr artık tanımlı
        IMUParamsPtr getIMUParams() const;

    private:
        std::string mPathSettings;
        std::string mPathOrbVoc;
        SensorConfigPtr mpConfig;
        EvParamsPtr mpEventParams;
        ViewerParamsPtr mpViewerParams;
        CamParamsPtr mpCamParams;
        std::pair<MyCalibPtr, MyCalibPtr> mPairCalibrator;
        ORBParamsPtr mpORBParams;  // ✅ ORBParamsPtr artık kullanılabilir
        IMUParamsPtr mpIMUParams;

        bool parseSettings(const std::string &settingsFile);
    };

} // namespace EORB_SLAM

#endif // ORB_SLAM3_EVENTLOADER_H

