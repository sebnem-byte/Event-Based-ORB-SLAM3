#include "compiler_options.h"
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dvs_msgs/EventArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "System.h"
#include "EventLoader.h"
#include <chrono>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

// 📌 **Global Değişkenler**
vector<EORB_SLAM::EventData> vEvBuff;  // Event verisi buffer'ı
bool new_event_data = false;            // Yeni veri geldiğini takip eden flag

// 📌 **SLAM Sistemi Global Nesne Olarak Tanımlandı**
ORB_SLAM3::System* SLAM = nullptr;

// 📌 **Event Callback Fonksiyonu**
static int eventBatchCount = 0;
const int batchThreshold = 3;  // 3 event callback'te bir kez SLAM'e gönder

void eventCallback(const dvs_msgs::EventArray::ConstPtr& event_msg) {
    if (event_msg->events.empty()) {
        ROS_WARN("❌ Gelen event mesajı boş!");
        return;
    }

    vEvBuff.reserve(event_msg->events.size());
    for (const auto& ev : event_msg->events) {
        vEvBuff.emplace_back(EORB_SLAM::EventData{static_cast<double>(ev.ts.toSec()), ev.x, ev.y, ev.polarity});
    }

    // 📌 **Event timestamp sıralamasını garantileyelim**
    std::sort(vEvBuff.begin(), vEvBuff.end(), [](const EORB_SLAM::EventData& a, const EORB_SLAM::EventData& b) {
        return a.ts < b.ts;
    });

    // 📌 **Timestamp normalizasyonu**
    double first_ts = vEvBuff.front().ts;
    for (auto &ev : vEvBuff) {
        ev.ts -= first_ts;
    }

    // 📌 **Event zaman farklarını kontrol edelim**
    double ts_diff = vEvBuff.back().ts - vEvBuff.front().ts;
    ROS_INFO("📡 İlk Event TS: %f", vEvBuff.front().ts);
    ROS_INFO("📡 Son Event TS: %f", vEvBuff.back().ts);
    ROS_INFO("📡 Event zaman farkı: %f", ts_diff);

    // 📌 **Eğer event zaman farkı çok küçükse event biriktir**
    static std::vector<EORB_SLAM::EventData> eventBuffer;
    static double accumulatedTs = 0.0;
    eventBuffer.insert(eventBuffer.end(), vEvBuff.begin(), vEvBuff.end());
    accumulatedTs += ts_diff;

    if (accumulatedTs < 0.2) {  // 🔴 En az 0.2s event toplanmadan SLAM'e gönderme!
        ROS_WARN("⚠️ Event zaman farkı hala çok küçük (%f s), event'ler biriktiriliyor.", accumulatedTs);
        return;
    }

    // 📌 **Event'ler yeterince toplandı, SLAM'e gönder**
    vEvBuff = eventBuffer;  // Eventleri gönderilecek buffer'a kopyala
    eventBuffer.clear();
    accumulatedTs = 0.0;  // Sayaç sıfırla

    new_event_data = true;
    ROS_INFO("✅ %zu event işlendi ve SLAM'e gönderildi.", vEvBuff.size());
}



// 📌 **Main Function**
int main(int argc, char **argv) {
    ros::init(argc, argv, "eorbslam_node");
    ros::NodeHandle nh;

    if (argc < 2) {
        ROS_ERROR("❌ Kullanım: rosrun EORB_SLAM3 eorbslam_node path_to_settings.yaml");
        return 1;
    }

    const string settingsFile = argv[1];
    ROS_INFO("📂 Settings file path: %s", settingsFile.c_str());

    // 📌 **SLAM İçin Parametreleri Yükle**
    EORB_SLAM::EvEthzLoader evEthzLoader(settingsFile);

    const EORB_SLAM::SensorConfigPtr dsConf = evEthzLoader.getConfigStat();
    const EORB_SLAM::EvParamsPtr evParams = evEthzLoader.getEventParams();

    if (!dsConf || !evParams) {
        ROS_ERROR("❌ SLAM için gerekli parametreler eksik!");
        return 1;
    }

    // 📌 **SLAM Sistemi Başlat**
    SLAM = new ORB_SLAM3::System(
        evEthzLoader.getPathOrbVoc(), dsConf, 
        evEthzLoader.getCamParams(),
        evEthzLoader.getPairCamCalibrator(), 
        evEthzLoader.getORBParams(),
        evEthzLoader.getIMUParams(),
        evEthzLoader.getViewerParams(), 
        evParams
    );

    if (!SLAM) {
        ROS_ERROR("❌ SLAM başlatılamadı!");
        return 1;
    }

    ROS_INFO("✅ ORB Vocabulary başarıyla yüklendi!");

    // 📌 **Event Mesajları İçin Subscriber Başlat**
    ros::Subscriber event_sub = nh.subscribe("/dvs/events", 100, eventCallback);

    if (!event_sub.getTopic().empty()) {
        ROS_INFO("📡 Subscriber başarıyla bağlandı! (%s)", event_sub.getTopic().c_str());
    } else {
        ROS_ERROR("❌ Subscriber bağlanamadı! /dvs/events aktif mi?");
    }

    // 📌 **Ana Döngü: SLAM'e Event Verisini Aktarma**
    ros::Rate rate(100); // 100 Hz döngü hızı
    int last_event_count = 0;
    
    while (ros::ok()) {
        ros::spinOnce(); // 📌 Callback fonksiyonlarını çalıştır
    

        // 📌 **SLAM durumu kontrolü**
        // 📌 **SLAM durumu kontrolü**
        // 📌 **SLAM durumu kontrolü**
        // 📌 **SLAM durumu kontrolü**
        // 📌 **SLAM durumu kontrolü**
    if (SLAM) {
        // SLAM'in takip ettiği durumu logla
        int trackingState = SLAM->GetTrackingState();
        ROS_INFO("📌 SLAM Tracking State: %d", trackingState);

        // SLAM'in kaç tane harita noktası takip ettiğini kontrol et
        vector<ORB_SLAM3::MapPoint*> trackedMapPoints = SLAM->GetTrackedMapPoints();
        ROS_INFO("📌 SLAM takip edilen harita noktaları: %zu", trackedMapPoints.size());

        // Eğer takip edilen nokta sayısı sıfırsa, SLAM muhtemelen çalışmıyor
        if (trackedMapPoints.empty()) {
            ROS_ERROR("🚨 SLAM takip edilen harita noktası yok! Event'ler işlenmiyor olabilir.");
        }
    }

        // 📌 **Event verisi SLAM'e gönderilirken kontrol edelim**
    if (new_event_data) {
        if (!vEvBuff.empty()) {
            // 📌 **Event timestamp'lerini sıralayalım**
            std::sort(vEvBuff.begin(), vEvBuff.end(), [](const EORB_SLAM::EventData& a, const EORB_SLAM::EventData& b) {
                return a.ts < b.ts;
            });

            // 📌 **Event verisini logla**
            ROS_INFO("📡 İlk Event: x=%d, y=%d, ts=%f, polarity=%d", 
                     vEvBuff.front().x, vEvBuff.front().y, vEvBuff.front().ts, vEvBuff.front().p);
            ROS_INFO("📡 Son Event: x=%d, y=%d, ts=%f, polarity=%d", 
                     vEvBuff.back().x, vEvBuff.back().y, vEvBuff.back().ts, vEvBuff.back().p);

            // 📌 **SLAM'e event verisini gönder**
            ROS_INFO("🚀 SLAM'e event verisi gönderiliyor... Event sayısı: %zu", vEvBuff.size());
            SLAM->TrackEvent(vEvBuff);

            new_event_data = false; // 📌 Event işlendi, flag sıfırla
            ROS_INFO("✅ SLAM'e event verisi başarıyla gönderildi.");
        } else {
            ROS_WARN("⚠️ Yeni event verisi gelmedi, SLAM çağrılmadı!");
        }
    }




    
        rate.sleep();
    }

    // 📌 **SLAM Sonuçlarını Kaydet ve Kapat**
    SLAM->SaveTrajectoryEvent("/home/sebnem/trajectory.txt");
    SLAM->SaveMap("/home/sebnem/orb_map.txt");

    SLAM->Shutdown();

    return 0;
}
