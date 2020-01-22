#ifndef YGZ_MAP_H_
#define YGZ_MAP_H_

#include <mutex>
#include <set>
#include <vector>

// 地图
namespace vill {

    class MapPoint;

    class KeyFrame;

    // Sort KeyFrames with mnId in mspKeyFrames
    class KFIdComapre {
    public:
        bool operator()(const KeyFrame *kfleft, const KeyFrame *kfright) const;
    };

    class Map {
    public:
        Map();

        void AddKeyFrame(KeyFrame *pKF);

        void AddMapPoint(MapPoint *pMP);

        void EraseMapPoint(MapPoint *pMP);

        void EraseKeyFrame(KeyFrame *pKF);

        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

        void InformNewBigChange();

        int GetLastBigChangeIdx();

        std::vector<KeyFrame *> GetAllKeyFrames();

        std::vector<MapPoint *> GetAllMapPoints();

        std::vector<MapPoint *> GetReferenceMapPoints();

        long unsigned int MapPointsInMap();

        long unsigned KeyFramesInMap();

        long unsigned int GetMaxKFid();

        void clear();

        std::vector<KeyFrame *> mvpKeyFrameOrigins;

        std::mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;

    protected:
        std::set<MapPoint *> mspMapPoints;
        std::set<KeyFrame *, KFIdComapre> mspKeyFrames;

        std::vector<MapPoint *> mvpReferenceMapPoints;

        long unsigned int mnMaxKFid;

        // Index related to a big change in the map (loop closure, global BA)
        int mnBigChangeIdx;

        std::mutex mMutexMap;
    };

} //namespace vill

#endif // MAP_H
