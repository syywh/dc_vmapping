#ifndef YGZ_KEYFRAMEDATABASE_H
#define YGZ_KEYFRAMEDATABASE_H

#include "ORBVocabulary.h"

#include <mutex>
#include <vector>
#include <list>

// Keyframe 数据库，基本没动
namespace vill {

    class KeyFrame;

    class Frame;


    class KeyFrameDatabase {
    public:

        KeyFrameDatabase(const ORBVocabulary &voc);

        void add(KeyFrame *pKF);

        void erase(KeyFrame *pKF);

        void clear();

        // Loop Detection
        std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);

        // Relocalization
        std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F);

    protected:

        // Associated vocabulary
        const ORBVocabulary *mpVoc; ///< 预先训练好的词典

        // Inverted file
        std::vector<list < KeyFrame * > >
        mvInvertedFile; ///< 倒排索引，mvInvertedFile[i]表示包含了第i个word id的所有关键帧

        // Mutex
        std::mutex mMutex;
    };

} //namespace vill

#endif
