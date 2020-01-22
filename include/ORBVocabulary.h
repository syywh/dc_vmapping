#ifndef ORBVOCABULARY_H_
#define ORBVOCABULARY_H_

#include "thirdparty/DBoW2/DBoW2/FORB.h"
#include "thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

// ORB 字典

namespace vill {

    typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
            ORBVocabulary;

} //namespace vill

#endif // ORBVOCABULARY_H
