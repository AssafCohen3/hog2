#ifndef BUCKETOPENCLOSED_H
#define BUCKETOPENCLOSED_H

#include <stdint.h>
#include <limits>
#include "BucketBasedList.h"

template<typename state, class environment, class dataStructure = BucketNodeData<state> >
class BucketOpenClosedList : public BucketBasedList<state, environment, dataStructure> {

    using Base = BucketBasedList<state, environment, dataStructure>;

public:
    BucketOpenClosedList() : Base() {}

    ~BucketOpenClosedList() {}

};

/**
  * number of expandable nodes such that f(n) <= f and g(n) < g
  */
template<typename state, class environment, class dataStructure>
int BucketOpenClosedList<state, environment, dataStructure>::expandableNodes(double f, double g) {

    int nodeCount = 0;
    for (const auto &flayer : Base::fLayers) {
        if (flayer.first > f)
            break;

        for (const auto &bucket : flayer.second) {
            if (bucket.first >= g)
                break;
            nodeCount += bucket.second.size();
        }

    }

    return nodeCount;

}

#endif
