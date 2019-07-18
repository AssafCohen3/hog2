#ifndef THREEDIMBUCKETBASEDLIST_H
#define THREEDIMBUCKETBASEDLIST_H

#include <cassert>
#include <vector>
#include <map>
#include <unordered_map>
#include <stdint.h>
#include <limits>
#include <functional>
#include "BucketBasedList.h"

enum MinCriterion {
    MinG, MinF, MinD
};

/*
 * Strong assumptions regarding the cached state have been made for this class
 * for example, whenever a bucket is created or emptied the cached state must be set to dirty
 * this has to be taken into account when modifying the usage of this class
 */
template<typename state, class environment, class dataStructure = BucketNodeData<state> >
class ThreeDimBucketBasedList {

    using Bucket = std::vector<const state *>;

public:

    ThreeDimBucketBasedList() : table(10, stateHasher) {}

    ~ThreeDimBucketBasedList() {}

    void Reset() {
        table.clear();
        fLayers.clear();
    }

    void AddOpenNode(const state val, double g, double h, double d, const state *parent = nullptr);

    std::pair<const state *, double> Pop(MinCriterion criterion);

    inline const dataStructure &Lookup(const state &objKey) const { return table.at(objKey); }

    /**
      * get the g value of a node or DBL_MAX if it doesn't exist
      **/
    const std::pair<bool, std::pair<bool, double>> getNodeG(const state &objKey, const double h, MinCriterion criterion) {
        auto nodeIt = table.find(objKey);
        if (nodeIt != table.end()) {
            double g = nodeIt->second.g;
            bool optimalG = nodeIt->second.bucket_index == -1 || (g + h <= getMinF(criterion)); // optimal g if expanded or their f is minimal
            return std::make_pair(true, std::make_pair(optimalG, nodeIt->second.g));
        } else {
            return std::make_pair(false, std::make_pair(false, std::numeric_limits<double>::max()));
        }
    }

    double getMinG(MinCriterion criterion) {
        if (bestBucket == nullptr) {
            bool expandableNodes = findBestBucket(criterion);
            if (!expandableNodes)
                return DBL_MAX;
        }
        return minG;
    }

    double getMinF(MinCriterion criterion) {
        if (bestBucket == nullptr) {
            bool expandableNodes = findBestBucket(criterion);
            if (!expandableNodes)
                return DBL_MAX;
        }
        return minF;
    }

    double getMinD(MinCriterion criterion) {
        if (bestBucket == nullptr) {
            bool expandableNodes = findBestBucket(criterion);
            if (!expandableNodes)
                return DBL_MAX;
        }
        return minD;
    }

    inline void setEnvironment(environment *env_) { env = env_; }

    int expandableNodes(double f, double g, bool earlyStopping = false);

    bool findBestBucket(MinCriterion criterion);

    void setLimits(double gLim_, double fLim_, double dLim_) {
        invalidateCachedValues();
        gLim = gLim_, fLim = fLim_, dLim = dLim_;
    }

private:

    environment *env;

    std::function<size_t(const state &)> stateHasher = [this](const state &x) { return env->GetStateHash(x); };

    std::unordered_map<const state, dataStructure, decltype(stateHasher)> table;

    // fist key is g, second is f, third is d
    std::map<double, std::map<double, std::map < double, Bucket> >> fLayers;

    Bucket *bestBucket = nullptr;

    double minG = DBL_MAX, minF = DBL_MAX, minD = DBL_MAX;
    double gLim = DBL_MAX, fLim = DBL_MAX, dLim = DBL_MAX;

    void invalidateCachedValues() {
        bestBucket = nullptr;
        minG = DBL_MAX;
        minF = DBL_MAX;
        minD = DBL_MAX;
    }

};

template<typename state, class environment, class dataStructure>
void ThreeDimBucketBasedList<state, environment, dataStructure>::AddOpenNode(const state val,
                                                                             const double g,
                                                                             const double h,
                                                                             const double d,
                                                                             const state *parent) {
    const double f = g + h;

    auto nodeIt = table.find(val);
    if (nodeIt != table.end()) { // node already exists
        double old_g = nodeIt->second.g;
        if (old_g <= g) {
            return;    // existing node has no worse g value, don't store
        } else {
            // invalidate pointer with higher g value in the open list
            // remember to get the right d, which depends on old_g!
            fLayers[old_g][old_g + h][d + (old_g - g)][nodeIt->second.bucket_index] = nullptr;

            auto &bucket = fLayers[g][f][d];
            nodeIt->second = dataStructure(g, parent, bucket.size()); // node exists but with worse g value, update
            bucket.push_back(&(nodeIt->first));
            if (g < minG || f < minF || d < minD)
                invalidateCachedValues();
        }
    } else {  // node doesn't exist
        auto &bucket = fLayers[g][f][d];
        auto it_pair = table.insert(std::make_pair(val, dataStructure(g, parent, bucket.size())));
        bucket.push_back(&(it_pair.first->first));
        if (g < minG || f < minF || d < minD)
            invalidateCachedValues();
    }

}


template<typename state, class environment, class dataStructure>
bool ThreeDimBucketBasedList<state, environment, dataStructure>::findBestBucket(MinCriterion criterion) {

    invalidateCachedValues();

    auto gLayerIt = fLayers.begin();
    while (gLayerIt != fLayers.end() && gLayerIt->first <= gLim) {
        auto &gLayer = gLayerIt->second;

        if (gLayer.size() == 0) { // if the whole g layer is empty, erase it
            gLayerIt = fLayers.erase(gLayerIt);
            continue;
        }

        auto fLayerIt = gLayer.begin();
        while (fLayerIt != gLayer.end() && fLayerIt->first <= fLim) {
            auto &fLayer = fLayerIt->second;

            if (fLayer.size() == 0) { // if the whole f layer is empty, erase it
                fLayerIt = gLayer.erase(fLayerIt);
                continue;
            }

            auto dLayerIt = fLayer.begin();
            while (dLayerIt != fLayer.end() && dLayerIt->first <= dLim) {

                // deal with bucket - first, check that is not empty
                Bucket &bucket = dLayerIt->second;
                if (bucket.size() == 0) {
                    dLayerIt = fLayer.erase(dLayerIt);
                    continue;
                }

                // pick it if it is the best based on the criterion
                if (gLayerIt->first < minG) {
                    minG = gLayerIt->first;
                    if (criterion == MinCriterion::MinG) bestBucket = &bucket;
                }

                if (fLayerIt->first < minF) {
                    minF = fLayerIt->first;
                    if (criterion == MinCriterion::MinF) bestBucket = &bucket;
                }

                if (dLayerIt->first < minD) {
                    minD = dLayerIt->first;
                    if (criterion == MinCriterion::MinD) bestBucket = &bucket;
                }

                dLayerIt++;
            }

            fLayerIt++;
        }

        gLayerIt++;
    }

    return bestBucket != nullptr;
}

template<typename state, class environment, class dataStructure>
std::pair<const state *, double>
ThreeDimBucketBasedList<state, environment, dataStructure>::Pop(MinCriterion criterion) {
    const state *poppedState = nullptr;

    while (poppedState == nullptr) {
        if (bestBucket == nullptr) {
            bool expandableNodes = findBestBucket(criterion);
            if (!expandableNodes)
                return std::make_pair(nullptr, -1);
        }

        poppedState = bestBucket->back();
        bestBucket->pop_back();
        if (bestBucket->size() == 0)
            invalidateCachedValues(); // whenever a bucket is emptied, cached must be invalidated
    }

    auto &node = table.at(*poppedState);
    node.bucket_index = -1;
    return std::make_pair(poppedState, node.g);
}

/**
  * number of expandable nodes such that f(n) <= f and g(n) < g
  */
template<typename state, class environment, class dataStructure>
int ThreeDimBucketBasedList<state, environment, dataStructure>::expandableNodes(double f,
                                                                                double g,
                                                                                bool earlyStopping) {
    int nodeCount = 0;
    for (const auto &flayer : fLayers) {
        if (flayer.first > f)
            break;

        for (const auto &bucket : flayer.second) {
            if (bucket.first >= g)
                break;

            nodeCount += bucket.second.size();
            if (earlyStopping) {
                return nodeCount;
            }
        }

    }

    return nodeCount;

}

#endif