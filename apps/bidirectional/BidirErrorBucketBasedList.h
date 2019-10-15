#ifndef BIDIRERRORBUCKETBASEDLIST_H
#define BIDIRERRORBUCKETBASEDLIST_H

#include <cassert>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <stdint.h>
#include <limits>
#include <functional>
#include "BucketBasedList.h"

enum MinCriterion {
    MinG, MinF, MinD, MinB
};

struct NodeValues {
    std::set<double> g_values;
    std::set<double> f_values;
    std::set<double> d_values;
    std::set<double> b_values;
};

template<typename state, class environment, bool useB = true, class dataStructure = BucketNodeData<state> >
class BidirErrorBucketBasedList {

    using Bucket = std::vector<const state *>;

public:

    BidirErrorBucketBasedList() : table(10, stateHasher) {}

    ~BidirErrorBucketBasedList() {}

    void Reset() {
        table.clear();
        fLayers.clear();
    }

    void AddOpenNode(const state val, double g, double h, double d, const state *parent = nullptr);

    std::pair<const state *, double> Pop();

    inline const dataStructure &Lookup(const state &objKey) const { return table.at(objKey); }

    /**
      * get the g value of a node or DBL_MAX if it doesn't exist
      **/
    const std::pair<bool, std::pair<bool, double>> getNodeG(const state &objKey, const double h) {
        // TODO: this should get a function and not h, as h is seldom needed and computing it may be expensive
        auto nodeIt = table.find(objKey);
        if (nodeIt != table.end()) {
            double g = nodeIt->second.g;

            // optimal g if expanded or their g or their f is minimal
            // TODO: implement check for min d or even pareto optimality
            bool optimalG = nodeIt->second.bucket_index == -1 ||
                            (isBestBucketComputed() && (g <= getMinG() || (g + h <= getMinF())));
            return std::make_pair(true, std::make_pair(optimalG, nodeIt->second.g));
        } else {
            return std::make_pair(false, std::make_pair(false, std::numeric_limits<double>::max()));
        }
    }

    double getMinG() {
        if (bestBucket == nullptr) throw std::runtime_error("No cached best bucket");
        return minG;
    }

    double getMinF() {
        if (bestBucket == nullptr) throw std::runtime_error("No cached best bucket");
        return minF;
    }

    double getMinD() {
        if (bestBucket == nullptr) throw std::runtime_error("No cached best bucket");
        return minD;
    }

    double getMinB() {
        if (bestBucket == nullptr) throw std::runtime_error("No cached best bucket");
        return minB;
    }

    inline void setEnvironment(environment *env_) { env = env_; }

    int countMinimumGNodes();

    NodeValues getNodeValues();

    bool isBestBucketComputed() { return bestBucket != nullptr; }

    void computeBestBucket(MinCriterion criterion, double gLim_, double fLim_, double dLim_, double bLim_);

private:

    environment *env;

    std::function<size_t(const state &)> stateHasher = [this](const state &x) { return env->GetStateHash(x); };

    std::unordered_map<const state, dataStructure, decltype(stateHasher)> table;

    // fist key is g, second is f, third is d
    std::map<double, std::map<double, std::map < double, Bucket> >> fLayers;

    Bucket *bestBucket = nullptr;

    double minG = DBL_MAX, minF = DBL_MAX, minD = DBL_MAX, minB = DBL_MAX;
    double gLim = DBL_MAX, fLim = DBL_MAX, dLim = DBL_MAX, bLim = DBL_MAX;

    void invalidateCachedValues() {
        bestBucket = nullptr;
        minG = DBL_MAX;
        minF = DBL_MAX;
        minD = DBL_MAX;
        minB = DBL_MAX;
    }

};

template<typename state, class environment, bool useB, class dataStructure>
void BidirErrorBucketBasedList<state, environment, useB, dataStructure>::AddOpenNode(const state val,
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
            if (g < minG || f < minF || d < minD || (useB && (f + d) < minB))
                invalidateCachedValues();
        }
    } else {  // node doesn't exist
        auto &bucket = fLayers[g][f][d];
        auto it_pair = table.insert(std::make_pair(val, dataStructure(g, parent, bucket.size())));
        bucket.push_back(&(it_pair.first->first));
        if (g < minG || f < minF || d < minD || (useB && (f + d) < minB))
            invalidateCachedValues();
    }

}

template<typename state, class environment, bool useB, class dataStructure>
void BidirErrorBucketBasedList<state, environment, useB, dataStructure>::computeBestBucket(MinCriterion criterion,
                                                                             double gLim_,
                                                                             double fLim_,
                                                                             double dLim_,
                                                                             double bLim_) {

    invalidateCachedValues();
    gLim = gLim_, fLim = fLim_, dLim = dLim_, bLim = bLim_;

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

                // check its b value against bLim
                double bValue = fLayerIt->first + dLayerIt->first;
                if (useB && bValue > bLim) {
                    dLayerIt++;
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

                if (useB && bValue < minB) {
                    minB = bValue;
                    if (criterion == MinCriterion::MinB) bestBucket = &bucket;
                }

                dLayerIt++;
            }
            fLayerIt++;
        }
        gLayerIt++;
    }
}

template<typename state, class environment, bool useB, class dataStructure>
std::pair<const state *, double>
BidirErrorBucketBasedList<state, environment, useB, dataStructure>::Pop() {
    const state *poppedState = nullptr;

    while (poppedState == nullptr) {
        if (!isBestBucketComputed()) {
            return std::make_pair(nullptr, -1); // only pop when a proper bucket is known based on some limits
        }

        poppedState = bestBucket->back();
        bestBucket->pop_back();
        if (bestBucket->size() == 0)
            invalidateCachedValues(); // whenever a bucket is emptied, the cache must be invalidated
    }

    auto &node = table.at(*poppedState);
    node.bucket_index = -1;
    return std::make_pair(poppedState, node.g);
}

template<typename state, class environment, bool useB, class dataStructure>
int BidirErrorBucketBasedList<state, environment, useB, dataStructure>::countMinimumGNodes() {

    if (fLayers.empty()) return 0;

    double minExpandableG = DBL_MAX;
    int nodeCount = 0;
    for (const auto &glayer : fLayers) {
        if (glayer.first > gLim)
            break;

        for (const auto &fLayer : glayer.second) {
            if (fLayer.first > fLim)
                break;

            for (const auto &bucket : fLayer.second) {
                if (bucket.first > dLim || (useB && fLayer.first + bucket.first > bLim))
                    break;

                if (glayer.first < minExpandableG) { // we found a lower g bucket with expandable nodes
                    minExpandableG = glayer.first;
                    nodeCount = 0;
                }
                nodeCount += bucket.second.size();
            }
        }
    }

    return minExpandableG == DBL_MAX ? 0 : nodeCount;

}

template<typename state, class environment, bool useB, class dataStructure>
NodeValues BidirErrorBucketBasedList<state, environment, useB, dataStructure>::getNodeValues() {

    NodeValues result;

    for (const auto &glayer : fLayers) {
        result.g_values.insert(glayer.first);

        for (const auto &fLayer : glayer.second) {
            result.f_values.insert(fLayer.first);

            for (const auto &bucket : fLayer.second) {
                result.d_values.insert(bucket.first);
                result.b_values.insert(fLayer.first + bucket.first);
            }
        }
    }

    return result;
}

#endif