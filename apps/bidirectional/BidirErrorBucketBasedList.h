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

    void AddOpenNode(const state val, double g, double h, double h_nx, const state *parent = nullptr);

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

    // fist key is g, second is h, third is h_nx (h_nx is sorted in reverse to traverse by ascending d)
    std::map<double, std::map<double, std::map < double, Bucket, std::greater<int>> >> fLayers;

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
                                                                               const double h_nx,
                                                                               const state *parent) {
    const double f = g + h;
    const double d = g - h_nx;

    auto nodeIt = table.find(val);
    if (nodeIt != table.end()) { // node already exists
        double old_g = nodeIt->second.g;
        if (old_g <= g) {
            return;    // existing node has no worse g value, don't store
        } else {
            // invalidate pointer with higher g value in the open list
            // remember to get the right d, which depends on old_g!
            fLayers[old_g][h][h_nx][nodeIt->second.bucket_index] = nullptr;

            auto &bucket = fLayers[g][h][h_nx];
            nodeIt->second = dataStructure(g, parent, bucket.size()); // node exists but with worse g value, update
            bucket.push_back(&(nodeIt->first));
            if (g < minG || f < minF || d < minD || (useB && (f + d) < minB))
                invalidateCachedValues();
        }
    } else {  // node doesn't exist
        auto &bucket = fLayers[g][h][h_nx];
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
    while (gLayerIt != fLayers.end()) {
        double g = gLayerIt->first;

        if (g > gLim) break;

        auto &gLayer = gLayerIt->second;

        if (gLayer.size() == 0) { // if the whole g layer is empty, erase it
            gLayerIt = fLayers.erase(gLayerIt);
            continue;
        }

        auto fLayerIt = gLayer.begin();
        while (fLayerIt != gLayer.end()) {
            double h = fLayerIt->first;
            double f = g + h;

            if (f > fLim) break;

            auto &fLayer = fLayerIt->second;

            if (fLayer.size() == 0) { // if the whole f layer is empty, erase it
                fLayerIt = gLayer.erase(fLayerIt);
                continue;
            }

            auto dLayerIt = fLayer.begin();
            while (dLayerIt != fLayer.end()) {
                double d = g - dLayerIt->first;

                // deal with bucket - first, check that is not empty
                Bucket &bucket = dLayerIt->second;
                if (bucket.size() == 0) {
                    dLayerIt = fLayer.erase(dLayerIt);
                    continue;
                }

                if (d > dLim) break;

                // check its b value against bLim
                double bValue = f + d;
                if (useB && bValue > bLim) {
                    dLayerIt++;
                    continue;
                }

                // pick it if it is the best based on the criterion
                if (g < minG) {
                    minG = g;
                    if (criterion == MinCriterion::MinG) bestBucket = &bucket;
                }

                if (f < minF) {
                    minF = f;
                    if (criterion == MinCriterion::MinF) bestBucket = &bucket;
                }

                if (d < minD) {
                    minD = d;
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
    for (const auto &gLayer : fLayers) {
        double g = gLayer.first;

        if (g > gLim)
            break;

        for (const auto &fLayer : gLayer.second) {
            double f = g + fLayer.first;

            if (f > fLim)
                break;

            for (const auto &bucket : fLayer.second) {
                double d = g - bucket.first;

                if (d > dLim || (useB && f + d > bLim))
                    break;

                if (g < minExpandableG) { // we found a lower g bucket with expandable nodes
                    minExpandableG = g;
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
        double g = glayer.first;
        result.g_values.insert(g);

        for (const auto &fLayer : glayer.second) {
            double f = g + fLayer.first;
            result.f_values.insert(f);

            for (const auto &bucket : fLayer.second) {
                double d = g - bucket.first;
                result.d_values.insert(d);
                result.b_values.insert(f + d);
            }
        }
    }

    return result;
}

#endif