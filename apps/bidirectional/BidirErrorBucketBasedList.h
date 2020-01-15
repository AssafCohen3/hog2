#ifndef BIDIRERRORBUCKETBASEDLIST_H
#define BIDIRERRORBUCKETBASEDLIST_H

#include <cassert>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <stdint.h>
#include <limits>
#include <climits>
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
    std::set<double> rf_values;
    std::set<double> rd_values;
};

struct BucketInfo {

    BucketInfo() : g(DBL_MAX), h(DBL_MAX), h_nx(DBL_MAX), nodes(INT_MAX) {}

    BucketInfo(double g_, double h_, double h_nx_, int nodes_) : g(g_), h(h_), h_nx(h_nx_), nodes(nodes_) {}

    bool operator==(const BucketInfo &info) const { return g == info.g && h == info.h && h_nx == info.h_nx; }

    double g, h, h_nx;
    int nodes; // the nodes attribute doesn't count for equals nor hash
};

struct BucketHash {

    std::size_t operator()(const BucketInfo &info) const {
        std::size_t hash = 0;
        hash = hash_combine(hash, info.g);
        hash = hash_combine(hash, info.h);
        hash = hash_combine(hash, info.h_nx);
        return hash;
    }

    inline std::size_t hash_combine(std::size_t &seed, const double value) const {
        return std::hash < double > {}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
};

template<typename state, class environment, bool useB = true, bool useRC = true, class dataStructure = BucketNodeData<state> >
class BidirErrorBucketBasedList {

    using Bucket = std::vector<const state *>;

public:

    BidirErrorBucketBasedList() : table(10, stateHasher) {}

    ~BidirErrorBucketBasedList() {}

    inline void Reset() {
        table.clear();
        fLayers.clear();
    }

    void AddOpenNode(const state val, double g, double h, double h_nx, const state *parent = nullptr);

    std::pair<const state *, double> Pop();

    bool IsEmpty() { return fLayers.size() == 0; }

    bool RemoveIfEmpty(double g, double h, double h_nx);

    const state *PopBucket(double g, double h, double h_nx);

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

    inline double getMinG() { return checkBestBucketAndReturnValue(minG); }

    inline double getMinF() { return checkBestBucketAndReturnValue(minF); }

    inline double getMinD() { return checkBestBucketAndReturnValue(minD); }

    inline double getMinB() { return checkBestBucketAndReturnValue(minB); }

    inline double getMinRF() { return checkBestBucketAndReturnValue(minRF); }

    inline double getMinRD() { return checkBestBucketAndReturnValue(minRD); }

    inline void setEnvironment(environment *env_) { env = env_; }

    std::vector <BucketInfo> getBucketInfo();

    int countMinimumGNodes();

    NodeValues getNodeValues();

    inline bool isBestBucketComputed() { return bestBucket != nullptr; }

    void computeBestBucket(MinCriterion criterion,
                           double gLim_, double fLim_, double dLim_,
                           double bLim_, double rfLim_, double rdLim_);

private:

    environment *env;

    std::function<size_t(const state &)> stateHasher = [this](const state &x) { return env->GetStateHash(x); };

    std::unordered_map<const state, dataStructure, decltype(stateHasher)> table;

    // fist key is g, second is h, third is h_nx (h_nx is sorted in reverse to traverse by ascending d)
    std::map<double, std::map<double, std::map < double, Bucket, std::greater < double>> >>
    fLayers;

    Bucket *bestBucket = nullptr;

    double minG = DBL_MAX, minF = DBL_MAX, minD = DBL_MAX, minB = DBL_MAX, minRF = DBL_MAX, minRD = DBL_MAX;
    double gLim = DBL_MAX, fLim = DBL_MAX, dLim = DBL_MAX, bLim = DBL_MAX, rfLim = DBL_MAX, rdLim = DBL_MAX;

    inline double checkBestBucketAndReturnValue(double value) {
        if (bestBucket == nullptr) throw std::runtime_error("No cached best bucket");
        return value;
    }

    inline void invalidateCachedValues() {
        bestBucket = nullptr;
        minG = DBL_MAX;
        minF = DBL_MAX;
        minD = DBL_MAX;
        minB = DBL_MAX;
        minRF = DBL_MAX;
        minRD = DBL_MAX;
    }

};

template<typename state, class environment, bool useB, bool useRC, class dataStructure>
void BidirErrorBucketBasedList<state, environment, useB, useRC, dataStructure>::AddOpenNode(const state val,
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
            auto bucketIndex = nodeIt->second.bucket_index;
            if (bucketIndex == -1) {
                std::cout << "  -- Node reopened!!!" << std::endl;
                exit(0);
            }

            // invalidate pointer with higher g value in the open list
            fLayers[old_g][h][h_nx][bucketIndex] = nullptr;

            auto &bucket = fLayers[g][h][h_nx];
            nodeIt->second = dataStructure(g, parent, bucket.size()); // node exists but with worse g value, update
            bucket.push_back(&(nodeIt->first));
            if (g < minG || f < minF || d < minD
                || (useB && (f + d) < minB)
                || (useRC && (g - h) < minRF) || (useRC && (g + h_nx) < minRD))
                invalidateCachedValues();
        }
    } else {  // node doesn't exist
        auto &bucket = fLayers[g][h][h_nx];
        auto it_pair = table.insert(std::make_pair(val, dataStructure(g, parent, bucket.size())));
        bucket.push_back(&(it_pair.first->first));
        if (g < minG || f < minF || d < minD
            || (useB && (f + d) < minB)
            || (useRC && (g - h) < minRF) || (useRC && (g + h_nx) < minRD))
            invalidateCachedValues();
    }

}

template<typename state, class environment, bool useB, bool useRC, class dataStructure>
void
BidirErrorBucketBasedList<state, environment, useB, useRC, dataStructure>::computeBestBucket(MinCriterion criterion,
                                                                                             double gLim_,
                                                                                             double fLim_,
                                                                                             double dLim_,
                                                                                             double bLim_,
                                                                                             double rfLim_,
                                                                                             double rdLim_) {
    invalidateCachedValues();
    gLim = gLim_, fLim = fLim_, dLim = dLim_, bLim = bLim_, rfLim = rfLim_, rdLim = rdLim_;

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

            // check its rf value against rfLim
            double rfValue = g - h;
            if (useRC && rfValue > rfLim) {
                fLayerIt++;
                continue; // continue because rf is decreasing
            }

            auto &fLayer = fLayerIt->second;

            if (fLayer.size() == 0) { // if the whole f layer is empty, erase it
                fLayerIt = gLayer.erase(fLayerIt);
                continue;
            }

            auto dLayerIt = fLayer.begin();
            while (dLayerIt != fLayer.end()) {
                double h_nx = dLayerIt->first;
                double d = g - h_nx;

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
                    break;
                }

                // check its rd value against rdLim
                double rdValue = g + h_nx;
                if (useRC && rdValue > rdLim) {
                    dLayerIt++;
                    continue; // continue because rd is decreasing
                }

                // pick the bucket as best bucket if it is the best based on the criterion
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

                if (useRC && rfValue < minRF) {
                    minRF = rfValue;
                }

                if (useRC && rdValue < minRD) {
                    minRD = rdValue;
                }

                dLayerIt++;
            }
            fLayerIt++;
        }
        gLayerIt++;
    }
}

template<typename state, class environment, bool useB, bool useRC, class dataStructure>
std::pair<const state *, double>
BidirErrorBucketBasedList<state, environment, useB, useRC, dataStructure>::Pop() {
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


template<typename state, class environment, bool useB, bool useRC, class dataStructure>
bool BidirErrorBucketBasedList<state, environment, useB, useRC, dataStructure>::RemoveIfEmpty(double g,
                                                                                              double h,
                                                                                              double h_nx) {
    Bucket &bucket = fLayers[g][h][h_nx];

    // remove erased entries to make sure that the bucket does not contain only invalid entries
    while (bucket.size() > 0 && bucket.back() == nullptr) {
        bucket.pop_back();
    }

    bool bucketEmptied = false;

    // delete empty dimensions
    if (bucket.size() == 0) {
        bucketEmptied = true;
        auto &fLayer = fLayers[g][h];
        fLayer.erase(h_nx);
        if (fLayer.size() == 0) {
            auto &gLayer = fLayers[g];
            gLayer.erase(h);
            if (gLayer.size() == 0) {
                fLayers.erase(g);
            }
        }
    }

    return bucketEmptied;
}

template<typename state, class environment, bool useB, bool useRC, class dataStructure>
const state *BidirErrorBucketBasedList<state, environment, useB, useRC, dataStructure>::PopBucket(double g,
                                                                                                  double h,
                                                                                                  double h_nx) {
    // pop state - it has to be a proper bucket, so call RemoveIfInvalid if needed
    Bucket &bucket = fLayers[g][h][h_nx];
    const state *poppedState = bucket.back();
    bucket.pop_back();
    RemoveIfEmpty(g, h, h_nx); // remove if it is empty

    // invalidate node 2 bucket index
    auto &node = table.at(*poppedState);
    node.bucket_index = -1;

    return poppedState;
}

template<typename state, class environment, bool useB, bool useRC, class dataStructure>
std::vector <BucketInfo> BidirErrorBucketBasedList<state, environment, useB, useRC, dataStructure>::getBucketInfo() {

    std::vector <BucketInfo> result;

    for (const auto &glayer : fLayers) {
        double g = glayer.first;
        for (const auto &fLayer : glayer.second) {
            double h = fLayer.first;
            for (const auto &bucket : fLayer.second) {
                double h_nx = bucket.first;
                result.push_back(BucketInfo(g, h, h_nx, bucket.second.size()));

                if (!useRC) break; // subsequent buckets will be dominated if RC is not used
            }
        }
    }

    return result;

}

template<typename state, class environment, bool useB, bool useRC, class dataStructure>
int BidirErrorBucketBasedList<state, environment, useB, useRC, dataStructure>::countMinimumGNodes() {

    if (fLayers.empty()) return 0;

    double minExpandableG = DBL_MAX;
    int nodeCount = 0;
    for (const auto &glayer : fLayers) {
        double g = glayer.first;

        if (g > gLim) break;

        for (const auto &fLayer : glayer.second) {
            double h = fLayer.first;
            double f = g + h;
            double rf = g - h;

            if (f > fLim) break;

            if (useRC && rf > rfLim) continue; // continue because rf is decreasing

            for (const auto &bucket : fLayer.second) {
                double h_nx = bucket.first;
                double d = g - h_nx;
                double rd = g + h_nx;
                double b = f + d;

                if (d > dLim || (useB && b > bLim)) break;

                if (useRC && rd > rdLim) continue; // continue because rd is decreasing

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

template<typename state, class environment, bool useB, bool useRC, class dataStructure>
NodeValues BidirErrorBucketBasedList<state, environment, useB, useRC, dataStructure>::getNodeValues() {

    NodeValues result;

    for (const auto &glayer : fLayers) {
        double g = glayer.first;
        result.g_values.insert(g);

        for (const auto &fLayer : glayer.second) {
            double h = fLayer.first;
            double f = g + h;
            double rf = g - h;
            result.f_values.insert(f);
            result.rf_values.insert(rf);

            for (const auto &bucket : fLayer.second) {
                double h_nx = bucket.first;
                double d = g - h_nx;
                double rd = g + h_nx;
                double b = f + d;
                result.d_values.insert(d);
                result.rd_values.insert(rd);
                result.b_values.insert(b);
            }
        }
    }

    return result;
}

#endif