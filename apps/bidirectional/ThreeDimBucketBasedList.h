#ifndef BUCKETBASEDLIST_H
#define BUCKETBASEDLIST_H

#include <cassert>
#include <vector>
#include <map>
#include <unordered_map>
#include <stdint.h>
#include <limits>
#include <functional>
#include "BucketBasedList.h"

template<typename state, class environment, class dataStructure = BucketNodeData <state> >
class ThreeDimBucketBasedList {

public:

    ThreeDimBucketBasedList() : table(10, stateHasher) {}

    ~ThreeDimBucketBasedList() {}

    void Reset() {
        table.clear();
        fLayers.clear();
    }

    virtual void AddOpenNode(const state val, double g, double h, double d, const state *parent = nullptr);

    virtual std::pair<const state *, double> Pop(double fLim = DBL_MAX, double gLim = DBL_MAX);

    inline const dataStructure &Lookup(const state &objKey) const { return table.at(objKey); }

    /**
      * get the g value of a node or DBL_MAX if it doesn't exist
      **/
    const std::pair<bool, double> getNodeG(const state &objKey) {
        auto nodeIt = table.find(objKey);
        if (nodeIt != table.end())
            return std::make_pair(true, nodeIt->second.g);
        else
            return std::make_pair(false, DBL_MAX);
    }

    double getMinF(double lowerBound = -1.0);

    inline void setEnvironment(environment *env_) { env = env_; }

    int expandableNodes(double f, double g, bool earlyStopping = false);

private:
    environment *env;

    std::function<size_t(const state &)> stateHasher = [this](const state &x) { return env->GetStateHash(x); };

protected:
    std::unordered_map<const state, dataStructure, decltype(stateHasher)> table;

    // sorted buckets, although if the same f layer and/or bucket is repeatedly accessed then other implementations may be faster
    // fist key is f, second is g
    // it could be the other way around, profiling is probably necessary to determine performance
    std::map<double, std::map<double, std::map < double, std::vector<const state *> > >> fLayers;

};

template<typename state, class environment, class dataStructure>
void ThreeDimBucketBasedList<state, environment, dataStructure>::AddOpenNode(const state val, double g, double h,
                                                                             double d, const state *parent) {

    auto nodeIt = table.find(val);
    if (nodeIt != table.end()) { // node already exists
        double old_g = nodeIt->second.g;
        if (old_g <= g) {
            return;    // existing node has no worse g value, don't store
        } else {
            // invalidate pointer with higher g value in the open list
            fLayers[old_g + h][old_g][nodeIt->second.bucket_index] = nullptr;

            auto &bucket = fLayers[g + h][g];
            nodeIt->second = dataStructure(g, parent, bucket.size()); // node exists but with worse g value, update
            bucket.push_back(&(nodeIt->first));
        }
    } else {  // node doesn't exist
        auto &layer = fLayers[g + h];
        auto &bucket = layer[g];
        auto it_pair = table.insert(std::make_pair(val, dataStructure(g, parent, bucket.size())));
        bucket.push_back(&(it_pair.first->first));
    }

}

// TODO: this method does f-ascending, g-descending, which is likely the best tie-breaker
// TODO: still, it would be good if this could be parametrized
template<typename state, class environment, class dataStructure>
std::pair<const state *, double>
ThreeDimBucketBasedList<state, environment, dataStructure>::Pop(double fLim, double gLim) {
    const state *poppedState = nullptr;

    auto currentLayerIt = fLayers.begin();

    do {
        auto &currentFLayer = currentLayerIt->second;
        auto bucket_it = currentFLayer.rbegin(); // start with the highest g bucket

        // get the position of a bucket with valid entries under the limits
        while (bucket_it != currentFLayer.rend() && bucket_it->first >= gLim) { // bucket strictly lower than g
            bucket_it++;
        }

        // find a bucket with valid entries
        while (bucket_it != currentFLayer.rend() && poppedState == nullptr) {
            auto &bucket = bucket_it->second;
            while (poppedState == nullptr && bucket.size() != 0) {
                poppedState = bucket.back();  // this may be an invalid entry, (already expanded state with a lower g)
                bucket.pop_back();
            }

            // delete bucket if empty
            if (bucket.size() == 0) {
                currentFLayer.erase(--bucket_it.base());
                if (bucket_it != currentFLayer.rend()) {
                    bucket_it++;
                }
            }
        }

        // if no adequate bucket has been found
        if (bucket_it == currentFLayer.rend()) {
            if (currentFLayer.begin() == currentFLayer.end()) { // last bucket in the layer was erased
                currentLayerIt = fLayers.erase(currentLayerIt);
                if (fLayers.size() == 0) {
                    break; // empty open list
                }
            } else {
                currentLayerIt++;
            }
        }


        // repeat until we find a valid entry or there's none within the limits
    } while (poppedState == nullptr && currentLayerIt != fLayers.end() && currentLayerIt->first <= fLim);

    if (poppedState == nullptr) {
        return std::make_pair(nullptr, -1); // no valid (expandable) nodes
    }


    auto &node = table.at(*poppedState);
    node.bucket_index = -1;
    return std::make_pair(poppedState, node.g);
}

/**
  * in order to get a proper estimate, this method has to find a non-expanded node
  */
template<typename state, class environment, class dataStructure>
double ThreeDimBucketBasedList<state, environment, dataStructure>::getMinF(double lowerBound) {

    while (!fLayers.empty()) {

        auto currentLayerIt = fLayers.begin();
        while (true) {
            if (currentLayerIt == fLayers.end()) // no f values above lower bound
                return DBL_MAX;
            if (currentLayerIt->first > lowerBound) // found f value above lower bound
                break;
            currentLayerIt == currentLayerIt++; // try next value
        }

        auto &currentFLayer = currentLayerIt->second;

        while (!currentFLayer.empty()) {
            auto &bucket = currentFLayer.begin()->second;

            while (!bucket.empty()) {
                if (bucket.back() != nullptr) // found a valid node
                    return currentLayerIt->first;
                else
                    bucket.pop_back(); // discard the node, as it has been expanded
            }

            currentFLayer.erase(currentFLayer.begin()); // bucket empty - get rid of it
        }

        fLayers.erase(currentLayerIt); // f layer empty - get rid of it
    }

    return DBL_MAX;
}

/**
  * number of expandable nodes such that f(n) <= f and g(n) < g
  */
template<typename state, class environment, class dataStructure>
int
ThreeDimBucketBasedList<state, environment, dataStructure>::expandableNodes(double f, double g, bool earlyStopping) {

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