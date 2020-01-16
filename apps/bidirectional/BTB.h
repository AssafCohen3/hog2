#ifndef BTB_H
#define BTB_H

#include "BidirErrorBucketBasedList.h"
#include "FPUtil.h"
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <math.h>
#include <utility>
#include <vector>
#include <queue>

struct PairsInfo {

    PairsInfo(bool forward_, BucketInfo mostConnected_,
              std::pair <BucketInfo, BucketInfo> smallestPair_,
              std::vector <std::pair<bool, BucketInfo>> vertexCover_) :
            forward(forward_), mostConnected(mostConnected_), smallestPair(smallestPair_), vertexCover(vertexCover_) {}

    bool forward;
    BucketInfo mostConnected;
    std::pair <BucketInfo, BucketInfo> smallestPair;
    std::vector <std::pair<bool, BucketInfo>> vertexCover;
};

struct MustExpandEdge {

    MustExpandEdge(BucketInfo fwBucket_, BucketInfo bwBucket_) : fwBucket(fwBucket_), bwBucket(bwBucket_) {}

    BucketInfo fwBucket;
    BucketInfo bwBucket;
};

struct EdgeCompare {
    // comparator is actually "greater than" so the heap is a min-heap
    bool operator()(const MustExpandEdge &e1, const MustExpandEdge &e2) {
        int e1Min = std::min(e1.fwBucket.nodes, e1.bwBucket.nodes);
        int e2Min = std::min(e2.fwBucket.nodes, e2.bwBucket.nodes);

        if (e1Min == e2Min)
            return std::max(e1.fwBucket.nodes, e1.bwBucket.nodes) > std::max(e2.fwBucket.nodes, e2.bwBucket.nodes);

        return e1Min > e2Min;

    }
};

enum BTBPolicy {
    Alternating, Smallest, MostConnected, VertexCover
};

template<class state, class action, class environment, bool useRC = true, class priorityQueue = BidirErrorBucketBasedList<state, environment, true, useRC, BucketNodeData<state>>>
class BTB {
public:
    BTB(BTBPolicy policy_, double epsilon_ = 1.0) {
        forwardHeuristic = 0;
        backwardHeuristic = 0;
        env = 0;
        ResetNodeCount();
        nodesExpanded = nodesTouched = 0;
        currentCost = DBL_MAX;
        epsilon = epsilon_;
        policy = policy_;
    }

    ~BTB() {
        forwardQueue.Reset();
        backwardQueue.Reset();
    }

    void GetPath(environment *env, const state &from, const state &to,
                 Heuristic <state> *forward, Heuristic <state> *backward, std::vector <state> &thePath);

    bool InitializeSearch(environment *env, const state &from, const state &to, Heuristic <state> *forward,
                          Heuristic <state> *backward, std::vector <state> &thePath);

    bool CheckSolution(std::vector <state> &thePath) { return fgreatereq(C, currentCost); }

    void ExpandBucket(bool forward, const BucketInfo &info);

    virtual const char *GetName() { return "BTB"; }

    void ResetNodeCount() {
        nodesExpanded = nodesTouched = 0;
        counts.clear();
    }

    inline const int GetNumForwardItems() { return forwardQueue.size(); }

    inline const BucketNodeData<state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }

    inline const int GetNumBackwardItems() { return backwardQueue.size(); }

    inline const BucketNodeData<state> &GetBackwardItem(unsigned int which) { return backwardQueue.Lookat(which); }

    uint64_t GetUniqueNodesExpanded() const { return nodesExpanded; }

    uint64_t GetNodesExpanded() const { return nodesExpanded; }

    uint64_t GetNodesTouched() const { return nodesTouched; }

    uint64_t GetNecessaryExpansions() {
        uint64_t necessary = 0;
        for (const auto &count : counts) {
            if (count.first < currentCost)
                necessary += count.second;
        }
        return necessary;
    }

    void Reset() {
        currentCost = DBL_MAX;
        forwardQueue.Reset();
        backwardQueue.Reset();
        ResetNodeCount();
    }

private:

    void ExtractPath(const priorityQueue &queue, state &collisionState, std::vector <state> &thePath) {
        thePath.push_back(collisionState);
        auto parent = queue.Lookup(collisionState).parent;
        while (parent != nullptr) {
            thePath.push_back(*parent);
            parent = queue.Lookup(*parent).parent;
        }
    }

    bool Expand(const state *currentState, double g,
                priorityQueue &current, priorityQueue &opposite,
                Heuristic <state> *heuristic, Heuristic <state> *reverseHeuristic,
                const state &target, const state &source);

    std::pair<bool, PairsInfo> ComputePairs(bool computeVertexCover = false);

    priorityQueue forwardQueue, backwardQueue;
    state goal, start;

    uint64_t nodesTouched, nodesExpanded;

    std::map<double, int> counts;

    state middleNode;
    double currentCost;
    double epsilon;

    environment *env;
    Heuristic <state> *forwardHeuristic;
    Heuristic <state> *backwardHeuristic;

    BTBPolicy policy;

    double C = 0.0;

};

template<class state, class action, class environment, bool useRC, class priorityQueue>
void BTB<state, action, environment, useRC, priorityQueue>::GetPath(environment *env,
                                                                    const state &from,
                                                                    const state &to,
                                                                    Heuristic <state> *forward,
                                                                    Heuristic <state> *backward,
                                                                    std::vector <state> &thePath) {
    if (!InitializeSearch(env, from, to, forward, backward, thePath))
        return;

    while (!forwardQueue.IsEmpty() && !backwardQueue.IsEmpty()) {
        auto pairComputation = ComputePairs(policy == BTBPolicy::VertexCover);
        if (pairComputation.first) {
            // check solution after increasing C
            if (CheckSolution(thePath)) break;

            // TODO think how we are going to parametrize the tie breaker
        }

        const PairsInfo &pairsInfo = pairComputation.second;

        if (policy == BTBPolicy::Alternating) { // alternate directions
            const BucketInfo &fwInfo = pairsInfo.smallestPair.first;
            const BucketInfo &bwInfo = pairsInfo.smallestPair.second;

            while (!forwardQueue.RemoveIfEmpty(fwInfo.g, fwInfo.h, fwInfo.h_nx) &&
                   !backwardQueue.RemoveIfEmpty(bwInfo.g, bwInfo.h, bwInfo.h_nx)) {
                auto fwPop = forwardQueue.PopBucket(fwInfo.g, fwInfo.h, fwInfo.h_nx);
                Expand(fwPop, fwInfo.g, forwardQueue, backwardQueue,
                       forwardHeuristic, backwardHeuristic, goal, start);

                auto bwPop = backwardQueue.PopBucket(bwInfo.g, bwInfo.h, bwInfo.h_nx);
                Expand(bwPop, bwInfo.g, backwardQueue, forwardQueue,
                       backwardHeuristic, forwardHeuristic, start, goal);
            }

        } else if (policy == BTBPolicy::MostConnected) { // expand the bucket with most connected nodes
            ExpandBucket(pairsInfo.forward, pairsInfo.mostConnected);
        } else if (policy == BTBPolicy::Smallest) { // expand the smallest bucket of the smallest pair
            bool forward = pairsInfo.smallestPair.first.nodes < pairsInfo.smallestPair.second.nodes;
            ExpandBucket(forward, forward ? pairsInfo.smallestPair.first : pairsInfo.smallestPair.second);
        } else if (policy == BTBPolicy::VertexCover) { // expand the bucket with most connected nodes
            auto vertexCover = pairsInfo.vertexCover;
            while (!vertexCover.empty()) {
                auto vertex = vertexCover.back();
                vertexCover.pop_back();
                ExpandBucket(vertex.first, vertex.second);
            }
        } else { exit(0); }

        if (CheckSolution(thePath)) break;

    }

    // reconstruct the solution path
    std::vector <state> pFor, pBack;
    ExtractPath(backwardQueue, middleNode, pBack);
    ExtractPath(forwardQueue, middleNode, pFor);
    reverse(pFor.begin(), pFor.end());
    thePath = pFor;
    thePath.insert(thePath.end(), pBack.begin() + 1, pBack.end());
}

template<class state, class action, class environment, bool useRC, class priorityQueue>
void BTB<state, action, environment, useRC, priorityQueue>::ExpandBucket(bool forward, const BucketInfo &info) {
    if (forward) {
        while (!forwardQueue.RemoveIfEmpty(info.g, info.h, info.h_nx)) {
            auto pop = forwardQueue.PopBucket(info.g, info.h, info.h_nx);
            Expand(pop, info.g, forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
        }
    } else {
        while (!backwardQueue.RemoveIfEmpty(info.g, info.h, info.h_nx)) {
            auto pop = backwardQueue.PopBucket(info.g, info.h, info.h_nx);
            Expand(pop, info.g, backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
        }
    }
}

template<class state, class action, class environment, bool useRC, class priorityQueue>
bool BTB<state, action, environment, useRC, priorityQueue>::InitializeSearch(environment *env, const state &from,
                                                                             const state &to,
                                                                             Heuristic <state> *forward,
                                                                             Heuristic <state> *backward,
                                                                             std::vector <state> &thePath) {
    this->env = env;
    forwardHeuristic = forward;
    backwardHeuristic = backward;
    Reset();
    start = from;
    goal = to;
    if (start == goal)
        return false;

    double forwardH = std::max(forwardHeuristic->HCost(start, goal), epsilon);
    double backwardH = std::max(backwardHeuristic->HCost(goal, start), epsilon);

    forwardQueue.setEnvironment(env);
    forwardQueue.AddOpenNode(start, 0, forwardH, 0);
    backwardQueue.setEnvironment(env);
    backwardQueue.AddOpenNode(goal, 0, backwardH, 0);

    C = std::max({forwardH, backwardH, epsilon});

    return true;
}

template<class state, class action, class environment, bool useRC, class priorityQueue>
std::pair<bool, PairsInfo>
BTB<state, action, environment, useRC, priorityQueue>::ComputePairs(bool computeVertexCover) {

    auto fwBuckets = forwardQueue.getBucketInfo();
    auto bwBuckets = backwardQueue.getBucketInfo();

    double btbC = DBL_MAX;

    // buckets that can be paired with the most nodes
    std::unordered_map<const BucketInfo, int, BucketHash> fwMostConnectedBuckets;
    std::unordered_map<const BucketInfo, int, BucketHash> bwMostConnectedBuckets;

    // arbitrary lower-bound bucket pair
    std::pair <BucketInfo, BucketInfo> smallestBucketPair;

    // TODO split different techniques into different methods, too much code in here
    std::priority_queue<MustExpandEdge, std::vector<MustExpandEdge>, EdgeCompare> edgeHeap;

    for (const auto &fwInfo: fwBuckets) {
        for (const auto &bwInfo: bwBuckets) {
            double fwKK = fwInfo.h - bwInfo.h_nx;
            double bwKK = bwInfo.h - fwInfo.h_nx;

            double distance = std::max({fwKK, bwKK, epsilon});

            if (useRC) {
                double fwRC = bwInfo.h_nx - fwInfo.h;
                double bwRC = fwInfo.h_nx - bwInfo.h;
                distance = std::max({fwRC, bwRC, distance});
            }

            double bound = fwInfo.g + bwInfo.g + distance;

            if (bound < btbC) { // lower the bound, choose the pair and reset the pairing info
                btbC = bound;
                fwMostConnectedBuckets.clear();
                bwMostConnectedBuckets.clear();
                // heaps have no clear method, so reassign it
                edgeHeap = std::priority_queue<MustExpandEdge, std::vector<MustExpandEdge>, EdgeCompare>();
                smallestBucketPair = std::make_pair(BucketInfo(), BucketInfo());
            }

            if (bound == btbC) {

                fwMostConnectedBuckets[fwInfo] += bwInfo.nodes;
                bwMostConnectedBuckets[bwInfo] += fwInfo.nodes;

                // select  the pair containing the smallest bucket and, among those, the one with overall fewer states
                int num_nodes = std::min(smallestBucketPair.first.nodes, smallestBucketPair.second.nodes);
                if (fwInfo.nodes < num_nodes || bwInfo.nodes < num_nodes) {
                    smallestBucketPair = std::make_pair(fwInfo, bwInfo);
                } else if (fwInfo.nodes == num_nodes && bwInfo.nodes < smallestBucketPair.second.nodes) {
                    smallestBucketPair.second = bwInfo;
                } else if (bwInfo.nodes == num_nodes && fwInfo.nodes < smallestBucketPair.first.nodes) {
                    smallestBucketPair.first = fwInfo;
                }

                if (computeVertexCover) edgeHeap.push(MustExpandEdge(fwInfo, bwInfo));
            }
        }
    }

    auto conn_comp = [](const std::pair<BucketInfo, int> &p1, const std::pair<BucketInfo, int> &p2) {
        return p1.second < p2.second;
    };

    auto fwMostConnected = std::max_element(fwMostConnectedBuckets.begin(), fwMostConnectedBuckets.end(), conn_comp);
    auto bwMostConnected = std::max_element(bwMostConnectedBuckets.begin(), bwMostConnectedBuckets.end(), conn_comp);

    bool forward = fwMostConnected->second >= bwMostConnected->second;
    auto mostConnected = forward ? fwMostConnected->first : bwMostConnected->first;

    std::vector <std::pair<bool, BucketInfo>> vertexCover;
    std::unordered_set <BucketInfo, BucketHash> seenBuckets;
    if (computeVertexCover) {
        while (!edgeHeap.empty()) {
            MustExpandEdge edge = edgeHeap.top();
            edgeHeap.pop();
            if (seenBuckets.find(edge.fwBucket) != seenBuckets.end() ||
                seenBuckets.find(edge.bwBucket) != seenBuckets.end())
                continue;

            bool fw = edge.fwBucket.nodes <= edge.bwBucket.nodes;
            BucketInfo bucket = fw ? edge.fwBucket : edge.bwBucket;
            vertexCover.push_back(std::make_pair(fw, bucket));
        }
    }

    bool updatedC = btbC > C;
    C = btbC;

    return std::make_pair(updatedC, PairsInfo(forward, mostConnected, smallestBucketPair, vertexCover));
}

template<class state, class action, class environment, bool useRC, class priorityQueue>
bool BTB<state, action, environment, useRC, priorityQueue>::Expand(const state *currentState,
                                                                   double g,
                                                                   priorityQueue &current,
                                                                   priorityQueue &opposite,
                                                                   Heuristic <state> *heuristic,
                                                                   Heuristic <state> *reverseHeuristic,
                                                                   const state &target, const state &source) {
    nodesExpanded++;
    counts[C] += 1;

    std::vector <state> neighbors;
    env->GetSuccessors(*currentState, neighbors);

    for (auto &succ : neighbors) {

        nodesTouched++;

        double succG = g + env->GCost(*currentState, succ);

        double h = std::max(heuristic->HCost(succ, target), epsilon);

        // ignore states with greater cost than best solution
        // this can be either g + h
        if (fgreatereq(succG + h, currentCost))
            continue;

        double h_nx = reverseHeuristic->HCost(succ, source);

        // check if there is a collision
        auto collision = opposite.getNodeG(succ, h_nx);
        if (collision.first) {
            auto gValue = collision.second;
            double collisionCost = succG + gValue.second;
            if (fless(collisionCost, currentCost)) {
                currentCost = collisionCost;
                middleNode = succ;

                if (fgreatereq(C, currentCost)) {
                    // add the node so the plan can be extracted
                    current.AddOpenNode(succ, succG, h, h_nx, currentState);
                    break; // step out, don't generate more nodes
                }
            } else if (gValue.first) {
                continue; // if the g value is provably optimal and the collision value is geq, prune the node
            }
        }

        // add it to the open list
        current.AddOpenNode(succ, succG, h, h_nx, currentState);
    }

    return true;
}

#endif
