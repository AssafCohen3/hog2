#ifndef DDBS_H
#define DDBS_H

#include "BidirErrorBucketBasedList.h"
#include "FPUtil.h"
#include <unordered_map>
#include <iostream>
#include <math.h>

template<class state, class action, class environment, class priorityQueue = BidirErrorBucketBasedList<state, environment, BucketNodeData<state>>>
class DDBS {
public:
    DDBS(bool alternating_ = true, double epsilon_ = 1.0, double gcd_ = 1.0) {
        forwardHeuristic = 0;
        backwardHeuristic = 0;
        env = 0;
        ResetNodeCount();
        expandForward = true;
        nodesExpanded = nodesTouched = 0;
        currentCost = DBL_MAX;
        epsilon = epsilon_;
        gcd = gcd_;
        alternating = alternating_;
    }

    ~DDBS() {
        forwardQueue.Reset();
        backwardQueue.Reset();
    }

    void GetPath(environment *env, const state &from, const state &to,
                 Heuristic <state> *forward, Heuristic <state> *backward, std::vector <state> &thePath);

    bool InitializeSearch(environment *env, const state &from, const state &to, Heuristic <state> *forward,
                          Heuristic <state> *backward, std::vector <state> &thePath);

    bool DoSingleSearchStep(std::vector <state> &thePath);

    virtual const char *GetName() { return "Baseline"; }

    void ResetNodeCount() {
        nodesExpanded = nodesTouched = 0;
        counts.clear();
    }

    inline const int GetNumForwardItems() { return forwardQueue.size(); }

    inline const BucketNodeData<state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }

    inline const int GetNumBackwardItems() { return backwardQueue.size(); }

    inline const BucketNodeData<state> &GetBackwardItem(unsigned int which) {
        return backwardQueue.Lookat(which);
    }

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

    enum RiseCriterion {
        RiseG, RiseForward, RiseBackward
    };

    void ExtractPath(const priorityQueue &queue, state &collisionState, std::vector <state> &thePath) {
        thePath.push_back(collisionState);
        auto parent = queue.Lookup(collisionState).parent;
        while (parent != nullptr) {
            thePath.push_back(*parent);
            parent = queue.Lookup(*parent).parent;
        }
    }

    void Expand(priorityQueue &current, priorityQueue &opposite,
                Heuristic <state> *heuristic, Heuristic <state> *reverseHeuristic,
                const state &target, const state &source, bool direction);

    bool UpdateC();

    void UpdateQueuesAndCriterion();

    MinBCriterion getMinBCriterion(bool forwardQueue) {
        switch (criterion) {
            case RiseG:
                return MinBCriterion::MinBG;
            case RiseForward:
                return forwardQueue ? MinBCriterion::MinBF : MinBCriterion::MinBD;
            case RiseBackward:
                return forwardQueue ? MinBCriterion::MinBD : MinBCriterion::MinBF;
        }
    }

    double computeBBound(double forwardB, double backwardB) {
        double unroundedLowerBound = (forwardB + forwardB) / 2;
        return ceil(unroundedLowerBound / gcd) * gcd; // round up to the next multiple of gcd
    }

    priorityQueue forwardQueue, backwardQueue;
    state goal, start;

    uint64_t nodesTouched, nodesExpanded;

    std::map<double, int> counts;

    state middleNode;
    double currentCost;
    double epsilon;
    double gcd;

    RiseCriterion criterion = RiseCriterion::RiseG;

    environment *env;
    Heuristic <state> *forwardHeuristic;
    Heuristic <state> *backwardHeuristic;

    bool expandForward = true;
    bool alternating;

    double C = 0.0;

    bool updateByF = true;
};

template<class state, class action, class environment, class priorityQueue>
void DDBS<state, action, environment, priorityQueue>::GetPath(environment *env, const state &from, const state &to,
                                                              Heuristic <state> *forward,
                                                              Heuristic <state> *backward,
                                                              std::vector <state> &thePath) {
    if (!InitializeSearch(env, from, to, forward, backward, thePath))
        return;

    while (!DoSingleSearchStep(thePath)) {}
}

template<class state, class action, class environment, class priorityQueue>
bool DDBS<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state &from,
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
    expandForward = true;

    double forwardH = std::max(forwardHeuristic->HCost(start, goal), epsilon);
    double backwardH = std::max(backwardHeuristic->HCost(goal, start), epsilon);

    forwardQueue.setEnvironment(env);
    forwardQueue.AddOpenNode(start, 0, forwardH, 0);
    backwardQueue.setEnvironment(env);
    backwardQueue.AddOpenNode(goal, 0, backwardH, 0);

    C = std::max(forwardH, backwardH);

    UpdateQueuesAndCriterion();

    return true;
}

template<class state, class action, class environment, class priorityQueue>
bool DDBS<state, action, environment, priorityQueue>::UpdateC() {

    bool updated = false;

    double fBound = std::max(
            forwardQueue.getMinF(getMinBCriterion(true)) + backwardQueue.getMinD(getMinBCriterion(false)),
            backwardQueue.getMinF(getMinBCriterion(false)) + forwardQueue.getMinD(getMinBCriterion(true)));
    double gBound = forwardQueue.getMinG(getMinBCriterion(true))
                    + backwardQueue.getMinG(getMinBCriterion(false))
                    + epsilon;

    double bBound = computeBBound(forwardQueue.getMinB(getMinBCriterion(true)),
                                  backwardQueue.getMinB(getMinBCriterion(false)));

    while (C < std::max(gBound, std::max(fBound, bBound)) && C < currentCost) {
        C += gcd;
        updated = true;
        UpdateQueuesAndCriterion();

        fBound = std::max(forwardQueue.getMinF(getMinBCriterion(true)) + backwardQueue.getMinD(getMinBCriterion(false)),
                          backwardQueue.getMinF(getMinBCriterion(false)) + forwardQueue.getMinD(getMinBCriterion(true)));
        gBound = forwardQueue.getMinG(getMinBCriterion(true)) + backwardQueue.getMinG(getMinBCriterion(false)) + epsilon;
        bBound = computeBBound(forwardQueue.getMinB(getMinBCriterion(true)),
                               backwardQueue.getMinB(getMinBCriterion(false)));
    }

    return updated;
}

template<class state, class action, class environment, class priorityQueue>
void DDBS<state, action, environment, priorityQueue>::UpdateQueuesAndCriterion() {

    // forward queue limits
    forwardQueue.setLimits(C, C, C, C);

    forwardQueue.findBestBucket(getMinBCriterion(true));

    double gMinF = forwardQueue.getMinG(getMinBCriterion(true));
    double fMinF = forwardQueue.getMinF(getMinBCriterion(true));
    double dMinF = forwardQueue.getMinD(getMinBCriterion(true));
    double bMinF = forwardQueue.getMinB(getMinBCriterion(true));

    // backwards queue limits
    backwardQueue.setLimits(C - (gMinF + epsilon), C - dMinF, C - fMinF, 2 * C - bMinF);

    backwardQueue.findBestBucket(getMinBCriterion(false));

    double gMinB = backwardQueue.getMinG(getMinBCriterion(false));
    double fMinB = backwardQueue.getMinF(getMinBCriterion(false));
    double dMinB = backwardQueue.getMinD(getMinBCriterion(false));
    double bMinB = backwardQueue.getMinB(getMinBCriterion(false));

    bool limitsChanged;

    int counter = 0;

    do { // fixpoint computation of limits

        limitsChanged = false;

        // forward queue limits
        forwardQueue.setLimits(C - (gMinB + epsilon), C - dMinB, C - fMinB, 2 * C - bMinB);

        forwardQueue.findBestBucket(getMinBCriterion(true));

        double gMinF_new = forwardQueue.getMinG(getMinBCriterion(true));
        double fMinF_new = forwardQueue.getMinF(getMinBCriterion(true));
        double dMinF_new = forwardQueue.getMinD(getMinBCriterion(true));
        double bMinF_new = forwardQueue.getMinB(getMinBCriterion(true));

        limitsChanged |= gMinF != gMinF_new || fMinF != fMinF_new || dMinF != dMinF_new || bMinF != bMinF_new;

        gMinF = gMinF_new, fMinF = fMinF_new, dMinF = dMinF_new, bMinF = bMinF_new;

        // backwards queue limits
        backwardQueue.setLimits(C - (gMinF + epsilon), C - dMinF, C - fMinF, 2 * C - bMinF);

        backwardQueue.findBestBucket(getMinBCriterion(false));

        double gMinB_new = backwardQueue.getMinG(getMinBCriterion(false));
        double fMinB_new = backwardQueue.getMinF(getMinBCriterion(false));
        double dMinB_new = backwardQueue.getMinD(getMinBCriterion(false));
        double bMinB_new = backwardQueue.getMinB(getMinBCriterion(false));

        limitsChanged |= gMinB != gMinB_new || fMinB != fMinB_new || dMinB != dMinB_new || bMinB != bMinB_new;

        gMinB = gMinB_new, fMinB = fMinB_new, dMinB = dMinB_new, bMinB = bMinB_new;

        counter++;

    } while (limitsChanged);

    // TODO: parametrize criterion strategy
    criterion = RiseCriterion::RiseG;

}

template<class state, class action, class environment, class priorityQueue>
bool DDBS<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector <state> &thePath) {

    if (UpdateC()) {
        // TODO think how we are going to parametrize the tie breaker
    }

    if (fgreatereq(C, currentCost)) { // optimal solution found
        std::vector <state> pFor, pBack;
        ExtractPath(backwardQueue, middleNode, pBack);
        ExtractPath(forwardQueue, middleNode, pFor);
        reverse(pFor.begin(), pFor.end());
        thePath = pFor;
        thePath.insert(thePath.end(), pBack.begin() + 1, pBack.end());
        return true;
    }

    // TODO: parametrize better whether we want to alternate or to take a look at the open lists
    if (alternating) { // alternate directions
        if (expandForward) {
            Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start, true);
            expandForward = false;
        } else {
            Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal, false);
            expandForward = true;
        }
    } else { // choose side with the fewest nodes with minimum g

        double gNodesForward = forwardQueue.countMinimumGNodes();
        double gNodesBackward = backwardQueue.countMinimumGNodes();

        if (gNodesForward <= gNodesBackward)
            Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start, true);
        else
            Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal, false);
    }

    return false;
}

template<class state, class action, class environment, class priorityQueue>
void DDBS<state, action, environment, priorityQueue>::Expand(priorityQueue &current, priorityQueue &opposite,
                                                             Heuristic <state> *heuristic,
                                                             Heuristic <state> *reverseHeuristic,
                                                             const state &target, const state &source,
                                                             bool forward) {

    auto nodePair = current.Pop(getMinBCriterion(forward));

    if (nodePair.first == nullptr) { // despite apparently having expandable nodes, all were invalidated entries
        // TODO can this ever happen as it is right now? investigate
        return;
    }

    const auto node = nodePair.first;
    auto nodeG = nodePair.second;
    nodesExpanded++;

    counts[C] += 1;

    std::vector <state> neighbors;
    env->GetSuccessors(*node, neighbors);

    for (auto &succ : neighbors) {

        nodesTouched++;

        double succG = nodeG + env->GCost(*node, succ);

        double h = std::max(heuristic->HCost(succ, target), epsilon);

        // ignore states with greater cost than best solution
        // this can be either g + h
        if (fgreatereq(succG + h, currentCost))
            continue;

        // check if there is a collision
        auto collision = opposite.getNodeG(succ, reverseHeuristic->HCost(succ, source), getMinBCriterion(forward));
        if (collision.first) {
            auto gValue = collision.second;
            double collisionCost = succG + gValue.second;
            if (fless(collisionCost, currentCost)) {
                currentCost = collisionCost;
                middleNode = succ;

                if (fgreatereq(C, currentCost)) {
                    // add the node so the plan can be extracted
                    current.AddOpenNode(succ, succG, h, succG - reverseHeuristic->HCost(succ, source), node);
                    break; // step out, don't generate more nodes
                }
            } else if (gValue.first) {
                continue; // if the g value is provably optimal and the collision value is geq, prune the node
            }
        }

        // add it to the open list
        current.AddOpenNode(succ, succG, h, succG - reverseHeuristic->HCost(succ, source), node);
    }
}

#endif
