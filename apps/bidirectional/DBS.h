#ifndef DBS_H
#define DBS_H

#include "ThreeDimBucketBasedList.h"
#include "FPUtil.h"
#include <unordered_map>
#include <iostream>

template<class state, class action, class environment, class priorityQueue = ThreeDimBucketBasedList<state, environment, BucketNodeData<state>>>
class DBS {
public:
    DBS(bool alternating_ = true, double epsilon_ = 1.0, double gcd_ = 1.0) {
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

    ~DBS() {
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

    MinCriterion getMinCriterion(bool forwardQueue) {
        switch (criterion) {
            case RiseG:
                return MinCriterion::MinG;
            case RiseForward:
                return forwardQueue ? MinCriterion::MinF : MinCriterion::MinD;
            case RiseBackward:
                return forwardQueue ? MinCriterion::MinD : MinCriterion::MinF;
        }
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
void DBS<state, action, environment, priorityQueue>::GetPath(environment *env, const state &from, const state &to,
                                                             Heuristic <state> *forward,
                                                             Heuristic <state> *backward,
                                                             std::vector <state> &thePath) {
    if (!InitializeSearch(env, from, to, forward, backward, thePath))
        return;

    while (!DoSingleSearchStep(thePath)) {}
}

template<class state, class action, class environment, class priorityQueue>
bool DBS<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state &from,
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
bool DBS<state, action, environment, priorityQueue>::UpdateC() {

    bool updated = false;

    double fBound = std::max(
            forwardQueue.getMinF(getMinCriterion(true)) + backwardQueue.getMinD(getMinCriterion(false)),
            backwardQueue.getMinF(getMinCriterion(false)) + forwardQueue.getMinD(getMinCriterion(true)));
    double gBound = forwardQueue.getMinG(getMinCriterion(true))
                    + backwardQueue.getMinG(getMinCriterion(false))
                    + epsilon;

    while (C < std::max(gBound, fBound) && C < currentCost) {
//        if (!alternating && C < gBound ) std::cout << "  -- C g Updated: " << (C + gcd) << " and " << currentCost << std::endl;
//        if (!alternating && C < fBound ) std::cout << "  -- C f Updated: " << (C + gcd) << " and " << currentCost << std::endl;
        C += gcd;
        updated = true;
        UpdateQueuesAndCriterion();

        fBound = std::max(forwardQueue.getMinF(getMinCriterion(true)) + backwardQueue.getMinD(getMinCriterion(false)),
                          backwardQueue.getMinF(getMinCriterion(false)) + forwardQueue.getMinD(getMinCriterion(true)));
        gBound = forwardQueue.getMinG(getMinCriterion(true)) + backwardQueue.getMinG(getMinCriterion(false)) + epsilon;
    }

    return updated;
}

template<class state, class action, class environment, class priorityQueue>
void DBS<state, action, environment, priorityQueue>::UpdateQueuesAndCriterion() {

    // forward queue limits
    forwardQueue.setLimits(C, C, C);

    forwardQueue.findBestBucket(getMinCriterion(true));

    double gMinF = forwardQueue.getMinG(getMinCriterion(true));
    double fMinF = forwardQueue.getMinF(getMinCriterion(true));
    double dMinF = forwardQueue.getMinD(getMinCriterion(true));

    // backwards queue limits
    backwardQueue.setLimits(C - (gMinF + epsilon), C - dMinF, C - fMinF);

    backwardQueue.findBestBucket(getMinCriterion(false));

    double gMinB = backwardQueue.getMinG(getMinCriterion(false));
    double fMinB = backwardQueue.getMinF(getMinCriterion(false));
    double dMinB = backwardQueue.getMinD(getMinCriterion(false));

    bool limitsChanged;

    int counter = 0;

    do { // fixpoint computation of limits

        limitsChanged = false;

        // forward queue limits
        forwardQueue.setLimits(C - (gMinB + epsilon), C - dMinB, C - fMinB);

        forwardQueue.findBestBucket(getMinCriterion(true));

        double gMinF_new = forwardQueue.getMinG(getMinCriterion(true));
        double fMinF_new = forwardQueue.getMinF(getMinCriterion(true));
        double dMinF_new = forwardQueue.getMinD(getMinCriterion(true));

        limitsChanged |= gMinF != gMinF_new || fMinF != fMinF_new || dMinF != dMinF_new;

        gMinF = gMinF_new, fMinF = fMinF_new, dMinF = dMinF_new;

        // backwards queue limits
        backwardQueue.setLimits(C - (gMinF + epsilon), C - dMinF, C - fMinF);

        backwardQueue.findBestBucket(getMinCriterion(false));

        double gMinB_new = backwardQueue.getMinG(getMinCriterion(false));
        double fMinB_new = backwardQueue.getMinF(getMinCriterion(false));
        double dMinB_new = backwardQueue.getMinD(getMinCriterion(false));

        limitsChanged |= gMinB != gMinB_new || fMinB != fMinB_new || dMinB != dMinB_new;

        gMinB = gMinB_new, fMinB = fMinB_new, dMinB = dMinB_new;

        counter++;

    } while (limitsChanged);

    // TODO: parametrize criterion strategy
    criterion = RiseCriterion::RiseG;

}

template<class state, class action, class environment, class priorityQueue>
bool DBS<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector <state> &thePath) {

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
        double fMinF = forwardQueue.getMinF(getMinCriterion(true));
        double dMinF = forwardQueue.getMinD(getMinCriterion(true));

        double fMinB = backwardQueue.getMinF(getMinCriterion(false));
        double dMinB = backwardQueue.getMinD(getMinCriterion(false));

        double gNodesForward = forwardQueue.countMinimumGNodes();
        double gNodesBackward = backwardQueue.countMinimumGNodes();

//        std::cout << "  Direction: " << gNodesForward << " - " << gNodesBackward << std::endl;

        if (gNodesForward <= gNodesBackward)
            Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start, true);
        else
            Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal, false);
    }

    return false;
}

template<class state, class action, class environment, class priorityQueue>
void DBS<state, action, environment, priorityQueue>::Expand(priorityQueue &current, priorityQueue &opposite,
                                                            Heuristic <state> *heuristic,
                                                            Heuristic <state> *reverseHeuristic,
                                                            const state &target, const state &source,
                                                            bool forward) {

    auto nodePair = current.Pop(getMinCriterion(forward));

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
        auto collision = opposite.getNodeG(succ, reverseHeuristic->HCost(succ, source), getMinCriterion(forward));
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
