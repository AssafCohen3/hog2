#ifndef DBBS_H
#define DBBS_H

#include "BidirErrorBucketBasedList.h"
#include "FPUtil.h"
#include <unordered_map>
#include <iostream>
#include <math.h>

template<class state, class action, class environment, bool useB = true, class priorityQueue = BidirErrorBucketBasedList<state, environment, useB, BucketNodeData<state>>>
class DBBS {
public:
    DBBS(bool alternating_ = true, double epsilon_ = 1.0, double gcd_ = 1.0) {
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

    ~DBBS() {
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
                const state &target, const state &source);

    bool UpdateC();

    double GetNextC();

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
};

template<class state, class action, class environment, bool useB, class priorityQueue>
void
DBBS<state, action, environment, useB, priorityQueue>::GetPath(environment *env, const state &from, const state &to,
                                                               Heuristic <state> *forward,
                                                               Heuristic <state> *backward,
                                                               std::vector <state> &thePath) {
    if (!InitializeSearch(env, from, to, forward, backward, thePath))
        return;

    while (!DoSingleSearchStep(thePath)) {}
}

template<class state, class action, class environment, bool useB, class priorityQueue>
bool DBBS<state, action, environment, useB, priorityQueue>::InitializeSearch(environment *env, const state &from,
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

    return true;
}

template<class state, class action, class environment, bool useB, class priorityQueue>
bool DBBS<state, action, environment, useB, priorityQueue>::UpdateC() {

    if (forwardQueue.isBestBucketComputed() && backwardQueue.isBestBucketComputed())
        return false; // no need to recompute anything, and no need to rise C

    bool incrementedC = false;

    while (C < currentCost && !forwardQueue.isBestBucketComputed() || !backwardQueue.isBestBucketComputed()) {

        // initial forward queue limits
        forwardQueue.computeBestBucket(getMinCriterion(true), C, C, C, 2.0 * C);

        double gMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinG() : C;
        double fMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinF() : C;
        double dMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinD() : C;
        double bMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinB() : 2 * C;

        // initial backwards queue limits
        backwardQueue.computeBestBucket(getMinCriterion(false), C - (gMinF + epsilon), C - dMinF, C - fMinF,
                                        2.0 * C - bMinF);

        double gMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinG() : C;
        double fMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinF() : C;
        double dMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinD() : C;
        double bMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinB() : 2 * C;

        bool limitsChanged = true;

        // fixpoint computation of limits
        while (limitsChanged && forwardQueue.isBestBucketComputed() && backwardQueue.isBestBucketComputed()) {

            limitsChanged = false;

            // forward queue limits
            forwardQueue.computeBestBucket(getMinCriterion(true), C - (gMinB + epsilon), C - dMinB, C - fMinB,
                                           2.0 * C - bMinB);

            double gMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinG() : C;
            double fMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinF() : C;
            double dMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinD() : C;
            double bMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinB() : 2 * C;

            limitsChanged |= gMinF != gMinF_new || fMinF != fMinF_new || dMinF != dMinF_new || bMinF != bMinF_new;

            gMinF = gMinF_new, fMinF = fMinF_new, dMinF = dMinF_new, bMinF = bMinF_new;

            // backwards queue limits
            backwardQueue.computeBestBucket(getMinCriterion(false), C - (gMinF + epsilon), C - dMinF, C - fMinF,
                                            2.0 * C - bMinF);

            double gMinB_new = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinG() : C;
            double fMinB_new = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinF() : C;
            double dMinB_new = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinD() : C;
            double bMinB_new = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinB() : 2 * C;

            limitsChanged |= gMinB != gMinB_new || fMinB != fMinB_new || dMinB != dMinB_new || bMinB != bMinB_new;

            gMinB = gMinB_new, fMinB = fMinB_new, dMinB = dMinB_new, bMinB = bMinB_new;
        };

        // if limits don't change and still no expandable bucket is found, increase C
        if (!forwardQueue.isBestBucketComputed() || !backwardQueue.isBestBucketComputed()) {
//            C += gcd;
            C = GetNextC();
            incrementedC = true;
        }
    }

    // TODO: parametrize criterion strategy
    criterion = RiseCriterion::RiseG;

    return incrementedC;
}

template<class state, class action, class environment, bool useB, class priorityQueue>
double DBBS<state, action, environment, useB, priorityQueue>::GetNextC() {

    double result = DBL_MAX;

    const NodeValues forwardValues = forwardQueue.getNodeValues();
    const NodeValues backwardValues = backwardQueue.getNodeValues();

    // g bound
    const auto &forward_g_values = forwardValues.g_values;
    const auto &backward_g_values = backwardValues.g_values;

    for (const double fw_g_value : forward_g_values)
        for (const double bw_g_value : backward_g_values) {
            double g_bound = fw_g_value + bw_g_value + epsilon;
            if (g_bound > C && g_bound < result)
                result = g_bound;
        }

    const auto &forward_f_values = forwardValues.f_values;
    const auto &forward_d_values = forwardValues.d_values;
    const auto &backward_f_values = backwardValues.f_values;
    const auto &backward_d_values = backwardValues.d_values;

    // forward KK bound
    for (const double fw_f_value : forward_f_values)
        for (const double bw_d_value : backward_d_values) {
            double fw_KK_bound = fw_f_value + bw_d_value;
            if (fw_KK_bound > C && fw_KK_bound < result)
                result = fw_KK_bound;
        }

    // backward KK bound
    for (const double bw_f_value : backward_f_values)
        for (const double fw_d_value : forward_d_values) {
            double bw_KK_bound = bw_f_value + fw_d_value;
            if (bw_KK_bound > C && bw_KK_bound < result)
                result = bw_KK_bound;
        }

    // b bound
    if (useB) {
        const auto &forward_b_values = forwardValues.b_values;
        const auto &backward_b_values = backwardValues.b_values;

        for (const double fw_b_value : forward_b_values)
            for (const double bw_b_value : backward_b_values) {
                double b_bound = gcd * std::ceil(((fw_b_value + bw_b_value) / 2) / gcd);
                if (b_bound > C && b_bound < result)
                    result = b_bound;
            }
    }


    return result;

}

template<class state, class action, class environment, bool useB, class priorityQueue>
bool DBBS<state, action, environment, useB, priorityQueue>::DoSingleSearchStep(std::vector <state> &thePath) {

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
            Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
            expandForward = false;
        } else {
            Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
            expandForward = true;
        }
    } else { // choose side with the fewest nodes with minimum g

        double gNodesForward = forwardQueue.countMinimumGNodes();
        double gNodesBackward = backwardQueue.countMinimumGNodes();

        if (gNodesForward <= gNodesBackward)
            Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
        else
            Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
    }

    return false;
}

template<class state, class action, class environment, bool useB, class priorityQueue>
void DBBS<state, action, environment, useB, priorityQueue>::Expand(priorityQueue &current, priorityQueue &opposite,
                                                                   Heuristic <state> *heuristic,
                                                                   Heuristic <state> *reverseHeuristic,
                                                                   const state &target, const state &source) {

    auto nodePair = current.Pop();

    // despite apparently having expandable nodes, best candidates may be invalidated entries
    if (nodePair.first == nullptr) {
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
        auto collision = opposite.getNodeG(succ, reverseHeuristic->HCost(succ, source));
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
