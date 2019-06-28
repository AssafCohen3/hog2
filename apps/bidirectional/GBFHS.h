#ifndef GBFHS_H
#define GBFHS_H

#include "BucketBasedList.h"
#include "FPUtil.h"
#include "Timer.h"
#include <unordered_map>

template<class state, class action, class environment, class priorityQueue = BucketBasedList<state, environment, BucketNodeData<state>>>
class GBFHS {
public:
    GBFHS(double threshold_ = -1) {
        forwardHeuristic = 0;
        backwardHeuristic = 0;
        env = 0;
        ResetNodeCount();
        expandForward = true;
        nodesExpanded = nodesTouched = 0;
        currentCost = DBL_MAX;
        threshold = threshold_;
    }

    virtual ~GBFHS() {}

    void GetPath(environment *env, const state &from, const state &to,
                 Heuristic <state> *forward, Heuristic <state> *backward, std::vector <state> &thePath);

    bool InitializeSearch(environment *env, const state &from, const state &to, Heuristic <state> *forward,
                          Heuristic <state> *backward, std::vector <state> &thePath);

    bool DoSingleSearchStep(std::vector <state> &thePath);

    virtual const char *GetName() { return "GBFHS"; }

    void ResetNodeCount() {
        nodesExpanded = nodesTouched = 0;
        counts.clear();
    }

    inline const int GetNumForwardItems() { return forwardQueue.size(); }

    inline const AStarOpenClosedData <state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }

    inline const int GetNumBackwardItems() { return backwardQueue.size(); }

    inline const AStarOpenClosedData <state> &GetBackwardItem(unsigned int which) {
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

    double getC() { return C; }
    double getGLimF() { return gLim_f; }
    double getGLimB() { return gLim_b; }

private:

    void ExtractPath(const priorityQueue &queue, const state &collisionState, std::vector <state> &thePath) {
        thePath.push_back(collisionState);
        auto parent = queue.Lookup(collisionState).parent;
        while (parent != nullptr) {
            thePath.push_back(*parent);
            parent = queue.Lookup(*parent).parent;
        }

    }

    void Expand(priorityQueue &current, priorityQueue &opposite,
                Heuristic <state> *heuristic, Heuristic <state> *reverseHeuristic,
                const state &target, const state &source, double gLim = DBL_MAX);

    bool UpdateC();

    void UpdateGLims();

    priorityQueue forwardQueue, backwardQueue;
    state goal, start;

    bool ExpandableNodes() {
        return forwardQueue.expandableNodes(C, gLim_f, true) > 0 || backwardQueue.expandableNodes(C, gLim_b, true) > 0;
    }

    // TODO: figure out the correct way, if probing is used alternating is most likely the best way
    bool SelectDirection() {
        int forwardExpandableNodes = forwardQueue.expandableNodes(C, gLim_f, true);
        int backwardExpandableNodes = backwardQueue.expandableNodes(C, gLim_b, true);

        if (forwardExpandableNodes == 0 && backwardExpandableNodes == 0) {
            std::cerr << "Selecting direction when there are no expandable nodes!" << std::endl;
            exit(0);
        }

        if (backwardExpandableNodes == 0) {
            expandForward = true;
        } else if (forwardExpandableNodes == 0) {
            expandForward = false;
        } else {
            expandForward = !expandForward;
        }

        return expandForward;
    }

    std::pair<double, double> SplitFunction() { // TODO: parametrize this

        if (threshold < 0.0) { // no threshold needed
            int forwardExpandableNodes = forwardQueue.expandableNodes(C, gLim_f + 1);
            int backwardExpandableNodes = backwardQueue.expandableNodes(C, gLim_b + 1);

            if (forwardExpandableNodes <= backwardExpandableNodes) {
                return std::make_pair(gLim_f + epsilon, gLim_b);
            } else {
                return std::make_pair(gLim_f, gLim_b + epsilon);
            }
        }

        if (gLim_f + 1 <= threshold) {
//            std::cout << "FW increased: " << threshold << " | " << gLim_f << " + " << gLim_b << std::endl;
            return std::make_pair(gLim_f + epsilon, gLim_b);
        } else {
//            std::cout << "FW increased: " << threshold << " | " << gLim_f << " + " << gLim_b << std::endl;
            return std::make_pair(gLim_f, gLim_b + epsilon);
        }

    }

    uint64_t nodesTouched, nodesExpanded;

    std::map<double, int> counts;

    state middleNode;
    double currentCost;
    double epsilon = 1.0; // TODO parametrize epsilon

    environment *env;
    Timer t;
    Heuristic <state> *forwardHeuristic;
    Heuristic <state> *backwardHeuristic;

    bool expandForward = true;
    bool updateGLimEagerly = true; // TODO: parametrize this

    double C, gLim_f, gLim_b = 0.0;

    double threshold;
};

template<class state, class action, class environment, class priorityQueue>
void GBFHS<state, action, environment, priorityQueue>::GetPath(environment *env, const state &from, const state &to,
                                                               Heuristic <state> *forward,
                                                               Heuristic <state> *backward,
                                                               std::vector <state> &thePath) {
    if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
        return;

    t.StartTimer();

    while (!DoSingleSearchStep(thePath)) {}
}

template<class state, class action, class environment, class priorityQueue>
bool GBFHS<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state &from,
                                                                        const state &to,
                                                                        Heuristic <state> *forward,
                                                                        Heuristic <state> *backward,
                                                                        std::vector <state> &thePath) {
    this->env = env;
    forwardHeuristic = forward;
    backwardHeuristic = backward;
    currentCost = DBL_MAX;
    forwardQueue.Reset();
    backwardQueue.Reset();
    ResetNodeCount();
    thePath.resize(0);
    start = from;
    goal = to;
    if (start == goal)
        return false;
    expandForward = true;

    double startH = std::max(forwardHeuristic->HCost(start, goal), epsilon);
    double goalH = std::max(backwardHeuristic->HCost(goal, start), epsilon);

    forwardQueue.setEnvironment(env);
    forwardQueue.AddOpenNode(start, 0, startH);
    backwardQueue.setEnvironment(env);
    backwardQueue.AddOpenNode(goal, 0, goalH);

    C = std::max(startH, goalH);
    gLim_f = 0.0;
    gLim_b = 0.0;

    return true;
}

template<class state, class action, class environment, class priorityQueue>
bool GBFHS<state, action, environment, priorityQueue>::UpdateC() {

    bool updated = false;

    while (!ExpandableNodes() && C < currentCost) {
        if (gLim_f + gLim_b + epsilon - 1 >= C) { // we have to update C
            auto minFF = forwardQueue.getMinF(C);
            auto minFB = backwardQueue.getMinF(C);
            C = std::min(minFF, minFB);

            updated = true;

            // if gLims are to be updated eagerly after C, do it
            if (updateGLimEagerly && C < currentCost) {
                UpdateGLims();
            }
        } else { // just updating gLims can make new nodes expandable
            UpdateGLims();
        }
    }

    // return whether a better collision is possible after updating (if updated) or not
    return updated && (gLim_f + gLim_b + epsilon - 1 >= C) && (C < currentCost);
}

template<class state, class action, class environment, class priorityQueue>
void GBFHS<state, action, environment, priorityQueue>::UpdateGLims() {
    // update g lims until they reach C eagerly or as long as there are no expandable nodes
    while (gLim_f + gLim_b + epsilon - 1 < C && (updateGLimEagerly || !ExpandableNodes())) {
        std::pair<double, double> limits = SplitFunction();
        gLim_f = limits.first;
        gLim_b = limits.second;

        if (gLim_f + gLim_b > C) {
            std::cerr << "  Wrong limits!!: " << (gLim_f + gLim_b) << " at " << C << std::endl;
            exit(0);
        }
    }
}

template<class state, class action, class environment, class priorityQueue>
bool GBFHS<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector <state> &thePath) {

    if (UpdateC()) {
        // TODO: include tie-breaker procedure
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

    bool expandForward = SelectDirection();

    if (expandForward) {
        Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start, gLim_f);
        expandForward = false;
    } else {
        Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal, gLim_b);
        expandForward = true;
    }

    return false;
}

template<class state, class action, class environment, class priorityQueue>
void GBFHS<state, action, environment, priorityQueue>::Expand(priorityQueue &current, priorityQueue &opposite,
                                                              Heuristic <state> *heuristic,
                                                              Heuristic <state> *reverseHeuristic,
                                                              const state &target,
                                                              const state &source,
                                                              double gLim) {
    auto nodePair = current.Pop(C, gLim);
    if (nodePair.first == nullptr) return; // if current contains only invalid entries, just return

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
        // TODO: this can be improved by checking collisions as well and using that value if the other g is optimal
        if (fgreatereq(succG + h, currentCost))
            continue;

        // check if there is a collision
        auto collision = opposite.getNodeG(succ);
        if (collision.first) {
            double collisionCost = succG + collision.second;
            if (fgreatereq(collisionCost, currentCost)) { // cost higher than the current solution, discard
                continue;
            } else if (fless(collisionCost, currentCost)) {
//                std::cout << " Collision found: " << collisionCost << " at " << C << std::endl;
                currentCost = collisionCost;
                middleNode = succ;
                current.AddOpenNode(succ, succG, h, node); // add the node so the plan can be extracted
                if (fgreatereq(C, currentCost)) {
                    break; // step out, don't generate more nodes
                }
            }
        }

        // add it to the open list
        current.AddOpenNode(succ, succG, h, node);
    }
}

#endif /* GBFHS_h */
