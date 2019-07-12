//
//  BidirPancake.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/7/17.
//  Copyright Â© 2017 University of Denver. All rights reserved.
//

#include "BidirPancake.h"
#include "PancakePuzzle.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "Baseline.h"
#include "GBFHS.h"
#include "DBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "DVCBS.h"
#include "PancakeInstances.h"
#include "CalculateWVC.h"
#include "fMM.h"
//#include "WeightedVertexGraph.h"

const int S = 10; // must be factor of sizes below

void TestPancakeTR();

void TestPancakeRandom();

void TestPancakeHard();

void TestRob();

void TestPancake() {
//	TestRob();
    TestPancakeRandom();
//	TestPancakeHard();
    exit(0);
}

void TestRob() {
//	0 3 2 1
    PancakePuzzleState<4> start;
    PancakePuzzleState<4> goal;
    PancakePuzzle<4> cake(1);
    ZeroHeuristic <PancakePuzzleState<4>> z;
    std::vector <PancakePuzzleState<4>> path;
    start.puzzle[0] = 0;
    start.puzzle[1] = 3;
    start.puzzle[2] = 2;
    start.puzzle[3] = 1;
    goal.puzzle[0] = 1;
    goal.puzzle[1] = 3;
    goal.puzzle[2] = 2;
    goal.puzzle[3] = 0;
    NBS<PancakePuzzleState < 4>, PancakePuzzleAction, PancakePuzzle < 4 >> nbs;
    MM <PancakePuzzleState<4>, PancakePuzzleAction, PancakePuzzle<4>> mm;
    mm.GetPath(&cake, start, goal, &cake, &cake, path);
    printf("MM: %lld expansions\n", mm.GetNodesExpanded());
    mm.GetPath(&cake, start, goal, &z, &z, path);
    printf("MM0: %lld expansions\n", mm.GetNodesExpanded());

    exit(0);
}

void TestPancakeTR() {
    // multiples of 5
    int arrangement[] = {0, 2, 4, 1, 3, 5, 7, 9, 6, 8, 10, 12, 14, 11, 13, 15, 17, 19, 16, 18, 20, 22, 24, 21, 23, 25,
                         27, 29, 26, 28, 30, 32, 34, 31, 33, 35, 37, 39, 36, 38, 40, 42, 44, 41, 43, 45, 47, 49, 46, 48,
                         50, 52, 54, 51, 53, 55, 57, 59, 56, 58, 60, 62, 64, 61, 63, 65, 67, 69, 66, 68, 70, 72, 74, 71,
                         73, 75, 77, 79, 76, 78, 80, 82, 84, 81, 83, 85, 87, 89, 86, 88, 90, 92, 94, 91, 93, 95, 97, 99,
                         96, 98,};
    // multiples of 9
//	const int arrangement[] = {0,4,7,2,5,8,3,6,1,9,13,16,11,14,17,12,15,10,18,22,25,20,23,26,21,24,19,27,31,34,29,32,35,30,33,28,36,40,43,38,41,44,39,42,37,45,49,52,47,50,53,48,51,46,54,58,61,56,59,62,57,60,55,63,67,70,65,68,71,66,69,64,72,76,79,74,77,80,75,78,73,81,85,88,83,86,89,84,87,82,90,94,97,92,95,98,93,96,91};

    for (int gap = 0; gap < 10; gap++) {

        PancakePuzzleState <S> start;
        PancakePuzzleState <S> goal;
        PancakePuzzle <S> pancake(gap);
        PancakePuzzle <S> pancake2(gap);

        NBS<PancakePuzzleState < S>, PancakePuzzleAction, PancakePuzzle < S >> nbs;
        DVCBS<PancakePuzzleState < S>, PancakePuzzleAction, PancakePuzzle < S >> dvcbs(1);
        MM <PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> mm;
        TemplateAStar <PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> astar;
        IDAStar<PancakePuzzleState<S>, PancakePuzzleAction, false> idastar;

        std::vector <PancakePuzzleState<S>> nbsPath;
        std::vector <PancakePuzzleState<S>> dvcbsPath;
        std::vector <PancakePuzzleState<S>> astarPath;
        std::vector <PancakePuzzleState<S>> mmPath;
        std::vector <PancakePuzzleAction> idaPath;
        Timer t1, t2, t3, t4, t5;


        goal.Reset();
        for (int x = 0; x < S; x++)
            start.puzzle[x] = arrangement[x];


        t1.StartTimer();
        astar.GetPath(&pancake, start, goal, astarPath);
        t1.EndTimer();
        uint64_t necessary = 0;
        double solutionCost = pancake.GetPathLength(astarPath);
        for (unsigned int x = 0; x < astar.GetNumItems(); x++) {
            const auto &item = astar.GetItem(x);
            if ((item.where == kClosedList) && (item.g + item.h < solutionCost))
                necessary++;
        }
        printf("A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(astarPath),
               astar.GetNodesExpanded(), necessary, t1.GetElapsedTime());

        goal.Reset();
        for (int x = 0; x < S; x++)
            start.puzzle[x] = arrangement[x];
        t2.StartTimer();
        nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
        t2.EndTimer();
        printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(nbsPath),
               nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime());

        goal.Reset();

        for (int x = 0; x < S; x++)
            start.puzzle[x] = arrangement[x];
        t5.StartTimer();
        dvcbs.GetPath(&pancake, start, goal, &pancake, &pancake2, dvcbsPath);
        t5.EndTimer();
        printf("DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(dvcbsPath),
               dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t5.GetElapsedTime());

        goal.Reset();

        for (int x = 0; x < S; x++)
            start.puzzle[x] = arrangement[x];
        t3.StartTimer();
        idastar.GetPath(&pancake, start, goal, idaPath);
        t3.EndTimer();
        printf("IDA* found path length %ld; %llu expanded; %llu generated; %1.2fs elapsed\n", idaPath.size(),
               idastar.GetNodesExpanded(), idastar.GetNodesTouched(), t3.GetElapsedTime());


        goal.Reset();
        for (int x = 0; x < S; x++)
            start.puzzle[x] = arrangement[x];
        t4.StartTimer();
        mm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
        t4.EndTimer();
        printf("MM found path length %1.0f; %llu expanded; %1.2fs elapsed\n", pancake.GetPathLength(mmPath),
               mm.GetNodesExpanded(), t4.GetElapsedTime());

        printf("Problem & IDA* & & A* & & MM & & NBS* & DVCBS & \\\\\n");
        printf("%d G-%d & %llu & %1.2fs & %llu & %1.2fs & %llu & %1.2fs & %llu & %1.2fs & %llu & %1.2fs \\\\ \n", S,
               gap,
               idastar.GetNodesExpanded(), t3.GetElapsedTime(),
               astar.GetNodesExpanded(), t1.GetElapsedTime(),
               mm.GetNodesExpanded(), t4.GetElapsedTime(),
               nbs.GetNodesExpanded(), t2.GetElapsedTime(), dvcbs.GetNodesExpanded(), t5.GetElapsedTime());
    }
    exit(0);
}

const int N = 14;
const int INSTANCES = 50;

void TestPancakeRandom() {
    for (int gap = 0; gap < 4; gap++) {
        srandom(0);
        PancakePuzzleState <N> start;
        PancakePuzzleState <N> original;
        PancakePuzzleState <N> goal;
        PancakePuzzle <N> pancake(gap);
        PancakePuzzle <N> pancake2(gap);


        std::vector <PancakePuzzleState<N>> nbsPath;
        std::vector <PancakePuzzleState<N>> dvcbsPath;
        std::vector <PancakePuzzleState<N>> nbsEpsilonPath;
        std::vector <PancakePuzzleState<N>> baselinePath;
        std::vector <PancakePuzzleState<N>> gbfhsPath;
        std::vector <PancakePuzzleState<N>> dbsPath;
        std::vector <PancakePuzzleState<N>> dvcbsEpsilonPath;
        std::vector <PancakePuzzleState<N>> bsPath;
        std::vector <PancakePuzzleState<N>> astarPath;
        std::vector <PancakePuzzleState<N>> rastarPath;
        std::vector <PancakePuzzleState<N>> mmPath;
        std::vector <PancakePuzzleState<N>> fmmPath;
        std::vector <PancakePuzzleAction> idaPath;
        Timer t1, t2, t3, t4, t5, t6, t7, t8;

        // variables for WVC calaculator (optimal p for FMM)
        std::vector < AStarOpenClosedData < PancakePuzzleState < N >> > astarOpenClose;
        std::vector < AStarOpenClosedData < PancakePuzzleState < N >> > rastarOpenClose;

        long nodes_Astar = 0, nodes_Astarn = 0,
                nodes_NBS = 0, nodes_NBSn = 0, nodes_NBSa = 0, nodes_NBSan = 0,
                nodes_DVCBS = 0, nodes_DVCBSn = 0, nodes_DVCBSa = 0, nodes_DVCBSan = 0,
                nodes_NBB = 0, nodes_NBBn = 0, nodes_GBFHS = 0, nodes_GBFHSn = 0, nodes_GBFHSl = 0, nodes_GBFHSln = 0,
                nodes_DBS = 0, nodes_DBSn = 0,
                nodes_GBFHSbest = 0, nodes_GBFHSbestn = 0;

        for (int count = 0; count < INSTANCES; count++) {

            if (count + 1.0 > INSTANCES + 1.0) {
                break;
            }

            srandom(random());

            double optimal_cost = -1.0;

            goal.Reset();
            original.Reset();
            for (int x = 0; x < N; x++) {
                std::swap(original.puzzle[x], original.puzzle[x + random() % (N - x)]);
            }
            // 13 7 12 2 10 9 11 6 8 5 4 1 0 3
//            original.puzzle[0] = 13;
//            original.puzzle[1] = 7;
//            original.puzzle[2] = 12;
//            original.puzzle[3] = 2;
//            original.puzzle[4] = 10;
//            original.puzzle[5] = 9;
//            original.puzzle[6] = 11;
//            original.puzzle[7] = 6;
//            original.puzzle[8] = 8;
//            original.puzzle[9] = 5;
//            original.puzzle[10] = 4;
//            original.puzzle[11] = 1;
//            original.puzzle[12] = 0;
//            original.puzzle[13] = 3;

            std::cout << "Pancake problem: " << (count) << " of " << INSTANCES << std::endl;
            std::cout << original << std::endl;

            // A*
            if (0) {
                if (gap != 3) {
                    TemplateAStar <PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar;
                    start = original;
                    t1.StartTimer();
                    astar.GetPath(&pancake, start, goal, astarPath);
                    t1.EndTimer();
                    printf("GAP-%d A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                           pancake.GetPathLength(astarPath),
                           astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
                    astarOpenClose = astar.openClosedList.elements;
                }
                // A*-A
                if (gap != 3) {
                    TemplateAStar <PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar(true);
                    start = original;
                    t1.StartTimer();
                    astar.GetPath(&pancake, start, goal, astarPath);
                    t1.EndTimer();
                    printf("GAP-%d A*-A found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                           pancake.GetPathLength(astarPath),
                           astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
                    astarOpenClose = astar.openClosedList.elements;
                }
            }
            if (0) {
                // A*
                if (gap != 3) {
                    TemplateAStar <PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar(false, 1);
                    start = original;
                    t1.StartTimer();
                    astar.GetPath(&pancake, start, goal, astarPath);
                    t1.EndTimer();
                    printf("GAP-%d A*-E found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                           pancake.GetPathLength(astarPath),
                           astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
                    astarOpenClose = astar.openClosedList.elements;
                }
                // A*-A
                if (gap != 3) {
                    TemplateAStar <PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar(true, 1);
                    start = original;
                    t1.StartTimer();
                    astar.GetPath(&pancake, start, goal, astarPath);
                    t1.EndTimer();
                    printf("GAP-%d A*-A-E found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                           gap, pancake.GetPathLength(astarPath),
                           astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
                    astarOpenClose = astar.openClosedList.elements;
                }
            }
            if (0) {
                // R-A*
                if (gap != 3) {
                    TemplateAStar <PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> rastar;
                    goal.Reset();
                    start = original;
                    t1.StartTimer();
                    rastar.GetPath(&pancake, goal, start, rastarPath);
                    t1.EndTimer();
                    printf("GAP-%d R-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                           pancake.GetPathLength(rastarPath),
                           rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions(), t1.GetElapsedTime());
                    rastarOpenClose = rastar.openClosedList.elements;
                }
            }

            //Single Solution
            if (1) {
                // NBS
                {
                    NBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, NBSQueue<
                            PancakePuzzleState < N>, 1, false >> nbsEpsilon(false);
                    goal.Reset();
                    start = original;
                    t2.StartTimer();
                    nbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsEpsilonPath);
                    t2.EndTimer();
                    printf("GAP-%d NBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(nbsEpsilonPath),
                           nbsEpsilon.GetNodesExpanded(), nbsEpsilon.GetNecessaryExpansions(), t2.GetElapsedTime(),
                           nbsEpsilon.getForwardMeetingPoint(), nbsEpsilon.getBackwardMeetingPoint(),
                           nbsEpsilon.getForwardUnnecessaryNodesInPath(),
                           nbsEpsilon.getBackwardUnnecessaryNodesInPath(), nbsEpsilon.GetExpansionUntilFirstSolution());

                    nodes_NBS += nbsEpsilon.GetNodesExpanded();
                    nodes_NBSn += nbsEpsilon.GetNecessaryExpansions();

                    // test optimality
                    if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(nbsEpsilonPath);
                    else if (optimal_cost != pancake.GetPathLength(nbsEpsilonPath)) {
                        printf("GAP-%d NBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                               optimal_cost, pancake.GetPathLength(nbsEpsilonPath));
                        exit(0);
                    }

                    NBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, NBSQueue<
                            PancakePuzzleState < N>, 1, true >> nbsEpsilonLeq(false, true);
                    goal.Reset();
                    start = original;
                    t2.StartTimer();
                    nbsEpsilonLeq.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsEpsilonPath);
                    t2.EndTimer();
                    printf("GAP-%d NBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(nbsEpsilonPath),
                           nbsEpsilonLeq.GetNodesExpanded(), nbsEpsilonLeq.GetNecessaryExpansions(),
                           t2.GetElapsedTime(), nbsEpsilonLeq.getForwardMeetingPoint(),
                           nbsEpsilonLeq.getBackwardMeetingPoint(), nbsEpsilonLeq.getForwardUnnecessaryNodesInPath(),
                           nbsEpsilonLeq.getBackwardUnnecessaryNodesInPath(),
                           nbsEpsilonLeq.GetExpansionUntilFirstSolution());

                    nodes_NBSa += nbsEpsilonLeq.GetNodesExpanded();
                    nodes_NBSan += nbsEpsilonLeq.GetNecessaryExpansions();

                    // test optimality
                    if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(nbsEpsilonPath);
                    else if (optimal_cost != pancake.GetPathLength(nbsEpsilonPath)) {
                        printf("GAP-%d NBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                               optimal_cost, pancake.GetPathLength(nbsEpsilonPath));
                        exit(0);
                    }
                }

                // DVCBS
                if (1) {
                    DVCBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, DVCBSQueue<
                            PancakePuzzleState < N>, 1, false >> dvcbsEpsilon(false);
                    goal.Reset();
                    start = original;
                    t6.StartTimer();
                    dvcbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, dvcbsEpsilonPath);
                    t6.EndTimer();
                    printf("GAP-%d DVCBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(dvcbsEpsilonPath),
                           dvcbsEpsilon.GetNodesExpanded(), dvcbsEpsilon.GetNecessaryExpansions(), t6.GetElapsedTime(),
                           dvcbsEpsilon.getForwardMeetingPoint(), dvcbsEpsilon.getBackwardMeetingPoint(),
                           dvcbsEpsilon.getForwardUnnecessaryNodesInPath(),
                           dvcbsEpsilon.getBackwardUnnecessaryNodesInPath(),
                           dvcbsEpsilon.GetExpansionUntilFirstSolution());

                    nodes_DVCBS += dvcbsEpsilon.GetNodesExpanded();
                    nodes_DVCBSn += dvcbsEpsilon.GetNecessaryExpansions();

                    // test optimality
//                    if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(dvcbsEpsilonPath);
//                    else if (optimal_cost != pancake.GetPathLength(dvcbsEpsilonPath)) {
//                        printf("GAP-%d DVCBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
//                               optimal_cost, pancake.GetPathLength(dvcbsEpsilonPath));
//                        exit(0);
//                    }
                }
                if (1) {
                    DVCBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, DVCBSQueue<
                            PancakePuzzleState < N>, 1, true >> dvcbsEpsilon(false, true);
                    goal.Reset();
                    start = original;
                    t6.StartTimer();
                    dvcbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, dvcbsEpsilonPath);
                    t6.EndTimer();
                    printf("GAP-%d DVCBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(dvcbsEpsilonPath),
                           dvcbsEpsilon.GetNodesExpanded(), dvcbsEpsilon.GetNecessaryExpansions(), t6.GetElapsedTime(),
                           dvcbsEpsilon.getForwardMeetingPoint(), dvcbsEpsilon.getBackwardMeetingPoint(),
                           dvcbsEpsilon.getForwardUnnecessaryNodesInPath(),
                           dvcbsEpsilon.getBackwardUnnecessaryNodesInPath(),
                           dvcbsEpsilon.GetExpansionUntilFirstSolution());

                    nodes_DVCBSa += dvcbsEpsilon.GetNodesExpanded();
                    nodes_DVCBSan += dvcbsEpsilon.GetNecessaryExpansions();

                    // test optimality
//                    if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(dvcbsEpsilonPath);
//                    else if (optimal_cost != pancake.GetPathLength(dvcbsEpsilonPath)) {
//                        printf("GAP-%d DVCBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
//                               optimal_cost, pancake.GetPathLength(dvcbsEpsilonPath));
//                        exit(0);
//                    }
                }
            }

            // NBB
            if (1) {
                Baseline<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> baseline;
                goal.Reset();
                start = original;
                t7.StartTimer();
                baseline.GetPath(&pancake, start, goal, &pancake, &pancake2, baselinePath);
                t7.EndTimer();
                printf("GAP-%d NBB found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                       gap, pancake.GetPathLength(baselinePath),
                       baseline.GetNodesExpanded(), baseline.GetNecessaryExpansions(), t7.GetElapsedTime());

                nodes_NBB += baseline.GetNodesExpanded();
                nodes_NBBn += baseline.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(baselinePath);
                else if (optimal_cost != pancake.GetPathLength(baselinePath)) {
                    printf("GAP-%d NBB reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(baselinePath));
                    exit(0);
                }
            }

            // GBFHS eager
            if (1) {
                GBFHS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> gbfhs(true);
                goal.Reset();
                start = original;
                t8.StartTimer();
                gbfhs.GetPath(&pancake, start, goal, &pancake, &pancake2, gbfhsPath);
                t8.EndTimer();
                std::cout << "GAP-" << gap << " GBFHS-eager found path length " << pancake.GetPathLength(gbfhsPath)
                          << "; "
                          << gbfhs.GetNodesExpanded() << " expanded; " << gbfhs.GetNecessaryExpansions()
                          << " necessary; "
                          << t8.GetElapsedTime() << "s elapsed, " << gbfhs.getC() << " C, " << gbfhs.getGLimF()
                          << " gLim_f, "
                          << gbfhs.getGLimB() << " gLim_b, updated forward?: " << gbfhs.getLastUpdatedLimit()
                          << std::endl;

                nodes_GBFHS += gbfhs.GetNodesExpanded();
                nodes_GBFHSn += gbfhs.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(gbfhsPath);
                else if (optimal_cost != pancake.GetPathLength(gbfhsPath)) {
                    printf("GAP-%d GBFHS-eager reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(gbfhsPath));
                    exit(0);
                }

                if (0) { // test all splits

                    int bestExpanded = INT_MAX;
                    int bestNecessary = INT_MAX;

                    // TODO: threshold must be incremented by lcd, not epsilon
                    for (int threshold = 0; threshold <= optimal_cost; threshold += 1) {
                        GBFHS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle <
                                                                            N >> gbfhs_split(true, threshold);
                        goal.Reset();
                        start = original;
                        gbfhsPath.clear();
                        t8.StartTimer();
                        gbfhs_split.GetPath(&pancake, start, goal, &pancake, &pancake2, gbfhsPath);
                        t8.EndTimer();
                        std::cout << "  GAP-" << gap << " GBFHS-" << threshold << " found path length "
                                  << pancake.GetPathLength(gbfhsPath) << "; " << gbfhs_split.GetNodesExpanded()
                                  << " expanded; " << gbfhs_split.GetNecessaryExpansions() << " necessary; "
                                  << t8.GetElapsedTime() << " elapsed, " << gbfhs_split.getGLimF() << " gLim_f, "
                                  << gbfhs_split.getGLimB() << " gLim_b" << std::endl;

                        if (gbfhs_split.GetNecessaryExpansions() < bestNecessary) {
                            bestExpanded = gbfhs_split.GetNodesExpanded();
                            bestNecessary = gbfhs_split.GetNecessaryExpansions();
                        }
                    }

                    nodes_GBFHSbest += bestExpanded;
                    nodes_GBFHSbestn += bestNecessary;
                }
            }

            // GBFHS lazy
            if (1) {
                GBFHS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> gbfhs(false);
                goal.Reset();
                start = original;
                t8.StartTimer();
                gbfhs.GetPath(&pancake, start, goal, &pancake, &pancake2, gbfhsPath);
                t8.EndTimer();
                std::cout << "GAP-" << gap << " GBFHS-lazy found path length " << pancake.GetPathLength(gbfhsPath)
                          << "; "
                          << gbfhs.GetNodesExpanded() << " expanded; " << gbfhs.GetNecessaryExpansions()
                          << " necessary; "
                          << t8.GetElapsedTime() << "s elapsed, " << gbfhs.getC() << " C, " << gbfhs.getGLimF()
                          << " gLim_f, "
                          << gbfhs.getGLimB() << " gLim_b" << std::endl;

                nodes_GBFHSl += gbfhs.GetNodesExpanded();
                nodes_GBFHSln += gbfhs.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(gbfhsPath);
                else if (optimal_cost != pancake.GetPathLength(gbfhsPath)) {
                    printf("GAP-%d GBFHS-lazy reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(gbfhsPath));
                    exit(0);
                }
            }

            // DBS
            if (1) {
                DBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> dbs;
                goal.Reset();
                start = original;
                t8.StartTimer();
                dbs.GetPath(&pancake, start, goal, &pancake, &pancake2, dbsPath);
                t8.EndTimer();
                std::cout << "GAP-" << gap << " DBS found path length " << pancake.GetPathLength(dbsPath) << "; "
                          << dbs.GetNodesExpanded() << " expanded; " << dbs.GetNecessaryExpansions()
                          << " necessary; "
                          << t8.GetElapsedTime() << "s elapsed" << std::endl;

                nodes_DBS += dbs.GetNodesExpanded();
                nodes_DBSn += dbs.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(dbsPath);
                else if (optimal_cost != pancake.GetPathLength(dbsPath)) {
                    printf("GAP-%d DBS reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(dbsPath));
                    exit(0);
                }
            }

            // A*
            if (0) {
                TemplateAStar <PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar(false, 1.0);
                start = original;
                t1.StartTimer();
                astar.GetPath(&pancake, start, goal, astarPath);
                t1.EndTimer();
                printf("GAP-%d A*-E found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                       pancake.GetPathLength(astarPath),
                       astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
                astarOpenClose = astar.openClosedList.elements;

                nodes_Astar += astar.GetNodesExpanded();
                nodes_Astarn += astar.GetNecessaryExpansions();
            }

            //ALL Solution
            if (0) {
                // NBS


                {
                    NBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, NBSQueue<
                            PancakePuzzleState < N>, 0, false >> nbs(true);
                    goal.Reset();
                    start = original;
                    t2.StartTimer();
                    nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
                    t2.EndTimer();
                    printf("GAP-%d NBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(nbsPath),
                           nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime(),
                           nbs.getForwardMeetingPoint(), nbs.getBackwardMeetingPoint(),
                           nbs.getForwardUnnecessaryNodesInPath(), nbs.getBackwardUnnecessaryNodesInPath(),
                           nbs.GetExpansionUntilFirstSolution());

                    NBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, NBSQueue<
                            PancakePuzzleState < N>, 0, true >> nbsLEQ(true, true);
                    goal.Reset();
                    start = original;
                    t2.StartTimer();
                    nbsLEQ.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
                    t2.EndTimer();
                    printf("GAP-%d NBS-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(nbsPath),
                           nbsLEQ.GetNodesExpanded(), nbsLEQ.GetNecessaryExpansions(), t2.GetElapsedTime(),
                           nbsLEQ.getForwardMeetingPoint(), nbsLEQ.getBackwardMeetingPoint(),
                           nbsLEQ.getForwardUnnecessaryNodesInPath(), nbsLEQ.getBackwardUnnecessaryNodesInPath(),
                           nbsLEQ.GetExpansionUntilFirstSolution());
                }
                {
                    NBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, NBSQueue<
                            PancakePuzzleState < N>, 1, false >> nbsEpsilon(true);
                    goal.Reset();
                    start = original;
                    t2.StartTimer();
                    nbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsEpsilonPath);
                    t2.EndTimer();
                    printf("GAP-%d NBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(nbsEpsilonPath),
                           nbsEpsilon.GetNodesExpanded(), nbsEpsilon.GetNecessaryExpansions(), t2.GetElapsedTime(),
                           nbsEpsilon.getForwardMeetingPoint(), nbsEpsilon.getBackwardMeetingPoint(),
                           nbsEpsilon.getForwardUnnecessaryNodesInPath(),
                           nbsEpsilon.getBackwardUnnecessaryNodesInPath(), nbsEpsilon.GetExpansionUntilFirstSolution());

                    NBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, NBSQueue<
                            PancakePuzzleState < N>, 1, true >> nbsEpsilonLeq(true, true);
                    goal.Reset();
                    start = original;
                    t2.StartTimer();
                    nbsEpsilonLeq.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsEpsilonPath);
                    t2.EndTimer();
                    printf("GAP-%d NBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(nbsEpsilonPath),
                           nbsEpsilonLeq.GetNodesExpanded(), nbsEpsilonLeq.GetNecessaryExpansions(),
                           t2.GetElapsedTime(), nbsEpsilonLeq.getForwardMeetingPoint(),
                           nbsEpsilonLeq.getBackwardMeetingPoint(), nbsEpsilonLeq.getForwardUnnecessaryNodesInPath(),
                           nbsEpsilonLeq.getBackwardUnnecessaryNodesInPath(),
                           nbsEpsilonLeq.GetExpansionUntilFirstSolution());
                }


                // DVCBS
                {
                    DVCBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, DVCBSQueue<
                            PancakePuzzleState < N>, 0, false >> dvcbs(true);
                    goal.Reset();
                    start = original;
                    t6.StartTimer();
                    dvcbs.GetPath(&pancake, start, goal, &pancake, &pancake2, dvcbsPath);
                    t6.EndTimer();
                    printf("GAP-%d DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(dvcbsPath),
                           dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t6.GetElapsedTime(),
                           dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                           dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                           dvcbs.GetExpansionUntilFirstSolution());
                }
                {
                    DVCBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, DVCBSQueue<
                            PancakePuzzleState < N>, 0, true >> dvcbs(true, true);
                    goal.Reset();
                    start = original;
                    t6.StartTimer();
                    dvcbs.GetPath(&pancake, start, goal, &pancake, &pancake2, dvcbsPath);
                    t6.EndTimer();
                    printf("GAP-%d DVCBS-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(dvcbsPath),
                           dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t6.GetElapsedTime(),
                           dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                           dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                           dvcbs.GetExpansionUntilFirstSolution());
                }
                // DVCBS
                {
                    DVCBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, DVCBSQueue<
                            PancakePuzzleState < N>, 1, false >> dvcbsEpsilon(true);
                    goal.Reset();
                    start = original;
                    t6.StartTimer();
                    dvcbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, dvcbsEpsilonPath);
                    t6.EndTimer();
                    printf("GAP-%d DVCBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(dvcbsEpsilonPath),
                           dvcbsEpsilon.GetNodesExpanded(), dvcbsEpsilon.GetNecessaryExpansions(), t6.GetElapsedTime(),
                           dvcbsEpsilon.getForwardMeetingPoint(), dvcbsEpsilon.getBackwardMeetingPoint(),
                           dvcbsEpsilon.getForwardUnnecessaryNodesInPath(),
                           dvcbsEpsilon.getBackwardUnnecessaryNodesInPath(),
                           dvcbsEpsilon.GetExpansionUntilFirstSolution());
                }
                {
                    DVCBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, DVCBSQueue<
                            PancakePuzzleState < N>, 1, true >> dvcbsEpsilon(true, true);
                    goal.Reset();
                    start = original;
                    t6.StartTimer();
                    dvcbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, dvcbsEpsilonPath);
                    t6.EndTimer();
                    printf("GAP-%d DVCBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                           gap, pancake.GetPathLength(dvcbsEpsilonPath),
                           dvcbsEpsilon.GetNodesExpanded(), dvcbsEpsilon.GetNecessaryExpansions(), t6.GetElapsedTime(),
                           dvcbsEpsilon.getForwardMeetingPoint(), dvcbsEpsilon.getBackwardMeetingPoint(),
                           dvcbsEpsilon.getForwardUnnecessaryNodesInPath(),
                           dvcbsEpsilon.getBackwardUnnecessaryNodesInPath(),
                           dvcbsEpsilon.GetExpansionUntilFirstSolution());


                }
            }
            //fMM
            if (0) {
                TemplateAStar <PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar(true);
                TemplateAStar <PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> rastar(true);
                astar.SetHeuristic(&pancake);
                //timer.StartTimer();
                goal.Reset();
                start = original;
                astar.GetPath(&pancake, start, goal, astarPath);
                int C = pancake.GetPathLength(astarPath);
                CalculateWVC<PancakePuzzleState < N>>
                calculateWVC;
                std::map<int, int> gCountMapForwardSingle = calculateWVC.initGCountMap(astar.openClosedList.elements,
                                                                                       C);
                std::map<int, int> gCountMapForwardAll = calculateWVC.initGCountMap(astar.openClosedList.elements,
                                                                                    C + 1);
                goal.Reset();
                start = original;
                rastar.SetHeuristic(&pancake2);
                //timer.StartTimer();
                rastar.GetPath(&pancake, goal, start, rastarPath);

                std::map<int, int> gCountMapBackwardSingle = calculateWVC.initGCountMap(rastar.openClosedList.elements,
                                                                                        C);
                std::map<int, int> gCountMapBackwardAll = calculateWVC.initGCountMap(rastar.openClosedList.elements,
                                                                                     C + 1);

                printf("Optimal-Necessary-L %f\n",
                       calculateWVC.CalcWVC(gCountMapForwardSingle, gCountMapBackwardSingle, C, 0, false));
                printf("Optimal-Necessary-E-L %f\n",
                       calculateWVC.CalcWVC(gCountMapForwardSingle, gCountMapBackwardSingle, C, 1, false));
                printf("Optimal-Necessary-LEQ %f\n",
                       calculateWVC.CalcWVC(gCountMapForwardAll, gCountMapBackwardAll, C, 0, true));
                printf("Optimal-Necessary-E-LEQ %f\n",
                       calculateWVC.CalcWVC(gCountMapForwardAll, gCountMapBackwardAll, C, 1, true));

            }


            // BS*
            if (0) {
                BSStar<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> bs;
                goal.Reset();
                start = original;
                t2.StartTimer();
                bs.GetPath(&pancake, start, goal, &pancake, &pancake2, bsPath);
                t2.EndTimer();
                printf("GAP-%d BS* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                       pancake.GetPathLength(bsPath),
                       bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), t2.GetElapsedTime());
            }

            // IDA*
            if (0) {
                IDAStar<PancakePuzzleState<N>, PancakePuzzleAction, false> idastar;
                goal.Reset();
                start = original;
                t3.StartTimer();
                idastar.GetPath(&pancake, start, goal, idaPath);
                t3.EndTimer();
                printf("GAP-%d IDA* found path length %ld; %llu expanded; %llu generated; %1.2fs elapsed\n", gap,
                       idaPath.size(),
                       idastar.GetNodesExpanded(), idastar.GetNodesTouched(), t3.GetElapsedTime());
            }

            // MM
            if (0) {
                MM <PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> mm;
                goal.Reset();
                start = original;
                t4.StartTimer();
                mm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
                t4.EndTimer();
                printf("GAP-%d MM found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                       pancake.GetPathLength(mmPath),
                       mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t4.GetElapsedTime());
            }
            // MM0
            if (0) {
                if (gap == 3) {
                    MM <PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> mm;
                    ZeroHeuristic <PancakePuzzleState<N>> z;
                    goal.Reset();
                    start = original;
                    t4.StartTimer();
                    mm.GetPath(&pancake, start, goal, &z, &z, mmPath);
                    t4.EndTimer();
                    printf("GAP-%d MM0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count,
                           pancake.GetPathLength(mmPath),
                           mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t4.GetElapsedTime());
                }
            }
        }

        printf("+++++++++++++++++++++++++++++++++++++++++\n");

        std::cout << " Experiments: " << INSTANCES << std::endl;

        std::cout << " GAP-" << gap << " NBS " << nodes_NBS << " expanded; " << nodes_NBSn
                  << " necessary" << std::endl;
        std::cout << " GAP-" << gap << " NBSa " << nodes_NBSa << " expanded; " << nodes_NBSan
                  << " necessary" << std::endl;
        std::cout << " GAP-" << gap << " DVCBS " << nodes_DVCBS << " expanded; "
                  << nodes_DVCBSn << " necessary" << std::endl;
        std::cout << " GAP-" << gap << " DVCBSa " << nodes_DVCBSa << " expanded; "
                  << nodes_DVCBSan << " necessary" << std::endl;
        std::cout << " GAP-" << gap << " NBB " << nodes_NBB << " expanded; " << nodes_NBBn
                  << " necessary" << std::endl;
        std::cout << " GAP-" << gap << " GBFHS-eager " << nodes_GBFHS << " expanded; "
                  << nodes_GBFHSn << " necessary" << std::endl;
        std::cout << " GAP-" << gap << " GBFHS-lazy " << nodes_GBFHSl << " expanded; "
                  << nodes_GBFHSln << " necessary" << std::endl;
//        std::cout << "GAP-" << gap << " GBFHS best " << nodes_GBFHSbest << " expanded; "
//                  << nodes_GBFHSbestn << " necessary" << std::endl;
        std::cout << " GAP-" << gap << " A* " << nodes_Astar << " expanded; "
                  << nodes_Astarn << " necessary" << std::endl;
        std::cout << " GAP-" << gap << " DBS " << nodes_DBS << " expanded; "
                  << nodes_DBSn << " necessary" << std::endl;

        printf("+++++++++++++++++++++++++++++++++++++++++\n");

    }
}

const int CNT = 28;

void TestPancakeHard() {
    srandom(2017218);
    PancakePuzzleState <CNT> start;
    PancakePuzzleState <CNT> original;
    PancakePuzzleState <CNT> goal;
    PancakePuzzle <CNT> pancake;
    PancakePuzzle <CNT> pancake2;


    std::vector <PancakePuzzleState<CNT>> nbsPath;
    std::vector <PancakePuzzleState<CNT>> dvcbsPath;
    std::vector <PancakePuzzleState<CNT>> bsPath;
    std::vector <PancakePuzzleState<CNT>> astarPath;
    std::vector <PancakePuzzleState<CNT>> mmPath;
    std::vector <PancakePuzzleAction> idaPath;
    Timer t1, t2, t3, t4, t5;

    for (int count = 2; count < 100; count++) {
        goal.Reset();
        original.Reset();
        GetPancakeInstance(original, count);

        printf("Problem %d of %d\n", count + 1, 100);
        std::cout << original << "; Initial heuristic " << pancake.HCost(original, goal) << "\n";

        // A*
        if (0) {
            TemplateAStar <PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> astar;
            start = original;
            t1.StartTimer();
            astar.GetPath(&pancake, start, goal, astarPath);
            t1.EndTimer();
            printf("HARD-%d A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count,
                   pancake.GetPathLength(astarPath),
                   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
//			std::unordered_map<int, bool> m;
//			for (int x = 0; x < astar.GetNumItems(); x++)
//			{
//				auto &i = astar.GetItem(x);
//				if (i.where != kClosedList)
//					continue;
//				int tmp = (((int)i.g)<<10)|(int)i.h;
//				if (m.find(tmp) == m.end())
//				{
//					m[tmp] = true;
//					printf("(%d, %d)\n", (int)i.g, (int)i.h);
//				}
//			}
        }

        // Reverse A*
        if (0) {
            TemplateAStar <PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> astar;
            goal.Reset();
            start = original;
            t1.StartTimer();
            astar.GetPath(&pancake, goal, start, astarPath);
            t1.EndTimer();
            printf("HARD-%d ReverseA* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count,
                   pancake.GetPathLength(astarPath),
                   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
//			std::unordered_map<int, bool> m;
//			for (int x = 0; x < astar.GetNumItems(); x++)
//			{
//				auto &i = astar.GetItem(x);
//				if (i.where != kClosedList)
//					continue;
//				int tmp = (((int)i.g)<<10)|(int)i.h;
//				if (m.find(tmp) == m.end())
//				{
//					m[tmp] = true;
//					printf("(%d, %d)\n", (int)i.g, (int)i.h);
//				}
//			}
        }

        // Find minimum
        /*
        if (0)
        {
            std::string t = "/Users/nathanst/pancake_";
            t += std::to_string(count);
            t += ".svg";

            start = original;
            GetWeightedVertexGraph<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>(start, goal, &pancake, &pancake, &pancake);
        }
        */
        // DVCBS
        if (1) {
            for (int i = 1; i <= 15; i++) {
                if (i != 4) {
                    continue;
                }
                DVCBS<PancakePuzzleState < CNT>, PancakePuzzleAction, PancakePuzzle < CNT >> dvcbs(i);
                goal.Reset();
                start = original;
                t2.StartTimer();
                dvcbs.GetPath(&pancake, start, goal, &pancake, &pancake2, dvcbsPath);
                t2.EndTimer();
                printf("HARD-%d DVCBS %d found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; \n",
                       count, i, pancake.GetPathLength(dvcbsPath),
                       dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t2.GetElapsedTime());
            }

        }
        // NBS
        if (1) {
            NBS<PancakePuzzleState < CNT>, PancakePuzzleAction, PancakePuzzle < CNT >> nbs;
            goal.Reset();
            start = original;
            t2.StartTimer();
            nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
            t2.EndTimer();
            printf("HARD-%d NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; \n", count,
                   pancake.GetPathLength(nbsPath),
                   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime());
        }

        // NBS0
        if (0) {
            ZeroHeuristic <PancakePuzzleState<CNT>> z;
            NBS<PancakePuzzleState < CNT>, PancakePuzzleAction, PancakePuzzle < CNT >> nbs;
            goal.Reset();
            start = original;
            t2.StartTimer();
            nbs.GetPath(&pancake, start, goal, &z, &z, nbsPath);
            t2.EndTimer();
            printf("HARD-%d NBS0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed;\n", count,
                   pancake.GetPathLength(nbsPath),
                   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime());
        }

        // BS*
        if (0) {
            BSStar<PancakePuzzleState < CNT>, PancakePuzzleAction, PancakePuzzle < CNT >> bs;
            goal.Reset();
            start = original;
            t2.StartTimer();
            bs.GetPath(&pancake, start, goal, &pancake, &pancake2, bsPath);
            t2.EndTimer();
            printf("HARD-%d BS* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count,
                   pancake.GetPathLength(bsPath),
                   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), t2.GetElapsedTime());
        }

        // IDA*
        if (0) {
            IDAStar<PancakePuzzleState<CNT>, PancakePuzzleAction, false> idastar;
            goal.Reset();
            start = original;
            t3.StartTimer();
            idastar.GetPath(&pancake, start, goal, idaPath);
            t3.EndTimer();
            printf("HARD-%d IDA* found path length %ld; %llu expanded; %llu generated; %1.2fs elapsed\n", count,
                   idaPath.size(),
                   idastar.GetNodesExpanded(), idastar.GetNodesTouched(), t3.GetElapsedTime());
        }

        // MM
        if (0) {
            MM <PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> mm;
            goal.Reset();
            start = original;
            t4.StartTimer();
            mm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
            t4.EndTimer();
            printf("HARD-%d MM found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count,
                   pancake.GetPathLength(mmPath),
                   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t1.GetElapsedTime());
        }

        // MM0
        if (0) {
            MM <PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> mm;
            ZeroHeuristic <PancakePuzzleState<CNT>> z;
            goal.Reset();
            start = original;
            t4.StartTimer();
            mm.GetPath(&pancake, start, goal, &z, &z, mmPath);
            t4.EndTimer();
            printf("HARD-%d MM0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count,
                   pancake.GetPathLength(mmPath),
                   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t1.GetElapsedTime());
        }
    }
}

