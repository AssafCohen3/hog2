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
#include "DBBS.h"
#include "BTB.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "BAE.h"
#include "DVCBS.h"
#include "PancakeInstances.h"
#include "CalculateWVC.h"
#include "fMM.h"

const int N = 14;
const int INSTANCES = 50;

void TestPancakeRandom() {
    for (int gap = 0; gap < 7; gap++) {
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
        std::vector <PancakePuzzleState<N>> dbbsPath;
        std::vector <PancakePuzzleState<N>> btbPath;
        std::vector <PancakePuzzleState<N>> btbSmallestPath;
        std::vector <PancakePuzzleState<N>> btbConnectedPath;
        std::vector <PancakePuzzleState<N>> dvcbsEpsilonPath;
        std::vector <PancakePuzzleState<N>> bsPath;
        std::vector <PancakePuzzleState<N>> baePath;
        std::vector <PancakePuzzleState<N>> astarPath;
        std::vector <PancakePuzzleState<N>> rastarPath;
        std::vector <PancakePuzzleState<N>> mmPath;
        std::vector <PancakePuzzleState<N>> fmmPath;
        std::vector <PancakePuzzleAction> idaPath;
        Timer t1, t2, t3, t4, t5, t6, t7, t8;

        // variables for WVC calaculator (optimal p for FMM)
        std::vector < AStarOpenClosedData < PancakePuzzleState < N >> > astarOpenClose;
        std::vector < AStarOpenClosedData < PancakePuzzleState < N >> > rastarOpenClose;

        long nodes_Astar = 0, nodes_Astarn = 0, notie_Astar = 0,
                nodes_BSstar = 0, nodes_BSstarn = 0, notie_BSstar = 0, nodes_BSstara = 0, nodes_BSstaran = 0, notie_BSstara = 0,
                nodes_NBS = 0, nodes_NBSn = 0, notie_NBS = 0, nodes_NBSa = 0, nodes_NBSan = 0, notie_NBSa = 0,
                nodes_DVCBS = 0, nodes_DVCBSn = 0, notie_DVCBS = 0, nodes_DVCBSa = 0, nodes_DVCBSan = 0, notie_DVCBSa = 0,
                nodes_NBB = 0, nodes_NBBn = 0, notie_NBB = 0,
                nodes_GBFHS = 0, nodes_GBFHSn = 0, notie_GBFHS = 0, nodes_GBFHSl = 0, nodes_GBFHSln = 0, notie_GBFHSl = 0,
                nodes_GBFHSbest = 0, nodes_GBFHSbestn = 0, notie_GBFHSbest = 0,
                nodes_BAE = 0, nodes_BAEn = 0, notie_BAE = 0, nodes_BAEp = 0, nodes_BAEpn = 0, notie_BAEp = 0,
                nodes_DBS = 0, nodes_DBSn = 0, notie_DBS = 0, nodes_DBSp = 0, nodes_DBSpn = 0, notie_DBSp = 0,
                nodes_BTB = 0, nodes_BTBn = 0, notie_BTB = 0,
                nodes_BTB_small = 0, nodes_BTB_smalln = 0, notie_BTB_small = 0,
                nodes_BTB_conn = 0, nodes_BTB_connn = 0, notie_BTB_conn = 0,
                nodes_DBBS = 0, nodes_DBBSn = 0, notie_DBBS = 0, nodes_DBBSp = 0, nodes_DBBSpn = 0, notie_DBBSp = 0;

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
            if (0) {
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
                    if (nbsEpsilon.GetNodesExpanded() == nbsEpsilon.GetNecessaryExpansions()) notie_NBS++;

                    // test optimality
                    if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(nbsEpsilonPath);
                    else if (optimal_cost != pancake.GetPathLength(nbsEpsilonPath)) {
                        printf("GAP-%d NBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                               optimal_cost, pancake.GetPathLength(nbsEpsilonPath));
                        exit(0);
                    }

                    if (optimal_cost < 3.0) continue;

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
                    if (nbsEpsilonLeq.GetNodesExpanded() == nbsEpsilonLeq.GetNecessaryExpansions()) notie_NBSa++;

                    // test optimality
                    if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(nbsEpsilonPath);
                    else if (optimal_cost != pancake.GetPathLength(nbsEpsilonPath)) {
                        printf("GAP-%d NBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                               optimal_cost, pancake.GetPathLength(nbsEpsilonPath));
                        exit(0);
                    }
                }

                // DVCBS
                if (0) {
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
                    if (dvcbsEpsilon.GetNodesExpanded() == dvcbsEpsilon.GetNecessaryExpansions()) notie_DVCBS++;

                    // test optimality
                    if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(dvcbsEpsilonPath);
                    else if (optimal_cost != pancake.GetPathLength(dvcbsEpsilonPath)) {
                        printf("GAP-%d DVCBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                               optimal_cost, pancake.GetPathLength(dvcbsEpsilonPath));
                        exit(0);
                    }
                }
                if (0) {
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
                    if (dvcbsEpsilon.GetNodesExpanded() == dvcbsEpsilon.GetNecessaryExpansions()) notie_DVCBSa++;

                    // test optimality
                    if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(dvcbsEpsilonPath);
                    else if (optimal_cost != pancake.GetPathLength(dvcbsEpsilonPath)) {
                        printf("GAP-%d DVCBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                               optimal_cost, pancake.GetPathLength(dvcbsEpsilonPath));
                        exit(0);
                    }
                }
            }

            // NBB
            if (0) {
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
                if (baseline.GetNodesExpanded() == baseline.GetNecessaryExpansions()) notie_NBB++;

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(baselinePath);
                else if (optimal_cost != pancake.GetPathLength(baselinePath)) {
                    printf("GAP-%d NBB reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(baselinePath));
                    exit(0);
                }
            }

            // GBFHS eager
            if (0) {
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
                if (gbfhs.GetNodesExpanded() == gbfhs.GetNecessaryExpansions()) notie_GBFHS++;

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
                                                                            N >> gbfhs_split(true, 1.0, 1.0, threshold);
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
            if (0) {
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
                if (gbfhs.GetNodesExpanded() == gbfhs.GetNecessaryExpansions()) notie_GBFHSl++;

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(gbfhsPath);
                else if (optimal_cost != pancake.GetPathLength(gbfhsPath)) {
                    printf("GAP-%d GBFHS-lazy reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(gbfhsPath));
                    exit(0);
                }
            }

            // DBGS
            if (1) {
                DBBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, true >
                                                                                        dbs(MinCriterion::MinG, true);
                goal.Reset();
                start = original;
                t8.StartTimer();
                dbs.GetPath(&pancake, start, goal, &pancake, &pancake2, dbsPath);
                t8.EndTimer();
                std::cout << "GAP-" << gap << " DBGS found path length " << pancake.GetPathLength(dbsPath) << "; "
                          << dbs.GetNodesExpanded() << " expanded; " << dbs.GetNecessaryExpansions()
                          << " necessary; "
                          << t8.GetElapsedTime() << "s elapsed" << std::endl;

                nodes_DBS += dbs.GetNodesExpanded();
                nodes_DBSn += dbs.GetNecessaryExpansions();
                if (dbs.GetNodesExpanded() == dbs.GetNecessaryExpansions()) notie_DBS++;

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(dbsPath);
                else if (optimal_cost != pancake.GetPathLength(dbsPath)) {
                    printf("GAP-%d DBGS reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(dbsPath));
                    exit(0);
                }
            }

            // DBS-p
            if (1) {
                DBBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >, true >
                                                                                        dbs(MinCriterion::MinG, false);
                goal.Reset();
                start = original;
                t8.StartTimer();
                dbs.GetPath(&pancake, start, goal, &pancake, &pancake2, dbsPath);
                t8.EndTimer();
                std::cout << "GAP-" << gap << " DBGS-p found path length " << pancake.GetPathLength(dbsPath) << "; "
                          << dbs.GetNodesExpanded() << " expanded; " << dbs.GetNecessaryExpansions()
                          << " necessary; "
                          << t8.GetElapsedTime() << "s elapsed" << std::endl;

                nodes_DBSp += dbs.GetNodesExpanded();
                nodes_DBSpn += dbs.GetNecessaryExpansions();
                if (dbs.GetNodesExpanded() == dbs.GetNecessaryExpansions()) notie_DBSp++;

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(dbsPath);
                else if (optimal_cost != pancake.GetPathLength(dbsPath)) {
                    printf("GAP-%d DBGS-p reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(dbsPath));
                    exit(0);
                }
            }

            int tempDBBS;

            // DBBS
            if (1) {
                DBBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> dbbs(MinCriterion::MinB, true);
                goal.Reset();
                start = original;
                t8.StartTimer();
                dbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, dbbsPath);
                t8.EndTimer();
                std::cout << "GAP-" << gap << " DBBS found path length " << pancake.GetPathLength(dbbsPath) << "; "
                          << dbbs.GetNodesExpanded() << " expanded; " << dbbs.GetNecessaryExpansions()
                          << " necessary; "
                          << t8.GetElapsedTime() << "s elapsed" << std::endl;

                nodes_DBBS += dbbs.GetNodesExpanded();
                nodes_DBBSn += dbbs.GetNecessaryExpansions();
                if (dbbs.GetNodesExpanded() == dbbs.GetNecessaryExpansions()) notie_DBBS++;

                tempDBBS = dbbs.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(dbbsPath);
                else if (optimal_cost != pancake.GetPathLength(dbbsPath)) {
                    printf("GAP-%d DBBS reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(dbbsPath));
                    exit(0);
                }
            }

            // DBBS-p
            if (1) {
                DBBS<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> dbbs(MinCriterion::MinB, false);
                goal.Reset();
                start = original;
                t8.StartTimer();
                dbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, dbbsPath);
                t8.EndTimer();
                std::cout << "GAP-" << gap << " DBBS-p found path length " << pancake.GetPathLength(dbbsPath) << "; "
                          << dbbs.GetNodesExpanded() << " expanded; " << dbbs.GetNecessaryExpansions()
                          << " necessary; "
                          << t8.GetElapsedTime() << "s elapsed" << std::endl;

                nodes_DBBSp += dbbs.GetNodesExpanded();
                nodes_DBBSpn += dbbs.GetNecessaryExpansions();
                if (dbbs.GetNodesExpanded() == dbbs.GetNecessaryExpansions()) notie_DBBSp++;

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(dbbsPath);
                else if (optimal_cost != pancake.GetPathLength(dbbsPath)) {
                    printf("GAP-%d DBBS-p reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(dbbsPath));
                    exit(0);
                }
            }

            // BS*
            if (0) {
                BSStar<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> bs;
                start = original;
                t1.StartTimer();
                bs.GetPath(&pancake, start, goal, &pancake, &pancake2, bsPath);
                t1.EndTimer();
                printf("GAP-%d BS* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                       pancake.GetPathLength(bsPath),
                       bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), t1.GetElapsedTime());

                nodes_BSstar += bs.GetNodesExpanded();
                nodes_BSstarn += bs.GetNecessaryExpansions();
                if (bs.GetNodesExpanded() == bs.GetNecessaryExpansions()) notie_BSstar++;

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(bsPath);
                else if (optimal_cost != pancake.GetPathLength(bsPath)) {
                    printf("GAP-%d BS* reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(bsPath));
                    exit(0);
                }
            }

            // BS*-a
            if (0) {
                BSStar<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> bs(true);
                start = original;
                t1.StartTimer();
                bs.GetPath(&pancake, start, goal, &pancake, &pancake2, bsPath);
                t1.EndTimer();
                printf("GAP-%d BS*-a found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                       pancake.GetPathLength(bsPath),
                       bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), t1.GetElapsedTime());

                nodes_BSstara += bs.GetNodesExpanded();
                nodes_BSstaran += bs.GetNecessaryExpansions();
                if (bs.GetNodesExpanded() == bs.GetNecessaryExpansions()) notie_BSstara++;

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(bsPath);
                else if (optimal_cost != pancake.GetPathLength(bsPath)) {
                    printf("GAP-%d BS*-a reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(bsPath));
                    exit(0);
                }
            }

            int tempBTBalt;

            // BTB alternating
            if (1) {
                BTB<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> btb(BTBPolicy::Alternating);
                goal.Reset();
                start = original;
                t8.StartTimer();
                btb.GetPath(&pancake, start, goal, &pancake, &pancake2, btbPath);
                t8.EndTimer();
                std::cout << "GAP-" << gap << " BTB alt found path length " << pancake.GetPathLength(btbPath) << "; "
                          << btb.GetNodesExpanded() << " expanded; " << btb.GetNecessaryExpansions()
                          << " necessary; "
                          << t8.GetElapsedTime() << "s elapsed" << std::endl;

                nodes_BTB += btb.GetNodesExpanded();
                nodes_BTBn += btb.GetNecessaryExpansions();
                if (btb.GetNodesExpanded() == btb.GetNecessaryExpansions()) notie_BTB++;

                tempBTBalt = btb.GetNecessaryExpansions();

                if (2 * tempDBBS + 2 < tempBTBalt) {
                    std::cout << "NOT NEAR-OPTIMAL!!!! " << tempDBBS << " and " << tempBTBalt << std::endl;
                }

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(btbPath);
                else if (optimal_cost != pancake.GetPathLength(btbPath)) {
                    printf("GAP-%d BTB reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(btbPath));
                    exit(0);
                }
            }

            // BTB smallest bucket
            if (1) {
                BTB<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> btb(BTBPolicy::Smallest);
                goal.Reset();
                start = original;
                t8.StartTimer();
                btb.GetPath(&pancake, start, goal, &pancake, &pancake2, btbSmallestPath);
                t8.EndTimer();
                std::cout << "GAP-" << gap << " BTB small found path length " << pancake.GetPathLength(btbSmallestPath)
                          << "; "
                          << btb.GetNodesExpanded() << " expanded; " << btb.GetNecessaryExpansions()
                          << " necessary; "
                          << t8.GetElapsedTime() << "s elapsed" << std::endl;

                nodes_BTB_small += btb.GetNodesExpanded();
                nodes_BTB_smalln += btb.GetNecessaryExpansions();
                if (btb.GetNodesExpanded() == btb.GetNecessaryExpansions()) notie_BTB_small++;

                if (2 * btb.GetNecessaryExpansions() + 2 < tempBTBalt) {
                    std::cout << "NOT NEAR-OPTIMAL!!!! " << btb.GetNecessaryExpansions()
                              << " and " << tempBTBalt << std::endl;
                }

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(btbSmallestPath);
                else if (optimal_cost != pancake.GetPathLength(btbSmallestPath)) {
                    printf("GAP-%d BTB small reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(btbSmallestPath));
                    exit(0);
                }
            }

            // BTB most connected bucket
            if (1) {
                BTB<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> btb(BTBPolicy::MostConnected);
                goal.Reset();
                start = original;
                t8.StartTimer();
                btb.GetPath(&pancake, start, goal, &pancake, &pancake2, btbConnectedPath);
                t8.EndTimer();
                std::cout << "GAP-" << gap << " BTB conn found path length " << pancake.GetPathLength(btbConnectedPath)
                          << "; "
                          << btb.GetNodesExpanded() << " expanded; " << btb.GetNecessaryExpansions()
                          << " necessary; "
                          << t8.GetElapsedTime() << "s elapsed" << std::endl;

                nodes_BTB_conn += btb.GetNodesExpanded();
                nodes_BTB_connn += btb.GetNecessaryExpansions();
                if (btb.GetNodesExpanded() == btb.GetNecessaryExpansions()) notie_BTB_conn++;

                if (2 * btb.GetNecessaryExpansions() + 2 < tempBTBalt) {
                    std::cout << "NOT NEAR-OPTIMAL!!!! " << btb.GetNecessaryExpansions()
                              << " and " << tempBTBalt << std::endl;
                }

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(btbConnectedPath);
                else if (optimal_cost != pancake.GetPathLength(btbConnectedPath)) {
                    printf("GAP-%d BTB conn reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(btbConnectedPath));
                    exit(0);
                }
            }

            // BAE*
            if (1) {
                BAE<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> bae;
                start = original;
                t1.StartTimer();
                bae.GetPath(&pancake, start, goal, &pancake, &pancake2, baePath);
                t1.EndTimer();
                printf("GAP-%d BAE* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                       pancake.GetPathLength(baePath),
                       bae.GetNodesExpanded(), bae.GetNecessaryExpansions(), t1.GetElapsedTime());

                nodes_BAE += bae.GetNodesExpanded();
                nodes_BAEn += bae.GetNecessaryExpansions();

                if (bae.GetNodesExpanded() == bae.GetNecessaryExpansions()) notie_BAE++;

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(baePath);
                else if (optimal_cost != pancake.GetPathLength(baePath)) {
                    printf("GAP-%d BAE* reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(baePath));
                    exit(0);
                }
            }

            // BAE*-p
            if (1) {
                BAE<PancakePuzzleState < N>, PancakePuzzleAction, PancakePuzzle < N >> bae(false);
                start = original;
                t1.StartTimer();
                bae.GetPath(&pancake, start, goal, &pancake, &pancake2, baePath);
                t1.EndTimer();
                printf("GAP-%d BAE*-p found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap,
                       pancake.GetPathLength(baePath),
                       bae.GetNodesExpanded(), bae.GetNecessaryExpansions(), t1.GetElapsedTime());

                nodes_BAEp += bae.GetNodesExpanded();
                nodes_BAEpn += bae.GetNecessaryExpansions();
                if (bae.GetNodesExpanded() == bae.GetNecessaryExpansions()) notie_BAEp++;

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(baePath);
                else if (optimal_cost != pancake.GetPathLength(baePath)) {
                    printf("GAP-%d BAE*-p reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(baePath));
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

                nodes_Astar += astar.GetNodesExpanded();
                nodes_Astarn += astar.GetNecessaryExpansions();
                if (astar.GetNodesExpanded() == astar.GetNecessaryExpansions()) notie_Astar++;

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(astarPath);
                else if (optimal_cost != pancake.GetPathLength(astarPath)) {
                    printf("GAP-%d BS* reported bad value!! optimal %1.0f; reported %1.0f;\n", gap,
                           optimal_cost, pancake.GetPathLength(astarPath));
                    exit(0);
                }
            }

        }

        printf("+++++++++++++++++++++++++++++++++++++++++\n");

        std::cout << " Experiments: " << INSTANCES << std::endl;

        std::cout << "Pancake" << " NBS " << nodes_NBS / INSTANCES << " expanded; "
                  << nodes_NBSn / INSTANCES << " necessary; "
                  << notie_NBS / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " NBSa " << nodes_NBSa / INSTANCES << " expanded; "
                  << nodes_NBSan / INSTANCES << " necessary; "
                  << notie_NBSa / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " DVCBS " << nodes_DVCBS / INSTANCES << " expanded; "
                  << nodes_DVCBSn / INSTANCES << " necessary; "
                  << notie_DVCBS / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " DVCBSa " << nodes_DVCBSa / INSTANCES << " expanded; "
                  << nodes_DVCBSan / INSTANCES << " necessary; "
                  << notie_DVCBSa / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " NBB " << nodes_NBB / INSTANCES << " expanded; "
                  << nodes_NBBn / INSTANCES << " necessary; "
                  << notie_NBB / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " GBFHS-eager " << nodes_GBFHS / INSTANCES << " expanded; "
                  << nodes_GBFHSn / INSTANCES << " necessary; "
                  << notie_GBFHS / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " GBFHS-lazy " << nodes_GBFHSl / INSTANCES << " expanded; "
                  << nodes_GBFHSln / INSTANCES << " necessary; "
                  << notie_GBFHSl / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " A* " << nodes_Astar / INSTANCES << " expanded; "
                  << nodes_Astarn / INSTANCES << " necessary; "
                  << notie_Astar / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " BS* " << nodes_BSstar / INSTANCES << " expanded; "
                  << nodes_BSstarn / INSTANCES << " necessary; "
                  << notie_BSstar / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " BS*-a " << nodes_BSstara / INSTANCES << " expanded; "
                  << nodes_BSstaran / INSTANCES << " necessary; "
                  << notie_BSstara / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " BAE* " << nodes_BAE / INSTANCES << " expanded; "
                  << nodes_BAEn / INSTANCES << " necessary; "
                  << notie_BAE / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " BAE*-p " << nodes_BAEp / INSTANCES << " expanded; "
                  << nodes_BAEpn / INSTANCES << " necessary; "
                  << notie_BAEp / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " DBGS " << nodes_DBS / INSTANCES << " expanded; "
                  << nodes_DBSn / INSTANCES << " necessary; "
                  << notie_DBS / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " DBGS-p " << nodes_DBSp / INSTANCES << " expanded; "
                  << nodes_DBSpn / INSTANCES << " necessary; "
                  << notie_DBSp / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " DBBS " << nodes_DBBS / INSTANCES << " expanded; "
                  << nodes_DBBSn / INSTANCES << " necessary; "
                  << notie_DBBS / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " DBBS-p " << nodes_DBBSp / INSTANCES << " expanded; "
                  << nodes_DBBSpn / INSTANCES << " necessary; "
                  << notie_DBBSp / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " BTB alt " << nodes_BTB / INSTANCES << " expanded; "
                  << nodes_BTBn / INSTANCES << " necessary; "
                  << notie_BTB / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " BTB small " << nodes_BTB_small / INSTANCES << " expanded; "
                  << nodes_BTB_smalln / INSTANCES << " necessary; "
                  << notie_BTB_small / (float) INSTANCES << " no last layer" << std::endl;
        std::cout << "Pancake" << " BTB connected " << nodes_BTB_conn / INSTANCES << " expanded; "
                  << nodes_BTB_connn / INSTANCES << " necessary; "
                  << notie_BTB_conn / (float) INSTANCES << " no last layer" << std::endl;

        printf("+++++++++++++++++++++++++++++++++++++++++\n");

    }

    exit(0);
}

void TestPancake() {
    TestPancakeRandom();
    exit(0);
}