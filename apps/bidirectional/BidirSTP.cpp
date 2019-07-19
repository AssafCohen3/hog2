//
//  BidirSTP.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/30/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirSTP.h"
#include "MNPuzzle.h"
#include "NBS.h"
#include "DVCBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "TemplateAStar.h"
#include "Baseline.h"
#include "GBFHS.h"
#include "DBS.h"

MNPuzzleState<4, 4> GetKorfInstance(int which) {
    int instances[100][16] =
            {{14, 13, 15, 7,  11, 12, 9,  5,  6,  0,  2,  1,  4,  8,  10, 3},
             {13, 5,  4,  10, 9,  12, 8,  14, 2,  3,  7,  1,  0,  15, 11, 6},
             {14, 7,  8,  2,  13, 11, 10, 4,  9,  12, 5,  0,  3,  6,  1,  15},
             {5,  12, 10, 7,  15, 11, 14, 0,  8,  2,  1,  13, 3,  4,  9,  6},
             {4,  7,  14, 13, 10, 3,  9,  12, 11, 5,  6,  15, 1,  2,  8,  0},
             {14, 7,  1,  9,  12, 3,  6,  15, 8,  11, 2,  5,  10, 0,  4,  13},
             {2,  11, 15, 5,  13, 4,  6,  7,  12, 8,  10, 1,  9,  3,  14, 0},
             {12, 11, 15, 3,  8,  0,  4,  2,  6,  13, 9,  5,  14, 1,  10, 7},
             {3,  14, 9,  11, 5,  4,  8,  2,  13, 12, 6,  7,  10, 1,  15, 0},
             {13, 11, 8,  9,  0,  15, 7,  10, 4,  3,  6,  14, 5,  12, 2,  1},
             {5,  9,  13, 14, 6,  3,  7,  12, 10, 8,  4,  0,  15, 2,  11, 1},
             {14, 1,  9,  6,  4,  8,  12, 5,  7,  2,  3,  0,  10, 11, 13, 15},
             {3,  6,  5,  2,  10, 0,  15, 14, 1,  4,  13, 12, 9,  8,  11, 7},
             {7,  6,  8,  1,  11, 5,  14, 10, 3,  4,  9,  13, 15, 2,  0,  12},
             {13, 11, 4,  12, 1,  8,  9,  15, 6,  5,  14, 2,  7,  3,  10, 0},
             {1,  3,  2,  5,  10, 9,  15, 6,  8,  14, 13, 11, 12, 4,  7,  0},
             {15, 14, 0,  4,  11, 1,  6,  13, 7,  5,  8,  9,  3,  2,  10, 12},
             {6,  0,  14, 12, 1,  15, 9,  10, 11, 4,  7,  2,  8,  3,  5,  13},
             {7,  11, 8,  3,  14, 0,  6,  15, 1,  4,  13, 9,  5,  12, 2,  10},
             {6,  12, 11, 3,  13, 7,  9,  15, 2,  14, 8,  10, 4,  1,  5,  0},
             {12, 8,  14, 6,  11, 4,  7,  0,  5,  1,  10, 15, 3,  13, 9,  2},
             {14, 3,  9,  1,  15, 8,  4,  5,  11, 7,  10, 13, 0,  2,  12, 6},
             {10, 9,  3,  11, 0,  13, 2,  14, 5,  6,  4,  7,  8,  15, 1,  12},
             {7,  3,  14, 13, 4,  1,  10, 8,  5,  12, 9,  11, 2,  15, 6,  0},
             {11, 4,  2,  7,  1,  0,  10, 15, 6,  9,  14, 8,  3,  13, 5,  12},
             {5,  7,  3,  12, 15, 13, 14, 8,  0,  10, 9,  6,  1,  4,  2,  11},
             {14, 1,  8,  15, 2,  6,  0,  3,  9,  12, 10, 13, 4,  7,  5,  11},
             {13, 14, 6,  12, 4,  5,  1,  0,  9,  3,  10, 2,  15, 11, 8,  7},
             {9,  8,  0,  2,  15, 1,  4,  14, 3,  10, 7,  5,  11, 13, 6,  12},
             {12, 15, 2,  6,  1,  14, 4,  8,  5,  3,  7,  0,  10, 13, 9,  11},
             {12, 8,  15, 13, 1,  0,  5,  4,  6,  3,  2,  11, 9,  7,  14, 10},
             {14, 10, 9,  4,  13, 6,  5,  8,  2,  12, 7,  0,  1,  3,  11, 15},
             {14, 3,  5,  15, 11, 6,  13, 9,  0,  10, 2,  12, 4,  1,  7,  8},
             {6,  11, 7,  8,  13, 2,  5,  4,  1,  10, 3,  9,  14, 0,  12, 15},
             {1,  6,  12, 14, 3,  2,  15, 8,  4,  5,  13, 9,  0,  7,  11, 10},
             {12, 6,  0,  4,  7,  3,  15, 1,  13, 9,  8,  11, 2,  14, 5,  10},
             {8,  1,  7,  12, 11, 0,  10, 5,  9,  15, 6,  13, 14, 2,  3,  4},
             {7,  15, 8,  2,  13, 6,  3,  12, 11, 0,  4,  10, 9,  5,  1,  14},
             {9,  0,  4,  10, 1,  14, 15, 3,  12, 6,  5,  7,  11, 13, 8,  2},
             {11, 5,  1,  14, 4,  12, 10, 0,  2,  7,  13, 3,  9,  15, 6,  8},
             {8,  13, 10, 9,  11, 3,  15, 6,  0,  1,  2,  14, 12, 5,  4,  7},
             {4,  5,  7,  2,  9,  14, 12, 13, 0,  3,  6,  11, 8,  1,  15, 10},
             {11, 15, 14, 13, 1,  9,  10, 4,  3,  6,  2,  12, 7,  5,  8,  0},
             {12, 9,  0,  6,  8,  3,  5,  14, 2,  4,  11, 7,  10, 1,  15, 13},
             {3,  14, 9,  7,  12, 15, 0,  4,  1,  8,  5,  6,  11, 10, 2,  13},
             {8,  4,  6,  1,  14, 12, 2,  15, 13, 10, 9,  5,  3,  7,  0,  11},
             {6,  10, 1,  14, 15, 8,  3,  5,  13, 0,  2,  7,  4,  9,  11, 12},
             {8,  11, 4,  6,  7,  3,  10, 9,  2,  12, 15, 13, 0,  1,  5,  14},
             {10, 0,  2,  4,  5,  1,  6,  12, 11, 13, 9,  7,  15, 3,  14, 8},
             {12, 5,  13, 11, 2,  10, 0,  9,  7,  8,  4,  3,  14, 6,  15, 1},
             {10, 2,  8,  4,  15, 0,  1,  14, 11, 13, 3,  6,  9,  7,  5,  12},
             {10, 8,  0,  12, 3,  7,  6,  2,  1,  14, 4,  11, 15, 13, 9,  5},
             {14, 9,  12, 13, 15, 4,  8,  10, 0,  2,  1,  7,  3,  11, 5,  6},
             {12, 11, 0,  8,  10, 2,  13, 15, 5,  4,  7,  3,  6,  9,  14, 1},
             {13, 8,  14, 3,  9,  1,  0,  7,  15, 5,  4,  10, 12, 2,  6,  11},
             {3,  15, 2,  5,  11, 6,  4,  7,  12, 9,  1,  0,  13, 14, 10, 8},
             {5,  11, 6,  9,  4,  13, 12, 0,  8,  2,  15, 10, 1,  7,  3,  14},
             {5,  0,  15, 8,  4,  6,  1,  14, 10, 11, 3,  9,  7,  12, 2,  13},
             {15, 14, 6,  7,  10, 1,  0,  11, 12, 8,  4,  9,  2,  5,  13, 3},
             {11, 14, 13, 1,  2,  3,  12, 4,  15, 7,  9,  5,  10, 6,  8,  0},
             {6,  13, 3,  2,  11, 9,  5,  10, 1,  7,  12, 14, 8,  4,  0,  15},
             {4,  6,  12, 0,  14, 2,  9,  13, 11, 8,  3,  15, 7,  10, 1,  5},
             {8,  10, 9,  11, 14, 1,  7,  15, 13, 4,  0,  12, 6,  2,  5,  3},
             {5,  2,  14, 0,  7,  8,  6,  3,  11, 12, 13, 15, 4,  10, 9,  1},
             {7,  8,  3,  2,  10, 12, 4,  6,  11, 13, 5,  15, 0,  1,  9,  14},
             {11, 6,  14, 12, 3,  5,  1,  15, 8,  0,  10, 13, 9,  7,  4,  2},
             {7,  1,  2,  4,  8,  3,  6,  11, 10, 15, 0,  5,  14, 12, 13, 9},
             {7,  3,  1,  13, 12, 10, 5,  2,  8,  0,  6,  11, 14, 15, 4,  9},
             {6,  0,  5,  15, 1,  14, 4,  9,  2,  13, 8,  10, 11, 12, 7,  3},
             {15, 1,  3,  12, 4,  0,  6,  5,  2,  8,  14, 9,  13, 10, 7,  11},
             {5,  7,  0,  11, 12, 1,  9,  10, 15, 6,  2,  3,  8,  4,  13, 14},
             {12, 15, 11, 10, 4,  5,  14, 0,  13, 7,  1,  2,  9,  8,  3,  6},
             {6,  14, 10, 5,  15, 8,  7,  1,  3,  4,  2,  0,  12, 9,  11, 13},
             {14, 13, 4,  11, 15, 8,  6,  9,  0,  7,  3,  1,  2,  10, 12, 5},
             {14, 4,  0,  10, 6,  5,  1,  3,  9,  2,  13, 15, 12, 7,  8,  11},
             {15, 10, 8,  3,  0,  6,  9,  5,  1,  14, 13, 11, 7,  2,  12, 4},
             {0,  13, 2,  4,  12, 14, 6,  9,  15, 1,  10, 3,  11, 5,  8,  7},
             {3,  14, 13, 6,  4,  15, 8,  9,  5,  12, 10, 0,  2,  7,  1,  11},
             {0,  1,  9,  7,  11, 13, 5,  3,  14, 12, 4,  2,  8,  6,  10, 15},
             {11, 0,  15, 8,  13, 12, 3,  5,  10, 1,  4,  6,  14, 9,  7,  2},
             {13, 0,  9,  12, 11, 6,  3,  5,  15, 8,  1,  10, 4,  14, 2,  7},
             {14, 10, 2,  1,  13, 9,  8,  11, 7,  3,  6,  12, 15, 5,  4,  0},
             {12, 3,  9,  1,  4,  5,  10, 2,  6,  11, 15, 0,  14, 7,  13, 8},
             {15, 8,  10, 7,  0,  12, 14, 1,  5,  9,  6,  3,  13, 11, 4,  2},
             {4,  7,  13, 10, 1,  2,  9,  6,  12, 8,  14, 5,  3,  0,  11, 15},
             {6,  0,  5,  10, 11, 12, 9,  2,  1,  7,  4,  3,  14, 8,  13, 15},
             {9,  5,  11, 10, 13, 0,  2,  1,  8,  6,  14, 12, 4,  7,  3,  15},
             {15, 2,  12, 11, 14, 13, 9,  5,  1,  3,  8,  7,  0,  10, 6,  4},
             {11, 1,  7,  4,  10, 13, 3,  8,  9,  14, 0,  15, 6,  5,  2,  12},
             {5,  4,  7,  1,  11, 12, 14, 15, 10, 13, 8,  6,  2,  0,  9,  3},
             {9,  7,  5,  2,  14, 15, 12, 10, 11, 3,  6,  1,  8,  13, 0,  4},
             {3,  2,  7,  9,  0,  15, 12, 4,  6,  11, 5,  14, 8,  13, 10, 1},
             {13, 9,  14, 6,  12, 8,  1,  2,  3,  4,  0,  7,  5,  10, 11, 15},
             {5,  7,  11, 8,  0,  14, 9,  13, 10, 12, 3,  15, 6,  1,  4,  2},
             {4,  3,  6,  13, 7,  15, 9,  0,  10, 5,  8,  11, 2,  12, 1,  14},
             {1,  7,  15, 14, 2,  6,  4,  9,  12, 11, 13, 3,  0,  8,  5,  10},
             {9,  14, 5,  7,  8,  15, 1,  2,  10, 4,  13, 6,  12, 0,  11, 3},
             {0,  11, 3,  12, 5,  2,  1,  9,  8,  10, 14, 15, 7,  4,  13, 6},
             {7,  15, 4,  0,  10, 9,  2,  5,  12, 11, 13, 6,  1,  3,  14, 8},
             {11, 4,  0,  8,  6,  10, 5,  13, 12, 7,  14, 3,  1,  2,  9,  15}};

    MNPuzzleState<4, 4> s;
    for (int x = 0; x < 16; x++) {
        s.puzzle[x] = instances[which][x];
        if (s.puzzle[x] == 0)
            s.blank = x;
    }
    return s;
}

void TestSTP() {
    MM <MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> mm;
    BSStar<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >> bs;
    TemplateAStar <MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
    TemplateAStar <MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> rastar;
    MNPuzzle<4, 4> mnp;

    long nodes_Astar = 0, nodes_Astarn = 0, notie_Astar = 0,
            nodes_NBS = 0, nodes_NBSn = 0, notie_NBS = 0, nodes_NBSa = 0, nodes_NBSan = 0, notie_NBSa = 0,
            nodes_DVCBS = 0, nodes_DVCBSn = 0, notie_DVCBS = 0, nodes_DVCBSa = 0, nodes_DVCBSan = 0, notie_DVCBSa = 0,
            nodes_NBB = 0, nodes_NBBn = 0, notie_NBB = 0,
            nodes_GBFHS = 0, nodes_GBFHSn = 0, notie_GBFHS = 0, nodes_GBFHSl = 0, nodes_GBFHSln = 0, notie_GBFHSl = 0,
            nodes_GBFHSbest = 0, nodes_GBFHSbestn = 0, notie_GBFHSbest = 0,
            nodes_DBS = 0, nodes_DBSn = 0, notie_DBS = 0;

    for (int x = 0; x < 100; x++) // 547 to 540
    {

        MNPuzzleState<4, 4> start, goal;
        printf("Problem %d of %d\n", x + 1, 100);

        std::vector <MNPuzzleState<4, 4>> nbsPath;
        std::vector <MNPuzzleState<4, 4>> dvcbsPath;
        std::vector <MNPuzzleState<4, 4>> nbsEpsilonPath;
        std::vector <MNPuzzleState<4, 4>> dvcbsEpsilonPath;
        std::vector <MNPuzzleState<4, 4>> astarPath;
        std::vector <MNPuzzleState<4, 4>> rastarPath;

        Timer t1, t2, t3;

        double optimal_cost = -1.0;

        if (1) {
            NBS<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >, NBSQueue<MNPuzzleState < 4, 4>, 1, false
                    >> nbsEpsilon(false);
            goal.Reset();
            start = GetKorfInstance(x);
            t2.StartTimer();
            nbsEpsilon.GetPath(&mnp, start, goal, &mnp, &mnp, nbsEpsilonPath);
            t2.EndTimer();
            //printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
            //	   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t2.GetElapsedTime());
            printf("NBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   mnp.GetPathLength(nbsEpsilonPath),
                   nbsEpsilon.GetNodesExpanded(), nbsEpsilon.GetNecessaryExpansions(), t2.GetElapsedTime(),
                   nbsEpsilon.getForwardMeetingPoint(), nbsEpsilon.getBackwardMeetingPoint(),
                   nbsEpsilon.getForwardUnnecessaryNodesInPath(),
                   nbsEpsilon.getBackwardUnnecessaryNodesInPath(), nbsEpsilon.GetExpansionUntilFirstSolution());

            nodes_NBS += nbsEpsilon.GetNodesExpanded();
            nodes_NBSn += nbsEpsilon.GetNecessaryExpansions();
            if (nbsEpsilon.GetNodesExpanded() == nbsEpsilon.GetNecessaryExpansions()) notie_NBS++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(nbsEpsilonPath);
            else if (optimal_cost != mnp.GetPathLength(nbsEpsilonPath)) {
                printf("NBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(nbsEpsilonPath));
                exit(0);
            }
        }

        if (1) {
            NBS<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >, NBSQueue<MNPuzzleState < 4, 4>, 1, true
                    >> nbsEpsilon(false, true);
            goal.Reset();
            start = GetKorfInstance(x);
            t2.StartTimer();
            nbsEpsilon.GetPath(&mnp, start, goal, &mnp, &mnp, nbsEpsilonPath);
            t2.EndTimer();
            //printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
            //	   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t2.GetElapsedTime());
            printf("NBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   mnp.GetPathLength(nbsEpsilonPath),
                   nbsEpsilon.GetNodesExpanded(), nbsEpsilon.GetNecessaryExpansions(), t2.GetElapsedTime(),
                   nbsEpsilon.getForwardMeetingPoint(), nbsEpsilon.getBackwardMeetingPoint(),
                   nbsEpsilon.getForwardUnnecessaryNodesInPath(),
                   nbsEpsilon.getBackwardUnnecessaryNodesInPath(), nbsEpsilon.GetExpansionUntilFirstSolution());

            nodes_NBSa += nbsEpsilon.GetNodesExpanded();
            nodes_NBSan += nbsEpsilon.GetNecessaryExpansions();
            if (nbsEpsilon.GetNodesExpanded() == nbsEpsilon.GetNecessaryExpansions()) notie_NBSa++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(nbsEpsilonPath);
            else if (optimal_cost != mnp.GetPathLength(nbsEpsilonPath)) {
                printf("NBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(nbsEpsilonPath));
                exit(0);
            }
        }

        if (1) {
            DVCBS<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >, DVCBSQueue<MNPuzzleState < 4, 4>, 1, false
                    >> dvcbs(false);
            goal.Reset();
            start = GetKorfInstance(x);
            t2.StartTimer();
            dvcbs.GetPath(&mnp, start, goal, &mnp, &mnp, dvcbsEpsilonPath);
            t2.EndTimer();
            //printf("DVCBS %d found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", i,mnp.GetPathLength(dvcbsPath),
            //	dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), dvcbs.GetNodesTouched(), t2.GetElapsedTime());

            printf("DVCBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   mnp.GetPathLength(dvcbsEpsilonPath),
                   dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t2.GetElapsedTime(),
                   dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                   dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                   dvcbs.GetExpansionUntilFirstSolution());

            nodes_DVCBS += dvcbs.GetNodesExpanded();
            nodes_DVCBSn += dvcbs.GetNecessaryExpansions();
            if (dvcbs.GetNodesExpanded() == dvcbs.GetNecessaryExpansions()) notie_DVCBS++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(dvcbsEpsilonPath);
            else if (optimal_cost != mnp.GetPathLength(dvcbsEpsilonPath)) {
                printf("DVCBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(dvcbsEpsilonPath));
                exit(0);
            }
        }

        if (1) {
            DVCBS<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >, DVCBSQueue<MNPuzzleState < 4, 4>, 1, true
                    >> dvcbs(false, true);
            goal.Reset();
            start = GetKorfInstance(x);
            t2.StartTimer();
            dvcbs.GetPath(&mnp, start, goal, &mnp, &mnp, dvcbsEpsilonPath);
            t2.EndTimer();
            //printf("DVCBS %d found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", i,mnp.GetPathLength(dvcbsPath),
            //	dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), dvcbs.GetNodesTouched(), t2.GetElapsedTime());

            printf("DVCBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   mnp.GetPathLength(dvcbsEpsilonPath),
                   dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t2.GetElapsedTime(),
                   dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                   dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                   dvcbs.GetExpansionUntilFirstSolution());

            nodes_DVCBSa += dvcbs.GetNodesExpanded();
            nodes_DVCBSan += dvcbs.GetNecessaryExpansions();
            if (dvcbs.GetNodesExpanded() == dvcbs.GetNecessaryExpansions()) notie_DVCBSa++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(dvcbsEpsilonPath);
            else if (optimal_cost != mnp.GetPathLength(dvcbsEpsilonPath)) {
                printf("DVCBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(dvcbsEpsilonPath));
                exit(0);
            }
        }

        if (1) {
            std::vector <MNPuzzleState<4, 4>> solutionPath;
            Baseline<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >> baseline;
            Timer timer;
            timer.StartTimer();
            baseline.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("NBB found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath),
                   baseline.GetNodesExpanded(), baseline.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_NBB += baseline.GetNodesExpanded();
            nodes_NBBn += baseline.GetNecessaryExpansions();
            if (baseline.GetNodesExpanded() == baseline.GetNecessaryExpansions()) notie_NBB++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("NBB reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        if (1) {
            std::vector <MNPuzzleState<4, 4>> solutionPath;
            GBFHS<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >> gbfhs(true);
            Timer timer;
            timer.StartTimer();
            gbfhs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            std::cout << "GBFHS-eager found path length " << mnp.GetPathLength(solutionPath)
                      << "; "
                      << gbfhs.GetNodesExpanded() << " expanded; " << gbfhs.GetNecessaryExpansions()
                      << " necessary; "
                      << timer.GetElapsedTime() << "s elapsed, " << gbfhs.getC() << " C, " << gbfhs.getGLimF()
                      << " gLim_f, "
                      << gbfhs.getGLimB() << " gLim_b, updated forward?: " << gbfhs.getLastUpdatedLimit() << std::endl;

            nodes_GBFHS += gbfhs.GetNodesExpanded();
            nodes_GBFHSn += gbfhs.GetNecessaryExpansions();
            if (gbfhs.GetNodesExpanded() == gbfhs.GetNecessaryExpansions()) notie_GBFHS++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("GBFHS-eager reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        if (1) {
            std::vector <MNPuzzleState<4, 4>> solutionPath;
            GBFHS<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >> gbfhs(false);
            Timer timer;
            timer.StartTimer();
            gbfhs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            std::cout << "GBFHS-lazy found path length " << mnp.GetPathLength(solutionPath)
                      << "; "
                      << gbfhs.GetNodesExpanded() << " expanded; " << gbfhs.GetNecessaryExpansions()
                      << " necessary; "
                      << timer.GetElapsedTime() << "s elapsed, " << gbfhs.getC() << " C, " << gbfhs.getGLimF()
                      << " gLim_f, "
                      << gbfhs.getGLimB() << " gLim_b, updated forward?: " << gbfhs.getLastUpdatedLimit() << std::endl;

            nodes_GBFHSl += gbfhs.GetNodesExpanded();
            nodes_GBFHSln += gbfhs.GetNecessaryExpansions();
            if (gbfhs.GetNodesExpanded() == gbfhs.GetNecessaryExpansions()) notie_GBFHSl++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("GBFHS-lazy reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        if (1) {
            std::vector <MNPuzzleState<4, 4>> solutionPath;
            TemplateAStar <MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar(false, 1.0);
            astar.SetHeuristic(&mnp);
            Timer timer;
            timer.StartTimer();
            astar.GetPath(&mnp, start, goal, solutionPath);
            timer.EndTimer();
            printf("A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath),
                   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_Astar += astar.GetNodesExpanded();
            nodes_Astarn += astar.GetNecessaryExpansions();
            if (astar.GetNodesExpanded() == astar.GetNecessaryExpansions()) notie_Astar++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("AStar reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }


        if (1) {
            std::vector <MNPuzzleState<4, 4>> solutionPath;
            DBS<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >> dbs;
            Timer timer;
            timer.StartTimer();
            dbs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("DBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath),
                   dbs.GetNodesExpanded(), dbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_DBS += dbs.GetNodesExpanded();
            nodes_DBSn += dbs.GetNecessaryExpansions();
            if (dbs.GetNodesExpanded() == dbs.GetNecessaryExpansions()) notie_DBS++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBS reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }
    }

    printf("+++++++++++++++++++++++++++++++++++++++++\n");

    std::cout << " Experiments: " << 100 << std::endl;

    std::cout << "ToH" << " NBS " << nodes_NBS / 100 << " expanded; "
              << nodes_NBSn / 100 << " necessary; "
              << notie_NBS / (float) 100 << " no last layer" << std::endl;
    std::cout << "ToH" << " NBSa " << nodes_NBSa / 100 << " expanded; "
              << nodes_NBSan / 100 << " necessary; "
              << notie_NBSa / (float) 100 << " no last layer" << std::endl;
    std::cout << "ToH" << " DVCBS " << nodes_DVCBS / 100 << " expanded; "
              << nodes_DVCBSn / 100 << " necessary; "
              << notie_DVCBS / (float) 100 << " no last layer" << std::endl;
    std::cout << "ToH" << " DVCBSa " << nodes_DVCBSa / 100 << " expanded; "
              << nodes_DVCBSan / 100 << " necessary; "
              << notie_DVCBSa / (float) 100 << " no last layer" << std::endl;
    std::cout << "ToH" << " NBB " << nodes_NBB / 100 << " expanded; "
              << nodes_NBBn / 100 << " necessary; "
              << notie_NBB / (float) 100 << " no last layer" << std::endl;
    std::cout << "ToH" << " GBFHS-eager " << nodes_GBFHS / 100 << " expanded; "
              << nodes_GBFHSn / 100 << " necessary; "
              << notie_GBFHS / (float) 100 << " no last layer" << std::endl;
    std::cout << "ToH" << " GBFHS-lazy " << nodes_GBFHSl / 100 << " expanded; "
              << nodes_GBFHSln / 100 << " necessary; "
              << notie_GBFHSl / (float) 100 << " no last layer" << std::endl;
    std::cout << "ToH" << " A* " << nodes_Astar / 100 << " expanded; "
              << nodes_Astarn / 100 << " necessary; "
              << notie_Astar / (float) 100 << " no last layer" << std::endl;
    std::cout << "ToH" << " DBS " << nodes_DBS / 100 << " expanded; "
              << nodes_DBSn / 100 << " necessary; "
              << notie_DBS / (float) 100 << " no last layer" << std::endl;

    printf("+++++++++++++++++++++++++++++++++++++++++\n");

    exit(0);
}


void TestSTPFull() {
    NBS<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >> nbs;
    DVCBS<MNPuzzleState < 4, 4>, slideDir, MNPuzzle < 4, 4 >> dvcbs(1);
    MM <MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> mm;
    MNPuzzle<4, 4> mnp;
    IDAStar <MNPuzzleState<4, 4>, slideDir> ida;
    TemplateAStar <MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;

    for (int x = 0; x < 100; x++) // 547 to 540
    {

        MNPuzzleState<4, 4> start, goal;
        printf("Problem %d of %d\n", x + 1, 100);

        std::vector <slideDir> idaPath;
        std::vector <MNPuzzleState<4, 4>> dvcbsPath;
        std::vector <MNPuzzleState<4, 4>> nbsPath;
        std::vector <MNPuzzleState<4, 4>> astarPath;
        std::vector <MNPuzzleState<4, 4>> mmPath;
        Timer t1, t2, t3, t4, t5;

        goal.Reset();
        start = GetKorfInstance(x);
        t1.StartTimer();
        ida.GetPath(&mnp, start, goal, idaPath);
        t1.EndTimer();
        printf("IDA* found path length %ld; %llu expanded; %1.2fs elapsed\n", idaPath.size(), ida.GetNodesExpanded(),
               t1.GetElapsedTime());

        goal.Reset();
        start = GetKorfInstance(x);
        t2.StartTimer();
        astar.GetPath(&mnp, start, goal, astarPath);
        t2.EndTimer();
        printf("A* found path length %ld; %llu expanded; %1.2fs elapsed\n", astarPath.size() - 1,
               astar.GetNodesExpanded(), t2.GetElapsedTime());

        goal.Reset();
        start = GetKorfInstance(x);
        t3.StartTimer();
        nbs.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
        t3.EndTimer();
        printf("NBS found path length %ld; %llu expanded; %1.2fs elapsed\n", nbsPath.size() - 1, nbs.GetNodesExpanded(),
               t3.GetElapsedTime());

        goal.Reset();
        start = GetKorfInstance(x);
        t5.StartTimer();
        dvcbs.GetPath(&mnp, start, goal, &mnp, &mnp, dvcbsPath);
        t5.EndTimer();
        printf("DVCBS found path length %ld; %llu expanded; %1.2fs elapsed\n", dvcbsPath.size() - 1,
               dvcbs.GetNodesExpanded(), t5.GetElapsedTime());

        goal.Reset();
        start = GetKorfInstance(x);
        t4.StartTimer();
        mm.GetPath(&mnp, start, goal, &mnp, &mnp, mmPath);
        t4.EndTimer();
        printf("MM found path length %ld; %llu expanded; %1.2fs elapsed\n", mmPath.size() - 1, mm.GetNodesExpanded(),
               t3.GetElapsedTime());


        std::cout << ida.GetNodesExpanded() << "\t" << astar.GetNodesExpanded() << "\t" << nbs.GetNodesExpanded()
                  << "\t";
        std::cout << t1.GetElapsedTime() << "\t" << t2.GetElapsedTime() << "\t" << t3.GetElapsedTime() << "\n";

        //if (!fequal)
        if (nbsPath.size() != idaPath.size() + 1) {
            std::cout << "error solution cost:\t expected cost\n";
            std::cout << nbsPath.size() << "\t" << idaPath.size() << "\n";
//			double d;
//			for (auto x : correctPath)
//			{
//				astar.GetClosedListGCost(x, d);
//				auto t = nbs.GetNodeForwardLocation(x);
//				auto u = nbs.GetNodeBackwardLocation(x);
//				std::cout << x << " is on " << t << " and " << u << "\n";
//				std::cout << "True g: " << d;
//				if (t != kUnseen)
//					std::cout << " forward g: " << nbs.GetNodeForwardG(x);
//				if (u != kUnseen)
//					std::cout << " backward g: " << nbs.GetNodeBackwardG(x);
//				std::cout << "\n";
//			}
            exit(0);
        }

    }
    exit(0);
}
