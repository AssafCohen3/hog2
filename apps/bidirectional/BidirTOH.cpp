//
//  BidirTOH.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/14/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirTOH.h"
#include "TOH.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "DVCBS.h"
#include "MM.h"
#include "BSStar.h"
#include "Baseline.h"
#include "GBFHS.h"
#include "DBS.h"
#include "CalculateWVC.h"


template<int numDisks, int pdb1Disks, int pdb2Disks = numDisks - pdb1Disks>
Heuristic <TOHState<numDisks>> *BuildPDB(const TOHState <numDisks> &goal) {
    TOH <numDisks> toh;
    TOH <pdb1Disks> absToh1;
    TOH <pdb2Disks> absToh2;
    TOHState <pdb1Disks> absTohState1;
    TOHState <pdb2Disks> absTohState2;


    TOHPDB <pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1,
                                                                                               goal); // top disks
    TOHPDB <pdb2Disks, numDisks> *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
    pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
    pdb2->BuildPDB(goal, std::thread::hardware_concurrency());

    Heuristic <TOHState<numDisks>> *h = new Heuristic <TOHState<numDisks>>;

    h->lookups.resize(0);
    h->lookups.push_back({kAddNode, 1, 2});
    h->lookups.push_back({kLeafNode, 0, 0});
    h->lookups.push_back({kLeafNode, 1, 1});
    h->heuristics.resize(0);
    h->heuristics.push_back(pdb1);
    h->heuristics.push_back(pdb2);

    return h;
}

template<int N, int pdb1Disks>
void TestTOH(int first, int last) {


    BSStar<TOHState < N>, TOHMove, TOH < N >> bs;
    MM <TOHState<N>, TOHMove, TOH<N>> mm;


    TOH <N> toh;
    TOHState <N> s;
    TOHState <N> g;
    std::vector <TOHState<N>> thePath;
    std::vector <TOHMove> actionPath;
    Heuristic <TOHState<N>> *f;
    Heuristic <TOHState<N>> *b;

    //g.Reset();
    //f = BuildPDB<N, pdb1Disks>(g);

    long nodes_Astar = 0, nodes_Astarn = 0,
            nodes_NBS = 0, nodes_NBSn = 0, nodes_NBSa = 0, nodes_NBSan = 0,
            nodes_DVCBS = 0, nodes_DVCBSn = 0, nodes_DVCBSa = 0, nodes_DVCBSan = 0,
            nodes_NBB = 0, nodes_NBBn = 0, nodes_GBFHS = 0, nodes_GBFHSn = 0, nodes_GBFHSl = 0, nodes_GBFHSln = 0,
            nodes_DBS = 0, nodes_DBSn = 0,
            nodes_GBFHSbest = 0, nodes_GBFHSbestn = 0;

    int table[] = {52058078, 116173544, 208694125, 131936966, 141559500, 133800745, 194246206, 50028346, 167007978,
                   207116816, 163867037, 119897198, 201847476, 210859515, 117688410, 121633885};
    int table2[] = {145008714, 165971878, 154717942, 218927374, 182772845, 5808407, 19155194, 137438954, 13143598,
                    124513215, 132635260, 39667704, 2462244, 41006424, 214146208, 54305743};
    for (int count = first; count < last; count++) {

        //printf("Seed: %d\n", table[count&0xF]^table2[(count>>4)&0xF]);
        srandom(table[count & 0xF] ^ table2[(count >> 4) & 0xF]);

        s.counts[0] = s.counts[1] = s.counts[2] = s.counts[3] = 0;
        for (int x = N; x > 0; x--) {
            int whichPeg = random() % 4;
            s.disks[whichPeg][s.counts[whichPeg]] = x;
            s.counts[whichPeg]++;
        }
        b = BuildPDB<N, pdb1Disks>(s);

        g.counts[0] = g.counts[1] = g.counts[2] = g.counts[3] = 0;
        for (int x = N; x > 0; x--) {
            int whichPeg = random() % 4;
            g.disks[whichPeg][g.counts[whichPeg]] = x;
            g.counts[whichPeg]++;
        }
        g.Reset();
        f = BuildPDB<N, pdb1Disks>(g);
        Timer timer;

        std::cout << "-----------------------------" << std::endl;
        std::cout << "ToH problem: " << count + 1 << " of " << last << std::endl;

        double optimal_cost = -1.0;

        //b = BuildPDB<N, pdb1Disks>(s);
        //printf("Starting heuristics: %f %f\n", f->HCost(s, g), b->HCost(g, s));
        //Single Solution
        if (1) {
            if (0) {
                DVCBS<TOHState < N>, TOHMove, TOH < N >, DVCBSQueue<TOHState < N>, 0, false >> dvcbs(false);
                timer.StartTimer();
                dvcbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                       dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                       dvcbs.GetExpansionUntilFirstSolution());
            }

            if (0) {
                DVCBS<TOHState < N>, TOHMove, TOH < N >, DVCBSQueue<TOHState < N>, 0, true >> dvcbs(false, true);
                timer.StartTimer();
                dvcbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("DVCBS-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                       dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                       dvcbs.GetExpansionUntilFirstSolution());

            }
            if (1) {
                NBS<TOHState < N>, TOHMove, TOH < N >, NBSQueue<TOHState < N>, 1, false >> nbsEpsilon(false);
                timer.StartTimer();
                nbsEpsilon.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("NBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       nbsEpsilon.GetNodesExpanded(), nbsEpsilon.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       nbsEpsilon.getForwardMeetingPoint(), nbsEpsilon.getBackwardMeetingPoint(),
                       nbsEpsilon.getForwardUnnecessaryNodesInPath(), nbsEpsilon.getBackwardUnnecessaryNodesInPath(),
                       nbsEpsilon.GetExpansionUntilFirstSolution());

                nodes_NBS += nbsEpsilon.GetNodesExpanded();
                nodes_NBSn += nbsEpsilon.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
                else if (optimal_cost != toh.GetPathLength(thePath)) {
                    printf("NBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                           optimal_cost, toh.GetPathLength(thePath));
                    exit(0);
                }

            }
            if (1) {
                NBS<TOHState < N>, TOHMove, TOH < N >, NBSQueue<TOHState < N>, 1, true >> nbsEpsilon(false, true);
                timer.StartTimer();
                nbsEpsilon.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("NBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       nbsEpsilon.GetNodesExpanded(), nbsEpsilon.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       nbsEpsilon.getForwardMeetingPoint(), nbsEpsilon.getBackwardMeetingPoint(),
                       nbsEpsilon.getForwardUnnecessaryNodesInPath(), nbsEpsilon.getBackwardUnnecessaryNodesInPath(),
                       nbsEpsilon.GetExpansionUntilFirstSolution());

                nodes_NBSa += nbsEpsilon.GetNodesExpanded();
                nodes_NBSan += nbsEpsilon.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
                else if (optimal_cost != toh.GetPathLength(thePath)) {
                    printf("NBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n",
                           optimal_cost, toh.GetPathLength(thePath));
                    exit(0);
                }

            }
            if (1) {
                DVCBS<TOHState < N>, TOHMove, TOH < N >, DVCBSQueue<TOHState < N>, 1, false >> dvcbs(false);
                timer.StartTimer();
                dvcbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("DVCBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                       dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                       dvcbs.GetExpansionUntilFirstSolution());

                nodes_DVCBS += dvcbs.GetNodesExpanded();
                nodes_DVCBSn += dvcbs.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
                else if (optimal_cost != toh.GetPathLength(thePath)) {
                    printf("DVCBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                           optimal_cost, toh.GetPathLength(thePath));
                    exit(0);
                }

            }
            if (1) {
                DVCBS<TOHState < N>, TOHMove, TOH < N >, DVCBSQueue<TOHState < N>, 1, true >> dvcbs(false, true);
                timer.StartTimer();
                dvcbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("DVCBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                       dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                       dvcbs.GetExpansionUntilFirstSolution());

                nodes_DVCBSa += dvcbs.GetNodesExpanded();
                nodes_DVCBSan += dvcbs.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
                else if (optimal_cost != toh.GetPathLength(thePath)) {
                    printf("DVCBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n",
                           optimal_cost, toh.GetPathLength(thePath));
                    exit(0);
                }
            }

            if (0) {
                NBS<TOHState < N>, TOHMove, TOH < N >, NBSQueue<TOHState < N>, 0, false >> nbs(false);
                timer.StartTimer();
                nbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("NBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       nbs.getForwardMeetingPoint(), nbs.getBackwardMeetingPoint(),
                       nbs.getForwardUnnecessaryNodesInPath(), nbs.getBackwardUnnecessaryNodesInPath(),
                       nbs.GetExpansionUntilFirstSolution());

            }
            if (0) {
                NBS<TOHState < N>, TOHMove, TOH < N >, NBSQueue<TOHState < N>, 0, true >> nbs(false, true);
                timer.StartTimer();
                nbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("NBS-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       nbs.getForwardMeetingPoint(), nbs.getBackwardMeetingPoint(),
                       nbs.getForwardUnnecessaryNodesInPath(), nbs.getBackwardUnnecessaryNodesInPath(),
                       nbs.GetExpansionUntilFirstSolution());

            }

            if (1) {
                Baseline<TOHState < N>, TOHMove, TOH < N >> baseline;
                timer.StartTimer();
                baseline.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("NBB found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                       toh.GetPathLength(thePath),
                       baseline.GetNodesExpanded(), baseline.GetNecessaryExpansions(), timer.GetElapsedTime());

                nodes_NBB += baseline.GetNodesExpanded();
                nodes_NBBn += baseline.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
                else if (optimal_cost != toh.GetPathLength(thePath)) {
                    printf("NBB reported bad value!! optimal %1.0f; reported %1.0f;\n",
                           optimal_cost, toh.GetPathLength(thePath));
                    exit(0);
                }
            }

            if (1) {
                GBFHS<TOHState < N>, TOHMove, TOH < N >> gbfhs(true);
                timer.StartTimer();
                gbfhs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                std::cout << "GBFHS-eager found path length " << toh.GetPathLength(thePath)
                          << "; "
                          << gbfhs.GetNodesExpanded() << " expanded; " << gbfhs.GetNecessaryExpansions()
                          << " necessary; "
                          << timer.GetElapsedTime() << "s elapsed, " << gbfhs.getC() << " C, " << gbfhs.getGLimF()
                          << " gLim_f, "
                          << gbfhs.getGLimB() << " gLim_b, updated forward?: " << gbfhs.getLastUpdatedLimit()
                          << std::endl;

                nodes_GBFHS += gbfhs.GetNodesExpanded();
                nodes_GBFHSn += gbfhs.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
                else if (optimal_cost != toh.GetPathLength(thePath)) {
                    printf("GBFHS-eager reported bad value!! optimal %1.0f; reported %1.0f;\n",
                           optimal_cost, toh.GetPathLength(thePath));
                    exit(0);
                }
            }

            if (1) {
                GBFHS<TOHState < N>, TOHMove, TOH < N >> gbfhs(false);
                timer.StartTimer();
                gbfhs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                std::cout << "GBFHS-lazy found path length " << toh.GetPathLength(thePath)
                          << "; "
                          << gbfhs.GetNodesExpanded() << " expanded; " << gbfhs.GetNecessaryExpansions()
                          << " necessary; "
                          << timer.GetElapsedTime() << "s elapsed, " << gbfhs.getC() << " C, " << gbfhs.getGLimF()
                          << " gLim_f, "
                          << gbfhs.getGLimB() << " gLim_b, updated forward?: " << gbfhs.getLastUpdatedLimit()
                          << std::endl;

                nodes_GBFHSl += gbfhs.GetNodesExpanded();
                nodes_GBFHSln += gbfhs.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
                else if (optimal_cost != toh.GetPathLength(thePath)) {
                    printf("GBFHS-lazy reported bad value!! optimal %1.0f; reported %1.0f;\n",
                           optimal_cost, toh.GetPathLength(thePath));
                    exit(0);
                }
            }

            if (1) {
                TemplateAStar <TOHState<N>, TOHMove, TOH<N>> astar(false, 1);
                astar.SetHeuristic(f);
                timer.StartTimer();
                astar.GetPath(&toh, s, g, thePath);
                timer.EndTimer();
                //printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
                printf("A*-E %llu nodes %llu necessary", astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
                printf(" %1.2fs elapsed\n", timer.GetElapsedTime());

                nodes_Astar += astar.GetNodesExpanded();
                nodes_Astarn += astar.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
                else if (optimal_cost != toh.GetPathLength(thePath)) {
                    printf("AStar reported bad value!! optimal %1.0f; reported %1.0f;\n",
                           optimal_cost, toh.GetPathLength(thePath));
                    exit(0);
                }
            }

            if (1) {
                DBS<TOHState < N>, TOHMove, TOH < N >> dbs;
                timer.StartTimer();
                dbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("DBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                       toh.GetPathLength(thePath),
                       dbs.GetNodesExpanded(), dbs.GetNecessaryExpansions(), timer.GetElapsedTime());

                nodes_DBS += dbs.GetNodesExpanded();
                nodes_DBSn += dbs.GetNecessaryExpansions();

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
                else if (optimal_cost != toh.GetPathLength(thePath)) {
                    printf("DBS reported bad value!! optimal %1.0f; reported %1.0f;\n",
                           optimal_cost, toh.GetPathLength(thePath));
                    exit(0);
                }
            }
        }

        //All Solutions
        if (0) {
            if (1) {
                DVCBS<TOHState < N>, TOHMove, TOH < N >, DVCBSQueue<TOHState < N>, 0, false >> dvcbs(true);
                timer.StartTimer();
                dvcbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                       dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                       dvcbs.GetExpansionUntilFirstSolution());
            }

            if (1) {
                DVCBS<TOHState < N>, TOHMove, TOH < N >, DVCBSQueue<TOHState < N>, 0, true >> dvcbs(true, true);
                timer.StartTimer();
                dvcbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("DVCBS-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                       dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                       dvcbs.GetExpansionUntilFirstSolution());
            }
            if (1) {
                DVCBS<TOHState < N>, TOHMove, TOH < N >, DVCBSQueue<TOHState < N>, 1, false >> dvcbs(true);
                timer.StartTimer();
                dvcbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("DVCBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                       dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                       dvcbs.GetExpansionUntilFirstSolution());
            }
            if (1) {
                DVCBS<TOHState < N>, TOHMove, TOH < N >, DVCBSQueue<TOHState < N>, 1, true >> dvcbs(true, true);
                timer.StartTimer();
                dvcbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("DVCBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                       dvcbs.getForwardUnnecessaryNodesInPath(), dvcbs.getBackwardUnnecessaryNodesInPath(),
                       dvcbs.GetExpansionUntilFirstSolution());

            }

            if (1) {
                NBS<TOHState < N>, TOHMove, TOH < N >, NBSQueue<TOHState < N>, 0, false >> nbs(true);
                timer.StartTimer();
                nbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("NBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       nbs.getForwardMeetingPoint(), nbs.getBackwardMeetingPoint(),
                       nbs.getForwardUnnecessaryNodesInPath(), nbs.getBackwardUnnecessaryNodesInPath(),
                       nbs.GetExpansionUntilFirstSolution());

            }
            if (1) {
                NBS<TOHState < N>, TOHMove, TOH < N >, NBSQueue<TOHState < N>, 0, true >> nbs(true, true);
                timer.StartTimer();
                nbs.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("NBS-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       nbs.getForwardMeetingPoint(), nbs.getBackwardMeetingPoint(),
                       nbs.getForwardUnnecessaryNodesInPath(), nbs.getBackwardUnnecessaryNodesInPath(),
                       nbs.GetExpansionUntilFirstSolution());

            }
            if (1) {
                NBS<TOHState < N>, TOHMove, TOH < N >, NBSQueue<TOHState < N>, 1, false >> nbsEpsilon(true);
                timer.StartTimer();
                nbsEpsilon.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("NBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       nbsEpsilon.GetNodesExpanded(), nbsEpsilon.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       nbsEpsilon.getForwardMeetingPoint(), nbsEpsilon.getBackwardMeetingPoint(),
                       nbsEpsilon.getForwardUnnecessaryNodesInPath(), nbsEpsilon.getBackwardUnnecessaryNodesInPath(),
                       nbsEpsilon.GetExpansionUntilFirstSolution());

            }
            if (1) {
                NBS<TOHState < N>, TOHMove, TOH < N >, NBSQueue<TOHState < N>, 1, true >> nbsEpsilon(true, true);
                timer.StartTimer();
                nbsEpsilon.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("NBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                       toh.GetPathLength(thePath),
                       nbsEpsilon.GetNodesExpanded(), nbsEpsilon.GetNecessaryExpansions(), timer.GetElapsedTime(),
                       nbsEpsilon.getForwardMeetingPoint(), nbsEpsilon.getBackwardMeetingPoint(),
                       nbsEpsilon.getForwardUnnecessaryNodesInPath(), nbsEpsilon.getBackwardUnnecessaryNodesInPath(),
                       nbsEpsilon.GetExpansionUntilFirstSolution());

            }
        }

        if (0) {
            //printf("-=-=-BS-=-=-\n");
            timer.StartTimer();
            bs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            //printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
            printf("BS %llu nodes %llu necessary ", bs.GetNodesExpanded(), bs.GetNecessaryExpansions());
            printf("%1.2fs elapsed\n", timer.GetElapsedTime());
        }

        if (0) {
            //printf("-=-=-MM-=-=-\n");
            timer.StartTimer();
            mm.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            //printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
            printf("MM %llu nodes %llu necessary", mm.GetNodesExpanded(), mm.GetNecessaryExpansions());
            printf("%1.2fs elapsed\n", timer.GetElapsedTime());
        }
        if (0) {
            //printf("-=-=-A*-=-=-\n");
            TemplateAStar <TOHState<N>, TOHMove, TOH<N>> astar;
            astar.SetHeuristic(f);
            timer.StartTimer();
            astar.GetPath(&toh, s, g, thePath);
            timer.EndTimer();
            //printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
            printf("A* %llu nodes %llu necessary", astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
            printf(" %1.2fs elapsed\n", timer.GetElapsedTime());
        }
        if (0) {
            //printf("-=-=-A*-=-=-\n");
            TemplateAStar <TOHState<N>, TOHMove, TOH<N>> astar(true);
            astar.SetHeuristic(f);
            timer.StartTimer();
            astar.GetPath(&toh, s, g, thePath);
            timer.EndTimer();
            //printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
            printf("A*-A %llu nodes %llu necessary", astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
            printf(" %1.2fs elapsed\n", timer.GetElapsedTime());
        }
        if (0) {
            //printf("-=-=-A*-=-=-\n");
            TemplateAStar <TOHState<N>, TOHMove, TOH<N>> astar(false, 1);
            astar.SetHeuristic(f);
            timer.StartTimer();
            astar.GetPath(&toh, s, g, thePath);
            timer.EndTimer();
            //printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
            printf("A*-E %llu nodes %llu necessary", astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
            printf(" %1.2fs elapsed\n", timer.GetElapsedTime());
        }
        if (0) {
            //printf("-=-=-A*-=-=-\n");
            TemplateAStar <TOHState<N>, TOHMove, TOH<N>> astar(true, 1);
            astar.SetHeuristic(f);
            timer.StartTimer();
            astar.GetPath(&toh, s, g, thePath);
            timer.EndTimer();
            //printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
            printf("A*-A-E %llu nodes %llu necessary", astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
            printf(" %1.2fs elapsed\n", timer.GetElapsedTime());
        }
        if (0) {
            TemplateAStar <TOHState<N>, TOHMove, TOH<N>> rastar;
            //printf("-=-=-A*-=-=-\n");
            rastar.SetHeuristic(b);
            timer.StartTimer();
            rastar.GetPath(&toh, g, s, thePath);
            timer.EndTimer();
            //printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
            printf("R-A* %llu nodes %llu necessary", rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions());
            printf("%1.2fs elapsed\n", timer.GetElapsedTime());
        }
        //fMM
        if (0) {

            TemplateAStar <TOHState<N>, TOHMove, TOH<N>> astar(true);
            TemplateAStar <TOHState<N>, TOHMove, TOH<N>> rastar(true);
            astar.SetHeuristic(f);
            timer.StartTimer();
            astar.GetPath(&toh, s, g, thePath);
            timer.EndTimer();
            printf("A* %llu nodes %llu necessary %1.0f path %1.2fs elapsed\n", astar.GetNodesExpanded(),
                   astar.GetNecessaryExpansions(), toh.GetPathLength(thePath), timer.GetElapsedTime());
            rastar.SetHeuristic(b);
            timer.StartTimer();
            rastar.GetPath(&toh, g, s, thePath);
            timer.EndTimer();
            printf("R-A* %llu nodes %llu necessary %1.0f path %1.2fs elapsed\n", rastar.GetNodesExpanded(),
                   rastar.GetNecessaryExpansions(), toh.GetPathLength(thePath), timer.GetElapsedTime());
            std::vector < AStarOpenClosedData < TOHState < N >> > astarOpenClose = astar.openClosedList.elements;
            std::vector < AStarOpenClosedData < TOHState < N >> > rastarOpenClose = rastar.openClosedList.elements;
            CalculateWVC<TOHState < N>>
            calculateWVC;
            int C = toh.GetPathLength(thePath);
            printf("Optimal-Necessary-L %f\n", calculateWVC.CalcWVC(astarOpenClose, rastarOpenClose, C, 0, false));
            printf("Optimal-Necessary-E-L %f\n", calculateWVC.CalcWVC(astarOpenClose, rastarOpenClose, C, 1, false));
            printf("Optimal-Necessary-LEQ %f\n", calculateWVC.CalcWVC(astarOpenClose, rastarOpenClose, C, 0, true));
            printf("Optimal-Necessary-E-LEQ %f\n", calculateWVC.CalcWVC(astarOpenClose, rastarOpenClose, C, 1, true));

        }
        while (b->heuristics.size() > 0) {
            delete b->heuristics.back();
            b->heuristics.pop_back();
        }
        delete b;
    }

    printf("+++++++++++++++++++++++++++++++++++++++++\n");

    auto experiments = last - first;

    std::cout << " Experiments: " << experiments << std::endl;

    std::cout << "ToH" << " NBS " << nodes_NBS / experiments << " expanded; "
              << nodes_NBSn / experiments << " necessary" << std::endl;
    std::cout << "ToH" << " NBSa " << nodes_NBSa / experiments << " expanded; "
              << nodes_NBSan / experiments << " necessary" << std::endl;
    std::cout << "ToH" << " DVCBS " << nodes_DVCBS / experiments << " expanded; "
              << nodes_DVCBSn / experiments << " necessary" << std::endl;
    std::cout << "ToH" << " DVCBSa " << nodes_DVCBSa / experiments << " expanded; "
              << nodes_DVCBSan / experiments << " necessary" << std::endl;
    std::cout << "ToH" << " NBB " << nodes_NBB / experiments << " expanded; "
              << nodes_NBBn / experiments << " necessary" << std::endl;
    std::cout << "ToH" << " GBFHS-eager " << nodes_GBFHS / experiments << " expanded; "
              << nodes_GBFHSn / experiments << " necessary" << std::endl;
    std::cout << "ToH" << " GBFHS-lazy " << nodes_GBFHSl / experiments << " expanded; "
              << nodes_GBFHSln / experiments << " necessary" << std::endl;
    std::cout << "ToH" << " A* " << nodes_Astar / experiments << " expanded; "
              << nodes_Astarn / experiments << " necessary" << std::endl;
    std::cout << "ToH" << " DBS " << nodes_DBS / experiments << " expanded; "
              << nodes_DBSn / experiments << " necessary" << std::endl;

    printf("+++++++++++++++++++++++++++++++++++++++++\n");

    while (f->heuristics.size() > 0) {
        delete f->heuristics.back();
        f->heuristics.pop_back();
    }
    delete f;
}

void TOHTest() {
    TestTOH<12, 2>(0, 50);
    TestTOH<12, 4>(0, 50);
//	TestTOH<14, 5>(0, 50);
    TestTOH<12, 6>(0, 50);
//	TestTOH<14, 7>(0, 50);
//	TestTOH<14, 8>(0, 50);
//	TestTOH<14, 9>(0, 50);
//	TestTOH<14, 10>(0, 50);
//	const int numDisks = 16; // [disks - 2] (4^14 - 256 million)
//	const int pdb1Disks = 10;
//	const int pdb2Disks = 6;
//	TOH<numDisks> toh;
//	TOHState<numDisks> s, g;
//	
//	TOHState<numDisks> goal;
//	TOH<pdb1Disks> absToh1;
//	TOH<pdb2Disks> absToh2;
//	TOHState<pdb1Disks> absTohState1;
//	TOHState<pdb2Disks> absTohState2;
//
//	TOHPDB<pdb1Disks, numDisks, pdb2Disks> pdb1(&absToh1); // top disks
//	TOHPDB<pdb2Disks, numDisks> pdb2(&absToh2); // bottom disks
//
//	ZeroHeuristic<TOHState<numDisks>> z;
//	
//	goal.Reset();
//	pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb2.BuildPDB(goal, std::thread::hardware_concurrency());
//	
////	s.Reset();
////	goal.Reset();
////	for (int x = 0; x < 100; x++)
////	{
////		std::vector<TOHMove> actionPath;
////		for (int x = 0; x < 20000; x++)
////		{
////			toh.GetActions(s, actionPath);
////			toh.ApplyAction(s, actionPath[random()%actionPath.size()]);
////		}
////		std::cout << s << "\n";
////		std::cout << "H1: " << pdb1.HCost(s, goal) << "\n";
////		std::cout << "H2: " << pdb2.HCost(s, goal) << "\n";
////	}
////	exit(0);
//	goal.Reset();
//	Heuristic<TOHState<numDisks>> h;
//	
//	h.lookups.resize(0);
//	h.lookups.push_back({kAddNode, 1, 2});
//	h.lookups.push_back({kLeafNode, 0, 0});
//	h.lookups.push_back({kLeafNode, 1, 1});
//	h.heuristics.resize(0);
//	h.heuristics.push_back(&pdb1);
//	h.heuristics.push_back(&pdb2);
//	printf("-=-=-=-==-=-=-=-=-\n");
//	printf("With %d and %d\n", pdb1Disks, pdb2Disks);
//	TestTOH<numDisks>(&h, 0, 10);
//
//	printf("-=-=-=-==-=-=-=-=-\n");
//	h.heuristics[1] = &z;
//	printf("With just %d\n", pdb1Disks);
//	TestTOH<numDisks>(&h, 0, 10);
//	exit(0);
}