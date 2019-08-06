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
#include "BAE.h"
#include "Baseline.h"
#include "GBFHS.h"
#include "DBBS.h"
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

    long nodes_Astar = 0, nodes_Astarn = 0, notie_Astar = 0,
            nodes_BSstar = 0, nodes_BSstarn = 0, notie_BSstar = 0, nodes_BSstara = 0, nodes_BSstaran = 0, notie_BSstara = 0,
            nodes_NBS = 0, nodes_NBSn = 0, notie_NBS = 0, nodes_NBSa = 0, nodes_NBSan = 0, notie_NBSa = 0,
            nodes_DVCBS = 0, nodes_DVCBSn = 0, notie_DVCBS = 0, nodes_DVCBSa = 0, nodes_DVCBSan = 0, notie_DVCBSa = 0,
            nodes_NBB = 0, nodes_NBBn = 0, notie_NBB = 0,
            nodes_GBFHS = 0, nodes_GBFHSn = 0, notie_GBFHS = 0, nodes_GBFHSl = 0, nodes_GBFHSln = 0, notie_GBFHSl = 0,
            nodes_GBFHSbest = 0, nodes_GBFHSbestn = 0, notie_GBFHSbest = 0,
            nodes_BAE = 0, nodes_BAEn = 0, notie_BAE = 0, nodes_BAEp = 0, nodes_BAEpn = 0, notie_BAEp = 0,
            nodes_DBS = 0, nodes_DBSn = 0, notie_DBS = 0, nodes_DBSp = 0, nodes_DBSpn = 0, notie_DBSp = 0,
            nodes_DDBS = 0, nodes_DDBSn = 0, notie_DDBS = 0, nodes_DDBSp = 0, nodes_DDBSpn = 0, notie_DDBSp = 0;

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
            if (nbsEpsilon.GetNodesExpanded() == nbsEpsilon.GetNecessaryExpansions()) notie_NBS++;

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
            if (nbsEpsilon.GetNodesExpanded() == nbsEpsilon.GetNecessaryExpansions()) notie_NBSa++;

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
            if (dvcbs.GetNodesExpanded() == dvcbs.GetNecessaryExpansions()) notie_DVCBS++;

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
            if (dvcbs.GetNodesExpanded() == dvcbs.GetNecessaryExpansions()) notie_DVCBSa++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DVCBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
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
            if (baseline.GetNodesExpanded() == baseline.GetNecessaryExpansions()) notie_NBB++;

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
            if (gbfhs.GetNodesExpanded() == gbfhs.GetNecessaryExpansions()) notie_GBFHS++;

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
            if (gbfhs.GetNodesExpanded() == gbfhs.GetNecessaryExpansions()) notie_GBFHSl++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("GBFHS-lazy reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // DBS
        if (1) {
            DBBS<TOHState < N>, TOHMove, TOH < N >, false> dbs;
            timer.StartTimer();
            dbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("DBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath),
                   dbs.GetNodesExpanded(), dbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_DBS += dbs.GetNodesExpanded();
            nodes_DBSn += dbs.GetNecessaryExpansions();
            if (dbs.GetNodesExpanded() == dbs.GetNecessaryExpansions()) notie_DBS++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBS reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // DBS-p
        if (1) {
            DBBS<TOHState < N>, TOHMove, TOH < N >, false> dbs(false);
            timer.StartTimer();
            dbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("DBS-p found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath),
                   dbs.GetNodesExpanded(), dbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_DBSp += dbs.GetNodesExpanded();
            nodes_DBSpn += dbs.GetNecessaryExpansions();
            if (dbs.GetNodesExpanded() == dbs.GetNecessaryExpansions()) notie_DBSp++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBS-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // DDBS
        if (1) {
            DBBS<TOHState < N>, TOHMove, TOH < N >> ddbs;
            timer.StartTimer();
            ddbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("DDBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath),
                   ddbs.GetNodesExpanded(), ddbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_DDBS += ddbs.GetNodesExpanded();
            nodes_DDBSn += ddbs.GetNecessaryExpansions();
            if (ddbs.GetNodesExpanded() == ddbs.GetNecessaryExpansions()) notie_DDBS++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DDBS reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // DDBS-p
        if (1) {
            DBBS<TOHState < N>, TOHMove, TOH < N >> ddbs(false);
            timer.StartTimer();
            ddbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("DDBS-p found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath),
                   ddbs.GetNodesExpanded(), ddbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_DDBSp += ddbs.GetNodesExpanded();
            nodes_DDBSpn += ddbs.GetNecessaryExpansions();
            if (ddbs.GetNodesExpanded() == ddbs.GetNecessaryExpansions()) notie_DDBSp++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DDBS-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // BS*
        if (1) {
            BSStar<TOHState < N>, TOHMove, TOH < N >> bs;
            timer.StartTimer();
            bs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("BS* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath),
                   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BSstar += bs.GetNodesExpanded();
            nodes_BSstarn += bs.GetNecessaryExpansions();
            if (bs.GetNodesExpanded() == bs.GetNecessaryExpansions()) notie_BSstar++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("BS* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // BS*-a
        if (1) {
            BSStar<TOHState < N>, TOHMove, TOH < N >> bs(true);
            timer.StartTimer();
            bs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("BS*-a found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath),
                   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BSstara += bs.GetNodesExpanded();
            nodes_BSstaran += bs.GetNecessaryExpansions();
            if (bs.GetNodesExpanded() == bs.GetNecessaryExpansions()) notie_BSstara++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("BS*-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // BAE*
        if (1) {
            BAE<TOHState < N>, TOHMove, TOH < N >> bae;
            timer.StartTimer();
            bae.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("BAE* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath),
                   bae.GetNodesExpanded(), bae.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BAE += bae.GetNodesExpanded();
            nodes_BAEn += bae.GetNecessaryExpansions();
            if (bae.GetNodesExpanded() == bae.GetNecessaryExpansions()) notie_BAE++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("BAE* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // BAE*-p
        if (1) {
            BAE<TOHState < N>, TOHMove, TOH < N >> bae(false);
            timer.StartTimer();
            bae.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("BAE*-p found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath),
                   bae.GetNodesExpanded(), bae.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BAEp += bae.GetNodesExpanded();
            nodes_BAEpn += bae.GetNecessaryExpansions();
            if (bae.GetNodesExpanded() == bae.GetNecessaryExpansions()) notie_BAEp++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("BAE*-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
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
            printf("A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath),
                   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_Astar += astar.GetNodesExpanded();
            nodes_Astarn += astar.GetNecessaryExpansions();
            if (astar.GetNodesExpanded() == astar.GetNecessaryExpansions()) notie_Astar++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("A* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
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
              << nodes_NBSn / experiments << " necessary; "
              << notie_NBS / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " NBSa " << nodes_NBSa / experiments << " expanded; "
              << nodes_NBSan / experiments << " necessary; "
              << notie_NBSa / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " DVCBS " << nodes_DVCBS / experiments << " expanded; "
              << nodes_DVCBSn / experiments << " necessary; "
              << notie_DVCBS / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " DVCBSa " << nodes_DVCBSa / experiments << " expanded; "
              << nodes_DVCBSan / experiments << " necessary; "
              << notie_DVCBSa / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " NBB " << nodes_NBB / experiments << " expanded; "
              << nodes_NBBn / experiments << " necessary; "
              << notie_NBB / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " GBFHS-eager " << nodes_GBFHS / experiments << " expanded; "
              << nodes_GBFHSn / experiments << " necessary; "
              << notie_GBFHS / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " GBFHS-lazy " << nodes_GBFHSl / experiments << " expanded; "
              << nodes_GBFHSln / experiments << " necessary; "
              << notie_GBFHSl / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " A* " << nodes_Astar / experiments << " expanded; "
              << nodes_Astarn / experiments << " necessary; "
              << notie_Astar / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " BS* " << nodes_BSstar / experiments << " expanded; "
              << nodes_BSstarn / experiments << " necessary; "
              << notie_BSstar / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " BS*-a " << nodes_BSstara / experiments << " expanded; "
              << nodes_BSstaran / experiments << " necessary; "
              << notie_BSstara / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " BAE* " << nodes_BAE / experiments << " expanded; "
              << nodes_BAEn / experiments << " necessary; "
              << notie_BAE / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " BAE*-p " << nodes_BAEp / experiments << " expanded; "
              << nodes_BAEpn / experiments << " necessary; "
              << notie_BAEp / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " DBS " << nodes_DBS / experiments << " expanded; "
              << nodes_DBSn / experiments << " necessary; "
              << notie_DBS / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " DBS-p " << nodes_DBSp / experiments << " expanded; "
              << nodes_DBSpn / experiments << " necessary; "
              << notie_DBSp / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " DDBS " << nodes_DDBS / experiments << " expanded; "
              << nodes_DDBSn / experiments << " necessary; "
              << notie_DDBS / (float) experiments << " no last layer" << std::endl;
    std::cout << "ToH" << " DDBS-p " << nodes_DDBSp / experiments << " expanded; "
              << nodes_DDBSpn / experiments << " necessary; "
              << notie_DDBSp / (float) experiments << " no last layer" << std::endl;

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
    TestTOH<12, 6>(0, 50);
    exit(0);
}