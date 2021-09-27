#include "BidirMaps.h"

void MapExperiment::runMap(const char *map, const char *scenario, double weight) {
    ScenarioLoader s(scenario);
    Map *m = new Map(map);
    MapEnvironment *me = new MapEnvironment(m);
    me->SetDiagonalCost(1.5);

    // 406 is bad! (?)
    for (int x = s.GetNumExperiments() - 1; x >= 0; x--) // 547 to 540
    {
        if (fequal(s.GetNthExperiment(x).GetDistance(), 0))
            continue;

        if (x != 0 && ((x + 1) % ((int) (s.GetNumExperiments() / 19)) != 0))
            continue;

        xyLoc start, goal;
        start.x = s.GetNthExperiment(x).GetStartX();
        start.y = s.GetNthExperiment(x).GetStartY();
        goal.x = s.GetNthExperiment(x).GetGoalX();
        goal.y = s.GetNthExperiment(x).GetGoalY();
//        start.x = 107;
//        start.y = 99;
//        goal.x = 236;
//        goal.y = 179;

        std::cout << map << "-" << start << "-" << goal << std::endl;

        experiments++;

        double optimal_cost = -1.0;

        if (0) {
            NBS<xyLoc, tDirection, MapEnvironment, NBSQueue<xyLoc, 1, false>> nbs(false);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            nbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("NBS-E-L found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   me->GetPathLength(path),
                   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                   nbs.getForwardMeetingPoint(), nbs.getBackwardMeetingPoint(),
                   nbs.getForwardUnnecessaryNodesInPath(), nbs.getBackwardUnnecessaryNodesInPath(),
                   nbs.GetExpansionUntilFirstSolution());

            nodes_NBS += nbs.GetNodesExpanded();
            nodes_NBSn += nbs.GetNecessaryExpansions();
            if (nbs.GetNodesExpanded() == nbs.GetNecessaryExpansions()) notie_NBS++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("NBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (0) {
            NBS<xyLoc, tDirection, MapEnvironment, NBSQueue<xyLoc, 1, true>> nbsA(false, true);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            nbsA.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("NBS-E-LEQ found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   me->GetPathLength(path),
                   nbsA.GetNodesExpanded(), nbsA.GetNecessaryExpansions(), timer.GetElapsedTime(),
                   nbsA.getForwardMeetingPoint(), nbsA.getBackwardMeetingPoint(),
                   nbsA.getForwardUnnecessaryNodesInPath(),
                   nbsA.getBackwardUnnecessaryNodesInPath(),
                   nbsA.GetExpansionUntilFirstSolution());

            nodes_NBSa += nbsA.GetNodesExpanded();
            nodes_NBSan += nbsA.GetNecessaryExpansions();
            if (nbsA.GetNodesExpanded() == nbsA.GetNecessaryExpansions()) notie_NBSa++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("NBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (0) {
            DVCBS<xyLoc, tDirection, MapEnvironment, DVCBSQueue<xyLoc, 1, false>> dvcbs(false);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            dvcbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("DVCBS-E-L found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   me->GetPathLength(path),
                   dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                   dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                   dvcbs.getForwardUnnecessaryNodesInPath(),
                   dvcbs.getBackwardUnnecessaryNodesInPath(),
                   dvcbs.GetExpansionUntilFirstSolution());

            nodes_DVCBS += dvcbs.GetNodesExpanded();
            nodes_DVCBSn += dvcbs.GetNecessaryExpansions();
            if (dvcbs.GetNodesExpanded() == dvcbs.GetNecessaryExpansions()) notie_DVCBS++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DVCBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (0) {
            DVCBS<xyLoc, tDirection, MapEnvironment, DVCBSQueue<xyLoc, 1, true>> dvcbs(false, true);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            dvcbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("DVCBS-E-LEQ found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   me->GetPathLength(path),
                   dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                   dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                   dvcbs.getForwardUnnecessaryNodesInPath(),
                   dvcbs.getBackwardUnnecessaryNodesInPath(),
                   dvcbs.GetExpansionUntilFirstSolution());

            nodes_DVCBSa += dvcbs.GetNodesExpanded();
            nodes_DVCBSan += dvcbs.GetNecessaryExpansions();
            if (dvcbs.GetNodesExpanded() == dvcbs.GetNecessaryExpansions()) notie_DVCBSa++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DVCBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (0) {
            Baseline<xyLoc, tDirection, MapEnvironment> baseline(1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            baseline.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("NBB found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   baseline.GetNodesExpanded(), baseline.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_NBB += baseline.GetNodesExpanded();
            nodes_NBBn += baseline.GetNecessaryExpansions();
            if (baseline.GetNodesExpanded() == baseline.GetNecessaryExpansions()) notie_NBB++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("NBB reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (0) {
            GBFHS<xyLoc, tDirection, MapEnvironment> gbfhs(true, 1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            gbfhs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            std::cout << "GBFHS-eager found path length " << me->GetPathLength(path)
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
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("GBFHS-eager reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (0) {
            GBFHS<xyLoc, tDirection, MapEnvironment> gbfhs(false, 1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            gbfhs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            std::cout << "GBFHS-lazy found path length " << me->GetPathLength(path)
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
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("GBFHS-lazy reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // DBGS
        if (1) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinG> dbs(true, true, 1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            dbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("DBGS found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   dbs.GetNodesExpanded(), dbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_DBS += dbs.GetNodesExpanded();
            nodes_DBSn += dbs.GetNecessaryExpansions();
            if (dbs.GetNodesExpanded() == dbs.GetNecessaryExpansions()) notie_DBS++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBGS reported bad value!! optimal %1.2f; reported %1.2f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // DBGS-p
        if (1) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinG> dbs(false, true, 1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            dbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("DBGS-p found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   dbs.GetNodesExpanded(), dbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_DBSp += dbs.GetNodesExpanded();
            nodes_DBSpn += dbs.GetNecessaryExpansions();
            if (dbs.GetNodesExpanded() == dbs.GetNecessaryExpansions()) notie_DBSp++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBGS-p reported bad value!! optimal %1.2f; reported %1.2f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        int tempDBBS;

        // DBBS
        if (1) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinB> dbbs(true, true, 1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            dbbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("DBBS found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   dbbs.GetNodesExpanded(), dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_DBBS += dbbs.GetNodesExpanded();
            nodes_DBBSn += dbbs.GetNecessaryExpansions();
            if (dbbs.GetNodesExpanded() == dbbs.GetNecessaryExpansions()) notie_DBBS++;

            tempDBBS = dbbs.GetNecessaryExpansions();

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBS reported bad value!! optimal %1.2f; reported %1.2f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // DBBS-p
        if (1) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinB> dbbs(false, true, 1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            dbbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("DBBS-p found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   dbbs.GetNodesExpanded(), dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_DBBSp += dbbs.GetNodesExpanded();
            nodes_DBBSpn += dbbs.GetNecessaryExpansions();
            if (dbbs.GetNodesExpanded() == dbbs.GetNecessaryExpansions()) notie_DBBSp++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBS-p reported bad value!! optimal %1.2f; reported %1.2f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // CDBBS
        if (1) {
            CDBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinB> cdbbs(1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            cdbbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("CDBBS-p found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   cdbbs.GetNodesExpanded(), cdbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_CDBBS += cdbbs.GetNodesExpanded();
            nodes_CDBBSn += cdbbs.GetNecessaryExpansions();
            if (cdbbs.GetNodesExpanded() == cdbbs.GetNecessaryExpansions()) notie_CDBBS++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("CDBBS reported bad value!! optimal %1.2f; reported %1.2f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        int tempBTBalt;

        // BTB alternating
        if (0) {
            BTB<xyLoc, tDirection, MapEnvironment> btb(BTBPolicy::Alternating, 1.0);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            btb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("BTB alt found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   btb.GetNodesExpanded(), btb.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BTB += btb.GetNodesExpanded();
            nodes_BTBn += btb.GetNecessaryExpansions();
            if (btb.GetNodesExpanded() == btb.GetNecessaryExpansions()) notie_BTB++;

            tempBTBalt = btb.GetNecessaryExpansions();

            if (2 * tempDBBS + 2 < tempBTBalt) {
                std::cout << "NOT NEAR-OPTIMAL!!!! " << tempDBBS << " and " << tempBTBalt << std::endl;
            }

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BTB alt reported bad value!! optimal %1.2f; reported %1.2f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // BTB smallest bucket
        if (0) {
            BTB<xyLoc, tDirection, MapEnvironment> btb(BTBPolicy::Smallest, 1.0);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            btb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("BTB small found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   btb.GetNodesExpanded(), btb.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BTB_small += btb.GetNodesExpanded();
            nodes_BTB_smalln += btb.GetNecessaryExpansions();
            if (btb.GetNodesExpanded() == btb.GetNecessaryExpansions()) notie_BTB_small++;

            if (2 * btb.GetNecessaryExpansions() + 2 < tempBTBalt) {
                std::cout << "NOT NEAR-OPTIMAL!!!! " << tempDBBS << " and " << tempBTBalt << std::endl;
            }

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BTB small reported bad value!! optimal %1.2f; reported %1.2f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // BTB most connected bucket
        if (0) {
            BTB<xyLoc, tDirection, MapEnvironment> btb(BTBPolicy::MostConnected, 1.0);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            btb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("BTB conn found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   btb.GetNodesExpanded(), btb.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BTB_conn += btb.GetNodesExpanded();
            nodes_BTB_connn += btb.GetNecessaryExpansions();
            if (btb.GetNodesExpanded() == btb.GetNecessaryExpansions()) notie_BTB_conn++;

            if (2 * btb.GetNecessaryExpansions() + 2 < tempBTBalt) {
                std::cout << "NOT NEAR-OPTIMAL!!!! " << tempDBBS << " and " << tempBTBalt << std::endl;
            }

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BTB conn reported bad value!! optimal %1.2f; reported %1.2f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // BTB vertex
        if (0) {
            BTB<xyLoc, tDirection, MapEnvironment> btb(BTBPolicy::VertexCover, 1.0);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            btb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("BTB vc found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   btb.GetNodesExpanded(), btb.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BTB_vc += btb.GetNodesExpanded();
            nodes_BTB_vcn += btb.GetNecessaryExpansions();
            if (btb.GetNodesExpanded() == btb.GetNecessaryExpansions()) notie_BTB_conn++;

            if (2 * btb.GetNecessaryExpansions() + 2 < tempBTBalt) {
                std::cout << "NOT NEAR-OPTIMAL!!!! " << tempDBBS << " and " << tempBTBalt << std::endl;
            }

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BTB vc reported bad value!! optimal %1.2f; reported %1.2f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // BS*-p
        if (0) {
            BSStar<xyLoc, tDirection, MapEnvironment> bs;
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            bs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("BS* found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BSstar += bs.GetNodesExpanded();
            nodes_BSstarn += bs.GetNecessaryExpansions();
            if (bs.GetNodesExpanded() == bs.GetNecessaryExpansions()) notie_BSstar++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BS* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // BS*-a
        if (0) {
            BSStar<xyLoc, tDirection, MapEnvironment> bs(true);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            bs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("BS*-a found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BSstara += bs.GetNodesExpanded();
            nodes_BSstaran += bs.GetNecessaryExpansions();
            if (bs.GetNodesExpanded() == bs.GetNecessaryExpansions()) notie_BSstara++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BS*-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // BAE*
        if (1) {
            BAE<xyLoc, tDirection, MapEnvironment> bae(true, 1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            bae.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("BAE* found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   bae.GetNodesExpanded(), bae.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BAE += bae.GetNodesExpanded();
            nodes_BAEn += bae.GetNecessaryExpansions();
            if (bae.GetNodesExpanded() == bae.GetNecessaryExpansions()) notie_BAE++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BAE* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // BAE*-p
        if (1) {
            BAE<xyLoc, tDirection, MapEnvironment> bae(false, 1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            bae.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("BAE*-p found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   bae.GetNodesExpanded(), bae.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_BAEp += bae.GetNodesExpanded();
            nodes_BAEpn += bae.GetNecessaryExpansions();
            if (bae.GetNodesExpanded() == bae.GetNecessaryExpansions()) notie_BAEp++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BAE*-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // A*
        if (0) {
            TemplateAStar <xyLoc, tDirection, MapEnvironment> astar(false, 1.0);
            std::vector <xyLoc> path;
            Timer timer;
            astar.SetHeuristic(me);
            timer.StartTimer();
            astar.GetPath(me, start, goal, path);
            timer.EndTimer();
            printf("A* found path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_Astar += astar.GetNodesExpanded();
            nodes_Astarn += astar.GetNecessaryExpansions();
            if (astar.GetNodesExpanded() == astar.GetNecessaryExpansions()) notie_Astar++;

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("A* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }
    }
}

