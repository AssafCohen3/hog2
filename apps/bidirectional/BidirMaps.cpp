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

        std::cout << map << "-" << start << "-" << goal << std::endl;

        experiments++;

        double optimal_cost = -1.0;

        if (1) {
            NBS<xyLoc, tDirection, MapEnvironment, NBSQueue<xyLoc, 1, false>> nbs(false);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            nbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("NBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   me->GetPathLength(path),
                   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                   nbs.getForwardMeetingPoint(), nbs.getBackwardMeetingPoint(),
                   nbs.getForwardUnnecessaryNodesInPath(), nbs.getBackwardUnnecessaryNodesInPath(),
                   nbs.GetExpansionUntilFirstSolution());

            nodes_NBS += nbs.GetNodesExpanded();
            nodes_NBSn += nbs.GetNecessaryExpansions();

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("NBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (1) {
            NBS<xyLoc, tDirection, MapEnvironment, NBSQueue<xyLoc, 1, true>> nbsA(false, true);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            nbsA.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("NBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   me->GetPathLength(path),
                   nbsA.GetNodesExpanded(), nbsA.GetNecessaryExpansions(), timer.GetElapsedTime(),
                   nbsA.getForwardMeetingPoint(), nbsA.getBackwardMeetingPoint(),
                   nbsA.getForwardUnnecessaryNodesInPath(),
                   nbsA.getBackwardUnnecessaryNodesInPath(),
                   nbsA.GetExpansionUntilFirstSolution());

            nodes_NBSa += nbsA.GetNodesExpanded();
            nodes_NBSan += nbsA.GetNecessaryExpansions();

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("NBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (1) {
            DVCBS<xyLoc, tDirection, MapEnvironment, DVCBSQueue<xyLoc, 1, false>> dvcbs(false);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            dvcbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("DVCBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   me->GetPathLength(path),
                   dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                   dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                   dvcbs.getForwardUnnecessaryNodesInPath(),
                   dvcbs.getBackwardUnnecessaryNodesInPath(),
                   dvcbs.GetExpansionUntilFirstSolution());

            nodes_DVCBS += dvcbs.GetNodesExpanded();
            nodes_DVCBSn += dvcbs.GetNecessaryExpansions();

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DVCBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (1) {
            DVCBS<xyLoc, tDirection, MapEnvironment, DVCBSQueue<xyLoc, 1, true>> dvcbs(false, true);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            dvcbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("DVCBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   me->GetPathLength(path),
                   dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime(),
                   dvcbs.getForwardMeetingPoint(), dvcbs.getBackwardMeetingPoint(),
                   dvcbs.getForwardUnnecessaryNodesInPath(),
                   dvcbs.getBackwardUnnecessaryNodesInPath(),
                   dvcbs.GetExpansionUntilFirstSolution());

            nodes_DVCBSa += dvcbs.GetNodesExpanded();
            nodes_DVCBSan += dvcbs.GetNecessaryExpansions();

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DVCBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (1) {
            Baseline<xyLoc, tDirection, MapEnvironment> baseline(1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            baseline.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("NBB found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   baseline.GetNodesExpanded(), baseline.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_NBB += baseline.GetNodesExpanded();
            nodes_NBBn += baseline.GetNecessaryExpansions();

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("NBB reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (1) {
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

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("GBFHS-eager reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (1) {
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

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("GBFHS-lazy reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (1) {
            DBS<xyLoc, tDirection, MapEnvironment> dbs(1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            dbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("DBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   dbs.GetNodesExpanded(), dbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_DBS += dbs.GetNodesExpanded();
            nodes_DBSn += dbs.GetNecessaryExpansions();

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBS reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        if (1) {
            TemplateAStar <xyLoc, tDirection, MapEnvironment> astar(false, 1.0);
            std::vector <xyLoc> path;
            Timer timer;
            astar.SetHeuristic(me);
            timer.StartTimer();
            astar.GetPath(me, start, goal, path);
            timer.EndTimer();
            printf("A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_Astar += astar.GetNodesExpanded();
            nodes_Astarn += astar.GetNecessaryExpansions();

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

