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

            NBS<xyLoc, tDirection, MapEnvironment, NBSQueue<xyLoc, 1, true>> nbsA(false, true);
            std::vector <xyLoc> pathA;
            timer.StartTimer();
            nbsA.GetPath(me, start, goal, me, me, pathA);
            timer.EndTimer();
            printf("NBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n",
                   me->GetPathLength(pathA),
                   nbsA.GetNodesExpanded(), nbsA.GetNecessaryExpansions(), timer.GetElapsedTime(),
                   nbsA.getForwardMeetingPoint(), nbsA.getBackwardMeetingPoint(),
                   nbsA.getForwardUnnecessaryNodesInPath(),
                   nbsA.getBackwardUnnecessaryNodesInPath(),
                   nbsA.GetExpansionUntilFirstSolution());

            nodes_NBSa += nbsA.GetNodesExpanded();
            nodes_NBSan += nbsA.GetNecessaryExpansions();
        }

        if (1) {
            {
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
            }

            {
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
        }

        if (1) {
            GBFHS<xyLoc, tDirection, MapEnvironment> gbfhs(true, 1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            gbfhs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("GBFHS-eager found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   gbfhs.GetNodesExpanded(), gbfhs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_GBFHS += gbfhs.GetNodesExpanded();
            nodes_GBFHSn += gbfhs.GetNecessaryExpansions();
        }

        if (1) {
            GBFHS<xyLoc, tDirection, MapEnvironment> gbfhs(false, 1.0, 0.5);
            std::vector <xyLoc> path;
            Timer timer;
            timer.StartTimer();
            gbfhs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("GBFHS-lazy found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path),
                   gbfhs.GetNodesExpanded(), gbfhs.GetNecessaryExpansions(), timer.GetElapsedTime());

            nodes_GBFHSl += gbfhs.GetNodesExpanded();
            nodes_GBFHSln += gbfhs.GetNecessaryExpansions();
        }
    }
}

