#ifndef BidirMAPS_h
#define BidirMAPS_h

#include <stdio.h>

#include "ScenarioLoader.h"
#include "Map2DEnvironment.h"
#include "MapGenerators.h"
#include "MapOverlay.h"
#include "NBSQueueGF.h"
#include "NBS.h"
#include "DVCBS.h"
#include "Baseline.h"
#include "GBFHS.h"
#include "DBS.h"
#include "TemplateAStar.h"

class MapExperiment {

    const std::string folder;
    const std::vector <std::string> mapFiles;
    const std::vector <std::string> scenarioFiles;
    const double weight;

    void runMap(const char *map, const char *scenario, double weight);

    long nodes_Astar = 0, nodes_Astarn = 0,
            nodes_NBS = 0, nodes_NBSn = 0, nodes_NBSa = 0, nodes_NBSan = 0,
            nodes_DVCBS = 0, nodes_DVCBSn = 0, nodes_DVCBSa = 0, nodes_DVCBSan = 0,
            nodes_NBB = 0, nodes_NBBn = 0, nodes_GBFHS = 0, nodes_GBFHSn = 0, nodes_GBFHSl = 0, nodes_GBFHSln = 0,
            nodes_DBS = 0, nodes_DBSn = 0,
            nodes_GBFHSbest = 0, nodes_GBFHSbestn = 0;

    int experiments = 0;

public:
    MapExperiment(std::string folder_,
                  std::vector <std::string> mapFiles_,
                  std::vector <std::string> scenarioFiles_,
                  double weight_) :
            folder(folder_), mapFiles(mapFiles_), scenarioFiles(scenarioFiles_), weight(weight_) {}

    void run() {

        // reset counters
        nodes_Astar = 0, nodes_Astarn = 0,
        nodes_NBS = 0, nodes_NBSn = 0, nodes_NBSa = 0, nodes_NBSan = 0,
        nodes_DVCBS = 0, nodes_DVCBSn = 0, nodes_DVCBSa = 0, nodes_DVCBSan = 0,
        nodes_NBB = 0, nodes_NBBn = 0, nodes_GBFHS = 0, nodes_GBFHSn = 0, nodes_GBFHSl = 0, nodes_GBFHSln = 0,
        nodes_DBS = 0, nodes_DBSn = 0,
        nodes_GBFHSbest = 0, nodes_GBFHSbestn = 0;

        experiments = 0;

        for (int i = 0; i < mapFiles.size(); i++) {
            runMap(("../../maps/" + folder + "/" + mapFiles[i]).c_str(),
                   ("../../scenarios/" + folder + "/" + scenarioFiles[i]).c_str(), 1.0);
        }

//        runMap(("../../maps/dao/brc505d.map"),
//               ("../../scenarios/dao/brc505d.map.scen"), 1.0);

        printf("+++++++++++++++++++++++++++++++++++++++++\n");

        std::cout << " Experiments: " << experiments << std::endl;

        std::cout << folder << " NBS " << nodes_NBS / experiments << " expanded; "
                  << nodes_NBSn / experiments << " necessary" << std::endl;
        std::cout << folder << " NBSa " << nodes_NBSa / experiments << " expanded; "
                  << nodes_NBSan / experiments << " necessary" << std::endl;
        std::cout << folder << " DVCBS " << nodes_DVCBS / experiments << " expanded; "
                  << nodes_DVCBSn / experiments << " necessary" << std::endl;
        std::cout << folder << " DVCBSa " << nodes_DVCBSa / experiments << " expanded; "
                  << nodes_DVCBSan / experiments << " necessary" << std::endl;
        std::cout << folder << " NBB " << nodes_NBB / experiments << " expanded; "
                  << nodes_NBBn / experiments << " necessary" << std::endl;
        std::cout << folder << " GBFHS-eager " << nodes_GBFHS / experiments << " expanded; "
                  << nodes_GBFHSn / experiments << " necessary" << std::endl;
        std::cout << folder << " GBFHS-lazy " << nodes_GBFHSl / experiments << " expanded; "
                  << nodes_GBFHSln / experiments << " necessary" << std::endl;
        std::cout << folder << " A* " << nodes_Astar / experiments << " expanded; "
                  << nodes_Astarn / experiments << " necessary" << std::endl;
        std::cout << folder << " DBS " << nodes_DBS / experiments << " expanded; "
                  << nodes_DBSn / experiments << " necessary" << std::endl;

        printf("+++++++++++++++++++++++++++++++++++++++++\n");

    }

};


#endif /* BidirMAPS_hpp */
