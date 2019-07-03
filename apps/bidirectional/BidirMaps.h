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

class MapExperiment {

    const std::string folder;
    const std::vector <std::string> mapFiles;
    const std::vector <std::string> scenarioFiles;
    const double weight;

    void runMap(const char *map, const char *scenario, double weight);

    long nodes_NBS, nodes_NBSn, nodes_NBSa, nodes_NBSan,
            nodes_DVCBS, nodes_DVCBSn, nodes_DVCBSa, nodes_DVCBSan,
            nodes_NBB, nodes_NBBn,
            nodes_GBFHS, nodes_GBFHSn, nodes_GBFHSl, nodes_GBFHSln = 0;

    int experiments = 0;

public:
    MapExperiment(std::string folder_,
                  std::vector <std::string> mapFiles_,
                  std::vector <std::string> scenarioFiles_,
                  double weight_) :
            folder(folder_), mapFiles(mapFiles_), scenarioFiles(scenarioFiles_), weight(weight_) {}

    void run() {

        // reset counters
        nodes_NBS, nodes_NBSn, nodes_NBSa, nodes_NBSan,
                nodes_DVCBS, nodes_DVCBSn, nodes_DVCBSa, nodes_DVCBSan,
                nodes_NBB, nodes_NBBn,
                nodes_GBFHS, nodes_GBFHSn, nodes_GBFHSl, nodes_GBFHSln = 0;

        experiments = 0;

        for (int i = 0; i < mapFiles.size(); i++) {
            runMap(("../../maps/" + folder + "/" + mapFiles[i]).c_str(),
                   ("../../scenarios/" + folder + "/" + scenarioFiles[i]).c_str(), 1.0);
        }

        printf("+++++++++++++++++++++++++++++++++++++++++\n");

        std::cout << " Experiments: " << experiments << std::endl;

        std::cout << folder << " NBS " << nodes_NBS << " expanded; " << nodes_NBSn << " necessary" << std::endl;
        std::cout << folder << " NBSa " << nodes_NBSa << " expanded; " << nodes_NBSan << " necessary" << std::endl;
        std::cout << folder << " DVCBS " << nodes_DVCBS << " expanded; " << nodes_DVCBSn << " necessary" << std::endl;
        std::cout << folder << " DVCBSa " << nodes_DVCBSa << " expanded; " << nodes_DVCBSan << " necessary"
                  << std::endl;
        std::cout << folder << " NBB " << nodes_NBB << " expanded; " << nodes_NBBn << " necessary" << std::endl;
        std::cout << folder << " GBFHS " << nodes_GBFHS << " expanded; " << nodes_GBFHSn << " necessary" << std::endl;
        std::cout << folder << " GBFHS " << nodes_GBFHSl << " expanded; " << nodes_GBFHSln << " necessary" << std::endl;

        printf("+++++++++++++++++++++++++++++++++++++++++\n");

    }

};


#endif /* BidirMAPS_hpp */
