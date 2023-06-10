#include "P2D/P2D.hpp"
#include "P2D/debug.hpp"
#include "VG2/VG2.hpp"

int main(int, char **)
{
    std::vector<P2D::Scenarios> expt = P2D::getExperiment("VG2");
    for (const P2D::Scenarios & scens : expt)
        P2D::writeResults(scens);


    _dbg11("hi");

    // bool build_map_only = false;
    // constexpr bool block = true;
    // using alg = P2D::VG2::VG2<block>;
    // std::string alg_type = block ? "B" : "N"; // N := NoblockAndReturnState, B := BlockAndReturnState // must be same as above

    // std::vector<std::array<std::string, 2>> map_data = {
    //     // {"sc1/", "Aftershock"},
    //     {"sc1/", "Aurora2"},
    //     // {"sc1/", "Aftershock2"},
    //     // {"sc1/", "ArcticStation" },

    //     // {"maze/", "maze512-4-0"},
    //     // {"maze/", "maze512-4-1"},
    //     // {"maze/", "maze512-4-2"},
    //     // {"maze/", "maze512-4-3"},
    //     // {"maze/", "maze512-4-4"},
    //     // {"maze/", "maze512-4-5"},
    //     // {"maze/", "maze512-4-6"},
    //     // {"maze/", "maze512-4-7"},
    //     // {"maze/", "maze512-4-8"},
    //     // {"maze/", "maze512-4-9"},

    //     // {"maze/", "maze512-8-0"},
    //     // {"maze/", "maze512-8-1"},
    //     // {"maze/", "maze512-8-2"},
    //     // {"maze/", "maze512-8-3"},
    //     // {"maze/", "maze512-8-4"},
    //     // {"maze/", "maze512-8-5"},
    //     // {"maze/", "maze512-8-6"},
    //     // {"maze/", "maze512-8-7"},
    //     // {"maze/", "maze512-8-8"},
    //     // {"maze/", "maze512-8-9"},

    //     // {"random/", "random512-10-0"},
    //     // {"random/", "random512-10-1"},
    //     // {"random/", "random512-10-2"},
    //     // {"random/", "random512-10-3"},
    //     // {"random/", "random512-10-4"},
    //     // {"random/", "random512-10-5"},
    //     // {"random/", "random512-10-6"},
    //     // {"random/", "random512-10-7"},
    //     // {"random/", "random512-10-8"},
    //     // {"random/", "random512-10-9"},

    //     // {"random/", "random512-20-0"},
    //     // {"random/", "random512-20-1"},
    //     // {"random/", "random512-20-2"},
    //     // {"random/", "random512-20-3"},
    //     // {"random/", "random512-20-4"},
    //     // {"random/", "random512-20-5"},
    //     // {"random/", "random512-20-6"},
    //     // {"random/", "random512-20-7"},
    //     // {"random/", "random512-20-8"},
    //     // {"random/", "random512-20-9"},

    //     // {"da2/", "ca_cave"},
    //     // {"da2/", "ca_cavern1_haunted"},
    //     // {"da2/", "ca_caverns1"},
    //     // {"da2/", "ca_caverns1_mines"},
    //     // {"da2/", "ca_caverns2"},
    //     // {"da2/", "ca_caverns2prc"},

    //     // {"maze/", "maze512-1-0"},
    //     // {"maze/", "maze512-2-0"},
    //     // {"maze/", "maze512-4-0"},
    //     // {"maze/", "maze512-8-0"},
    //     // {"maze/", "maze512-16-0"},
    //     // {"maze/", "maze512-32-0"},

    //     // {"random/", "random512-10-0"},
    //     // {"random/", "random512-20-0"},
    //     // {"random/", "random512-30-0"},
    //     // {"random/", "random512-40-0"},

    //     // {"room/", "8room_000"},
    //     // {"room/", "16room_000"},
    //     // {"room/", "32room_000"},
    //     // {"room/", "64room_000"},

    //     // {"street/", "Berlin_0_256"},
    //     // {"street/", "Paris_0_256"},
    //     // {"street/", "Boston_0_512"},
    //     // {"street/", "Milan_0_512"},
    //     // {"street/", "NewYork_0_512"},
    //     // {"street/", "Denver_0_1024"},
    //     // {"street/", "London_0_1024"},
    //     // {"street/", "London_2_1024"},
    //     // {"street/", "Moscow_0_1024"},
    //     // {"street/", "Shanghai_0_1024"},

    //     // {"wc3maps512/", "battleground"},
    //     // {"wc3maps512/", "darkforest"},
    //     // {"wc3maps512/", "icecrown"},
    //     // {"wc3maps512/", "thecrucible"},

    //     // {"dao/", "arena"},
    //     // {"dao/", "brc997d"},
    //     // {"dao/", "den203d"},
    //     // {"dao/", "den504d"},
    //     // {"dao/", "lak100c"},
    //     // {"dao/", "lak203d"},
    //     // {"dao/", "lak506d"},
    //     // {"dao/", "lgt604d"},
    //     // {"dao/", "orz303d"},
    //     // {"dao/", "ost002d"},
    //     // {"dao/", "arena2"},
    //     // {"dao/", "brc999d"},
    //     // {"dao/", "den204d"},
    //     // {"dao/", "den505d"},
    //     // {"dao/", "lak100d"},
    //     // {"dao/", "lak250d"},
    //     // {"dao/", "lak507d"},
    //     // {"dao/", "lgt605d"},
    //     // {"dao/", "orz304d"},
    //     // {"dao/", "ost003d"},
    //     // {"dao/", "brc000d"},
    //     // {"dao/", "combat"},
    //     // {"dao/", "den206d"},
    //     // {"dao/", "den510d"},
    //     // {"dao/", "lak100n"},
    //     // {"dao/", "lak300d"},
    //     // {"dao/", "lak510d"},
    //     // {"dao/", "orz000d"},
    //     // {"dao/", "orz500d"},
    //     // {"dao/", "ost004d"},
    //     // {"dao/", "brc100d"},
    //     // {"dao/", "combat2"},
    //     // {"dao/", "den207d"},
    //     // {"dao/", "den520d"},
    //     // {"dao/", "lak101d"},
    //     // {"dao/", "lak302d"},
    //     // {"dao/", "lak511d"},
    //     // {"dao/", "orz100d"},
    //     // {"dao/", "orz601d"},
    //     // {"dao/", "ost100d"},
    //     // {"dao/", "brc101d"},
    //     // // begin from here
    //     // {"dao/", "den000d"},
    //     // {"dao/", "den308d"},
    //     // {"dao/", "den600d"},
    //     // {"dao/", "lak102d"},
    //     // {"dao/", "lak303d"},
    //     // {"dao/", "lak512d"},
    //     // {"dao/", "orz101d"},
    //     // {"dao/", "orz700d"},
    //     // {"dao/", "ost101d"},
    //     // {"dao/", "brc200d"},
    //     // {"dao/", "den001d"},
    //     // {"dao/", "den312d"},
    //     // {"dao/", "den601d"},
    //     // {"dao/", "lak103d"},
    //     // {"dao/", "lak304d"},
    //     // {"dao/", "lak513d"},
    //     // {"dao/", "orz102d"},
    //     // {"dao/", "orz701d"},
    //     // {"dao/", "ost102d"},
    //     // {"dao/", "brc201d"},
    //     // {"dao/", "den005d"},
    //     // {"dao/", "den400d"},
    //     // {"dao/", "den602d"},
    //     // {"dao/", "lak104d"},
    //     // {"dao/", "lak307d"},
    //     // {"dao/", "lak514d"},
    //     // {"dao/", "orz103d"},
    //     // {"dao/", "orz702d"},
    //     // {"dao/", "oth000d"},
    //     // {"dao/", "brc202d"},
    //     // {"dao/", "den009d"},
    //     // {"dao/", "den401d"},
    //     // {"dao/", "den900d"},
    //     // {"dao/", "lak105d"},
    //     // {"dao/", "lak308d"},
    //     // {"dao/", "lak515d"},
    //     // {"dao/", "orz105d"},
    //     // {"dao/", "orz703d"},
    //     // {"dao/", "oth001d"},
    //     // {"dao/", "brc203d"},
    //     // {"dao/", "den011d"},
    //     // {"dao/", "den403d"},
    //     // {"dao/", "den901d"},
    //     // {"dao/", "lak106d"},
    //     // {"dao/", "lak400d"},
    //     // {"dao/", "lak519d"},
    //     // {"dao/", "orz106d"},
    //     // {"dao/", "orz704d"},
    //     // {"dao/", "oth999d"},
    //     // {"dao/", "brc204d"},
    //     // {"dao/", "den012d"},
    //     // {"dao/", "den404d"},
    //     // {"dao/", "den998d"},
    //     // {"dao/", "lak107d"},
    //     // {"dao/", "lak401d"},
    //     // {"dao/", "lak526d"},
    //     // {"dao/", "orz107d"},
    //     // {"dao/", "orz800d"},
    //     // {"dao/", "rmtst"},
    //     // {"dao/", "brc300d"},
    //     // {"dao/", "den020d"},
    //     // {"dao/", "den405d"},
    //     // {"dao/", "hrt000d"},
    //     // {"dao/", "lak108d"},
    //     // {"dao/", "lak403d"},
    //     // {"dao/", "lgt101d"},
    //     // {"dao/", "orz200d"},
    //     // {"dao/", "orz900d"},
    //     // {"dao/", "rmtst01"},
    //     // {"dao/", "brc501d"},
    //     // {"dao/", "den101d"},
    //     // {"dao/", "den407d"},
    //     // {"dao/", "hrt001d"},
    //     // {"dao/", "lak109d"},
    //     // {"dao/", "lak404d"},
    //     // {"dao/", "lgt300d"},
    //     // {"dao/", "orz201d"},
    //     // {"dao/", "orz901d"},
    //     // {"dao/", "rmtst03"},
    //     // {"dao/", "brc502d"},
    //     // {"dao/", "den200d"},
    //     // {"dao/", "den408d"},
    //     // {"dao/", "hrt002d"},
    //     // {"dao/", "lak110d"},
    //     // {"dao/", "lak405d"},
    //     // {"dao/", "lgt600d"},
    //     // {"dao/", "orz203d"},
    //     // {"dao/", "orz999d"},
    //     // {"dao/", "brc503d"},
    //     // {"dao/", "den200n"},
    //     // {"dao/", "den500d"},
    //     // {"dao/", "hrt201d"},
    //     // {"dao/", "lak200d"},
    //     // {"dao/", "lak503d"},
    //     // {"dao/", "lgt601d"},
    //     // {"dao/", "orz300d"},
    //     // {"dao/", "ost000a"},
    //     // {"dao/", "brc504d"},
    //     // {"dao/", "den201d"},
    //     // {"dao/", "den501d"},
    //     // {"dao/", "hrt201n"},
    //     // {"dao/", "lak201d"},
    //     // {"dao/", "lak504d"},
    //     // {"dao/", "lgt602d"},
    //     // {"dao/", "orz301d"},
    //     // {"dao/", "ost000t"},
    //     // {"dao/", "brc505d"},
    //     // {"dao/", "den202d"},
    //     // {"dao/", "den502d"},
    //     // {"dao/", "isound1"},
    //     // {"dao/", "lak202d"},
    //     // {"dao/", "lak505d"},
    //     // {"dao/", "lgt603d"},
    //     // {"dao/", "orz302d"},
    //     // {"dao/", "ost001d"},
    // };

    // std::string alg_name = "VG2";
    // std::string map_ext = ".map";
    // std::string map_dir = "data/";
    // std::string scen_ext = ".map.scen";
    // std::string scen_dir = map_dir;
    // std::string result_dir = "results/";
    // std::string result_ext = ".results";
    // std::string proc_dir = "results/VG2/";
    // std::string proc_ext = ".combinations";
    // unsigned int expt_num = 0;
    // for (auto &map_datum : map_data)
    // {
    //     std::string sub_dir = map_datum[0];
    //     std::string map_name = map_datum[1];
    //     std::string scen_name = map_name;
    //     std::string result_name = alg_name + "_" + map_name + "." + alg_type;
    //     std::string vg_proc_file = proc_dir + sub_dir + map_name + "." + alg_type + proc_ext;

    //     P2D::OcGrid M(map_dir + sub_dir + map_name + map_ext);

    //     alg VG2(&M, vg_proc_file, true, 8);

    //     if (!build_map_only)
    //     {
    //         P2D::Benchmark::Experiments expts = P2D::Benchmark::readScenariosFromFile(result_name, scen_dir + sub_dir + scen_name + scen_ext);
    //         P2D::Benchmark::runScenarios<alg>(expts, expt_num, 1, &VG2);
    //         if (expt_num == 0)
    //             P2D::Benchmark::writeResultsToFile(expts, result_dir + sub_dir + result_name + result_ext);
    //     }
    // }

    return 0;
}