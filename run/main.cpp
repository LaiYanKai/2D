#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <assert.h>

#include "P2D/P2D.hpp"
// #include "VG2/VG2.hpp"
// #include "ANYA2/ANYA2.hpp"
// #include "TS2/TS2.hpp"
#include "R2/R2.hpp"
struct Args
{
    std::filesystem::path results_dir = "results", map_dir = "data", scen_dir = "data";
    std::vector<std::filesystem::path> names, algs;
    std::vector<int> ids;
};

inline bool isOption(const std::string &test) { return test.size() > 2 && test.substr(0, 2) == "--"; }

bool isValidOption(const std::string &option)
{
    return (option == "--results_dir" || option == "--map_dir" || option == "--scen_dir" || option == "--names" || option == "--algs" || option == "--ids");
    // else
    //     throw std::runtime_error("Invalid option: " + option);
}
void parseOption(Args &args, const std::string &option, int &arg_idx, const int &argc, char **&argv)
{
    assert(isOption(option));

    bool specified = false;
    while (1)
    {
        if (arg_idx >= argc)
            break;

        std::string arg(argv[arg_idx]);
        if (isOption(arg) == true)
        {
            if (isValidOption(arg) == false)
                throw std::runtime_error("Invalid option: " + arg);
            parseOption(args, arg, ++arg_idx, argc, argv);
            break;
        }

        if (specified == true)
            std::cout << "[WARN]: " << option << " is specified more than once. The last value will be used." << std::endl;

        if (option == "--results_dir")
        {
            args.results_dir = arg;
            specified = true;
        }
        else if (option == "--map_dir")
        {
            args.map_dir = arg;
            specified = true;
        }
        else if (option == "--scen_dir")
        {
            args.scen_dir = arg;
            specified = true;
        }
        else if (option == "--names")
        {
            args.names.emplace_back(arg);
        }
        else if (option == "--algs")
        {
            if (arg == "VG2B" || arg == "VG2N" || arg == "TS2B" || arg == "TS2N" || arg == "ANYA2B" || arg == "ANYA2N" || arg == "R2" || arg == "R2E")
                args.algs.emplace_back(arg);
            else
                throw std::out_of_range("--algs must be one of the following: VG2B, VG2N, TS2B, TS2N, ANYA2B, ANYA2N, R2, R2E");
        }
        else if (option == "--ids")
        {
            if (args.ids.empty() == true || args.ids.back() != -1)
            {
                int id = std::stoi(arg);
                if (id < 0)
                    args.ids = {-1};
                else
                    args.ids.push_back(id);
            }
        }

        ++arg_idx;
    }
}
void parseOptions(Args &args, const int &argc, char **&argv)
{
    int arg_idx = 1;
    std::string option(argv[arg_idx]);
    if (isOption(option) == true)
    {
        if (isValidOption(option) == false)
            throw std::runtime_error("Invalid option: " + option);
        parseOption(args, option, ++arg_idx, argc, argv);
    }
    else
        throw std::runtime_error(std::string("Must start with a '--' option, and not '") + option + "'");
}

int main(int argc, char *argv[])
{
    Args args;
    parseOptions(args, argc, argv);

    if (args.names.empty() == true)
        throw std::out_of_range("At least one file must be specified for --names. If a pair of scenario file 'arena.map.scen' and map file 'arena.map' are to be run, and the files are located in the sub-directory 'dao', then the argument is: '--name dao/arena'. Specify other pairs with spaces: '--name dao/arena da2/ht_mansion2b sc1/Aurora'");

    if (args.algs.empty() == true)
        throw std::out_of_range("At least one file must be specified for --algs. If 'TS2B' is to be run, the argument is '--algs TS2B'. Multiple algorithms can be specified: '--algs TS2B ANYA2B VG2B'");

    if (args.ids.empty() == true)
        args.ids = {-1};

    // check if scen file and map file exists
    std::vector<std::array<std::filesystem::path, 3>> fpaths;
    for (const std::filesystem::path name : args.names)
    {
        std::filesystem::path fp_map = args.map_dir / name;
        fp_map.replace_extension("map");
        if (std::filesystem::exists(fp_map) == false)
            throw std::out_of_range("Map file " + fp_map.string() + " does not exist!");
        std::filesystem::path fp_scen = args.map_dir / name;
        fp_scen.replace_extension("map.scen");
        if (std::filesystem::exists(fp_scen) == false)
            throw std::out_of_range("Scen file " + fp_scen.string() + " does not exist!");

        fpaths.push_back({fp_map, fp_scen, name});
    }

    for (const auto &fpath : fpaths)
    {
        P2D::Grid grid;
        P2D::getMap(grid, fpath[0]);

        for (const auto &alg : args.algs)
        {
            P2D::Scenarios scens;
            scens.fp_map = fpath[0];
            scens.fp_scen = fpath[1];
            scens.fp_name = fpath[2];
            scens.fp_alg = alg;
            scens.fp_results = args.results_dir / scens.fp_name;
            scens.fp_results.replace_extension(alg.string() + ".results");
            P2D::getScenarios(scens);

            // if (alg == "TS2B")
            // {
            //     P2D::TS2::TS2<true> alg(&grid);
            //     for (int id : args.ids)
            //         P2D::run(&alg, scens, id);
            // }
            // else if (alg == "TS2N")
            // {
            //     P2D::TS2::TS2<false> alg(&grid);
            //     for (int id : args.ids)
            //         P2D::run(&alg, scens, id);
            // }
            // else if (alg == "VG2B")
            // {
            //     std::filesystem::path fp_vg = "VG2/combinations";
            //     fp_vg = fp_vg / scens.fp_name;
            //     fp_vg.replace_extension(".VG2B.combinations");

            //     P2D::VG2::VG2<true> alg(&grid, fp_vg);
            //     for (int id : args.ids)
            //         P2D::run(&alg, scens, id);
            // }
            // else if (alg == "VG2N")
            // {
            //     std::filesystem::path fp_vg = "VG2/combinations";
            //     fp_vg = fp_vg / scens.fp_name;
            //     fp_vg.replace_extension(".VG2N.combinations");

            //     P2D::VG2::VG2<false> alg(&grid, fp_vg);
            //     for (int id : args.ids)
            //         P2D::run(&alg, scens, id);
            // }
            // else if (alg == "ANYA2B")
            // {
            //     P2D::ANYA2::ANYA2<true> alg(&grid);
            //     for (int id : args.ids)
            //         P2D::run(&alg, scens, id);
            // }
            // else if (alg == "ANYA2N")
            // {
            //     P2D::ANYA2::ANYA2<false> alg(&grid);
            //     for (int id : args.ids)
            //         P2D::run(&alg, scens, id);
            // }
            if (alg == "R2")
            {

            }
            else if (alg == "R2E")
            {
            }
            else
                throw std::out_of_range("Invalid algorithm: " + alg.string());

            if (args.ids.front() == -1)
                P2D::writeResults(scens);
        }
    }
    return 0;
}