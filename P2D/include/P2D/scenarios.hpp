#include <iomanip>
#include <ostream>
#include <vector>
#include <fstream>
#include <filesystem>

#include "types.hpp"
#include "math.hpp"
#include "Vec2.hpp"
#include "grid.hpp"

#pragma once
namespace P2D
{
    struct Scenario
    {
        V2 start = V2(0, 0), goal = V2(0, 0);
        float_t cost = NAN, nanosec = NAN;
        std::vector<V2> path = {};
        Scenario(const int_t &start_x, const int_t &start_y, const int_t &goal_x, const int_t &goal_y) : start(start_x, start_y), goal(goal_x, goal_y) {}

        std::string repr(const int type = 0) const
        {
            std::stringstream ss;
            ss << std::fixed;
            ss << "( ";
            ss << this->start;
            ss << " ) to ( ";
            ss << this->goal;
            ss << " ) $( ";
            ss << std::setw(7) << this->cost;
            ss << " ) size( ";
            ss << std::setw(3) << this->path.size();
            ss << " ) elapsed( ";
            ss << std::setprecision(0) << this->nanosec;
            ss << " ns )";

            if (type != 0)
            {
                ss << " Path( ";
                for (const V2 &coord : this->path)
                {
                    ss << coord;
                    ss << "; ";
                }
                ss << " )";
            }

            return ss.str();
        }
        friend std::ostream &operator<<(std::ostream &out, Scenario const &scen)
        {
            out << scen.repr();
            return out;
        }
    };
    struct Scenarios
    {
        std::filesystem::path fp_name, fp_alg;
        std::filesystem::path fp_scen, fp_map, fp_results;
        std::vector<Scenario> scens = {};
        // Scenarios(const std::string &dir, const std::string &name, const std::string &alg)
        //     : fp_dir(dir), fp_name(name), fp_alg(alg)
        // {
        //     fp_scen = fp_scen / fp_dir / fp_name;
        //     fp_scen.replace_extension("map.scen");
        //     fp_map = fp_map / fp_dir / fp_name;
        //     fp_map.replace_extension("map");
        //     fp_results = fp_results / fp_dir / fp_name;
        //     fp_results.replace_extension(fp_alg.string() + ".results");
        // }
    };

    inline void getScenarios(Scenarios &scens)
    {
        assert(scens.fp_name != "");
        std::ifstream file(scens.fp_scen);
        if (!file)
            throw std::runtime_error("getScenarios: Cannot access scenario file '" + scens.fp_scen.string() + "'");

        // read version
        std::string tmp_s;
        int version;
        file >> tmp_s >> version;
        if (version != 1)
            throw std::runtime_error("getScenarios: Version " + std::to_string(version) + " not recognized for scenario file");

        int_t sx, sy, tx, ty;
        while (file >> tmp_s >> tmp_s >> tmp_s >> tmp_s >> sy >> sx >> ty >> tx >> tmp_s) // flipped to preserve rotational direction while displaying map properly (as in the .map)
            scens.scens.emplace_back(sx, sy, tx, ty);
        file.close();
    }

    inline void getMap(Grid &grid, std::filesystem::path fp_map)
    {
        // read from Benchmark
        std::ifstream file(fp_map);
        if (!file)
            throw std::runtime_error("initBenchmark: '" + fp_map.string() + "' cannot be found!");

        int_t size_x, size_y;
        std::string tmp;
        file >> tmp >> tmp >> tmp >> size_x >> tmp >> size_y >> tmp; // x: height, y:width
        if (size_x <= 0 || size_y <= 0)
            throw std::runtime_error("initBenchmark: size is invalid: (" + std::to_string(size_x) + "," + std::to_string(size_y) + ")'");

        assert(size_x > 0);
        assert(size_y > 0);
        bool *data = {new bool[size_x * size_y]};

        char c;
        int k = 0;
        while (file >> c)
        {
            if (std::isspace(c))
                assert(false);
            data[k++] = (c != '.' && c != 'G' && c != 'S');
        }

        file.close();

        grid.init(data, {size_x, size_y});
        delete[] data;
    }

    inline void writeResults(const Scenarios &scens)
    {
        if (scens.scens.empty())
            std::cout << "writeResults: Nothing to write because there are no scenarios" << std::endl;

        std::filesystem::create_directories(scens.fp_results.parent_path()); // create directory if it doesn't exist

        std::ofstream file(scens.fp_results, std::ios::out);
        if (!file)
            throw std::runtime_error("writeResults: Cannot write to '" + scens.fp_results.string() + "'");

        for (const Scenario &scen : scens.scens)
        {
            // scen.start, scen.goal, scen.path, scen.nanosec, scen.cost
            file << std::fixed << std::setprecision(0) << scen.nanosec;
            for (V2 coord : scen.path)
                file << "\t" << coord[0] << "\t" << coord[1];
            file << std::endl;
        }
        std::cout << "writeResults: Wrote results to '" << scens.fp_results << "'" << std::endl;
    }

    // from expt.sh, setups a vector of scenarios, with each element representing a benchmark map and its scenarios.
    // std::vector<Scenarios> getExperiment(const std::string &alg)
    // {
    //     std::ifstream file("expt.sh");
    //     if (!file)
    //         throw std::runtime_error("expt.sh does not exist");

    //     std::vector<Scenarios> expt;
    //     std::string line;
    //     while (std::getline(file, line))
    //     {
    //         std::istringstream iss(line);
    //         std::string dir, name;
    //         if (!(iss >> dir >> name))
    //             continue; // error

    //         if (dir[0] == '#')
    //             continue; // ignore comments

    //         std::cout << dir << "/" << name << std::flush;
    //         expt.emplace_back(dir, name, alg);
    //         getScenarios(expt.back());
    //         std::cout << " has " << expt.back().scens.size() << " scenarios " << std::endl;
    //     }

    //     return expt;
    // }

    // 0 for all, 1 for first scen, 2 for 2nd scen etc.
    template <class T>
    inline void run(T *const &alg, Scenarios &scens, int scen_num)
    {
        if (scen_num >= int(scens.scens.size()))
            throw std::runtime_error("run: scen_num (" + std::to_string(scen_num) + ") is >= number of scenarios (" + std::to_string(scens.scens.size()) + ")");

        int scen_end;
        if (scen_num < 0)
        {
            scen_end = scens.scens.size();
            scen_num = 0;
        }
        else
            scen_end = scen_num + 1;

        std::cout << "=== runScenarios: " << scens.fp_name.string() << " ===" << std::endl;

        auto time_zero = std::chrono::high_resolution_clock::now();
        float_t avg_nanosec = 0;

        for (; scen_num < scen_end; ++scen_num)
        {
            Scenario &scen = scens.scens[scen_num];
            auto time_start = std::chrono::high_resolution_clock::now();
            scen.path = alg->run(scen.start, scen.goal);
            auto time_end = std::chrono::high_resolution_clock::now();

            // get time elapsed
            std::chrono::duration<float_t, std::nano> dur = time_end - time_start;
            scen.nanosec = dur.count();
            avg_nanosec += scen.nanosec;

            // calculate cost
            scen.cost = norm(scen.path);

            // get total duration elapsed
            std::chrono::duration<float_t, std::ratio<60, 1>> dur_total = std::chrono::high_resolution_clock::now() - time_zero;

            // feedback
            std::cout << scens.fp_name.string();
            std::cout << "." << scens.fp_alg.string();
            std::cout << std::fixed;
            std::cout << " [ ";
            std::cout << std::setw(5) << scen_num;
            std::cout << " / ";
            std::cout << std::setw(5) << scens.scens.size() - 1;
            std::cout << " ]: ( ";
            std::cout << std::setw(7) << dur_total.count();
            std::cout << " min ) ";
            std::cout << scen;
            if (scen.path.empty())
                std::cout << " --- Not solvable";
            std::cout << std::endl;
        }
    }
}