#include <iomanip>
#include <ostream>
#include <vector>
#include <fstream>
#include "math.hpp"
#include "Vec2.hpp"
#include "grid.hpp"

namespace RR2star
{
    struct Scenario
    {
        V2 start = V2(0, 0), goal = V2(0, 0);
        float_t cost = NAN, microseconds = NAN;
        std::vector<V2> path;
        Scenario(const int_t &start_x, const int_t &start_y, const int_t &goal_x, const int_t &goal_y) : start(start_x, start_y), goal(goal_x, goal_y) {}

        friend std::ostream &operator<<(std::ostream &out, Scenario const &scen)
        {
            out << std::fixed;
            out << "( ";
            out << scen.start;
            out << " ) to ( ";
            out << scen.goal;
            out << " ) $( ";
            out << std::setw(7) << scen.cost;
            out << " ) size( ";
            out << std::setw(3) << scen.path.size();
            out << ") elapsed( ";
            out << scen.microseconds;
            out << " us ) Path( ";
            for (const V2 &coord : scen.path)
            {
                out << coord;
                out << "; ";
            }
            out << " )";
            return out;
        }
    };
    struct Scenarios
    {
        std::string name = "", filepath_scen = "", filepath_map = "", filepath_results = "";
        std::vector<Scenario> scens;
        Scenarios(const std::string &name, const std::string &filepath_scen, const std::string &filepath_map, const std::string &filepath_results)
            : name(name), filepath_scen(filepath_scen), filepath_map(filepath_map), filepath_results(filepath_results) {}
    };

    void getScenarios(Scenarios &scens)
    {
        assert(scens.name != "");
        std::ifstream file(scens.filepath_scen);
        if (!file)
            throw std::runtime_error("getScenarios: Cannot access scenario file '" + scens.filepath_scen + "'");

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

    void getMap(Grid &grid, Scenarios &scens)
    {
        // read from Benchmark
        std::ifstream file(scens.filepath_map);
        if (!file)
            throw std::runtime_error("initBenchmark: '" + scens.filepath_map + "' cannot be found!");

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

    void writeResults(Scenarios &scens)
    {
        if (scens.scens.empty())
            std::cout << "writeResults: Nothing to write because there are no scenarios" << std::endl;

        std::ofstream file;
        (scens.filepath_scen, std::ios::out | std::ios::binary);
        if (!file)
            throw std::runtime_error("writeResults: Cannot write to '" + scens.filepath_results + "'");

        for (Scenario &scen : scens.scens)
        {
            // scen.start, scen.goal, scen.path, scen.microseconds, scen.cost
            file << std::fixed << std::setprecision(3) << scen.microseconds;
            for (V2 coord : scen.path)
                file << "\t" << coord[0] << "\t" << coord[1];
        }
        std::cout << "writeResults: Wrote results to '" << scens.filepath_results << "'";
    }
}