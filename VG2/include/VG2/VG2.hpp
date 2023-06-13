#include <unordered_map>
#include <filesystem>
#include <fstream>

#include "P2D/P2D.hpp"
#include "node.hpp"

#pragma once
#define FILE_DELIMITER -1
#define FILE_SZ sizeof(mapkey_t)
namespace P2D::VG2
{
    template <bool diag_block = true>
    class VG2
    {
    private:
        std::unordered_map<mapkey_t, Corner> crns;
        Nodes nodes;
        Grid *const grid;
        OpenList<Node> open_list;
        Los<diag_block> los;

        // finds convex corners from the map and returns vertex keys of the convex corners
        std::vector<mapkey_t> _setupFindCorners()
        {
            assert(grid != nullptr);

            std::vector<mapkey_t> crn_keys;
            const mapkey_t rkv_y = grid->getRelKey<false>(2);
            const mapkey_t rkc_y = grid->getRelKey<true>(2);

            int_t max_vx = grid->getBoundary<false>(0) - 2;
            int_t max_vy = grid->getBoundary<false>(2) - 2;
            std::cout << "Finding Corners..." << std::flush;
            for (int_t vx = 1; vx <= max_vx; ++vx)
            {
                std::cout << "\rFinding Corners... " << vx << "/" << max_vx << " rows (height) of x vertices" << std::flush;
                mapkey_t kv = grid->coordToKey<false>(vx, 1);

                // fill window for back left and back right
                mapkey_t kc_l = grid->addKeyToRelKey(kv, grid->getCellRelKey(5, vx));
                mapkey_t kc_r = grid->addKeyToRelKey(kv, grid->getCellRelKey(7, vx));
                char window = (grid->isOc(kc_l) << 1) | (grid->isOc(kc_r));

                for (int_t vy = 1; vy <= max_vy; ++vy)
                {
                    // shift window left and filter
                    window <<= 2;
                    window &= 0b1111; // in format bl,br,fl,fr

                    // fill window for front left nad front right
                    kc_l = grid->addKeyToRelKey(kc_l, rkc_y);
                    kc_r = grid->addKeyToRelKey(kc_r, rkc_y);
                    window |= (grid->isOc(kc_l) << 1) | grid->isOc(kc_r);

                    // identify convex corner at current vertex location
                    switch (window)
                    {
                    case 0b1000:
                    case 0b0100:
                    case 0b0010:
                    case 0b0001:
                        // std::cout << vx << "," << vy << "; " << std::flush;
                        crn_keys.push_back(kv);
                        break;
                    case 0b1001:
                    case 0b0110:
                        if constexpr (diag_block == false)
                        {
                            // std::cout << vx << "," << vy << "; " << std::flush;
                            crn_keys.push_back(kv);
                        }
                        break;
                    default:
                        break;
                    }

                    // update vertex key by moving to next y
                    kv = grid->addKeyToRelKey(kv, rkv_y);
                }
            }

            std::cout << std::endl;
            return crn_keys;
        }

        std::vector<std::vector<mapkey_t>> _setupFindCombinations(std::vector<mapkey_t> &crn_keys)
        {
            size_t i = 0, num_combinations = 0;
            std::vector<std::vector<mapkey_t>> combinations;
            std::cout << "Testing LOS..." << std::flush;
            for (auto key_a_ = crn_keys.begin(); key_a_ != crn_keys.end(); ++key_a_)
            {
                std::cout << "\rTesting LOS... " << ++i << "/" << crn_keys.size() << " corners" << std::flush;

                const V2 crn_coord_a = grid->keyToCoord<false>(*key_a_);
                std::vector<mapkey_t> keys_visible;
                for (auto key_b_ = key_a_ + 1; key_b_ != crn_keys.end(); ++key_b_)
                { // test the pair
                    const V2 crn_coord_b = grid->keyToCoord<false>(*key_b_);
                    if (los.template cast<false>(*key_a_, crn_coord_a, *key_b_, crn_coord_b))
                    { // visible
                        keys_visible.push_back(*key_b_);
                        ++num_combinations;
                    }
                }

                if (keys_visible.empty() == false)
                {
                    keys_visible.push_back(*key_a_);
                    combinations.emplace_back(std::move(keys_visible));
                }
            }
            std::cout << std::endl;
            std::cout << "There were " << num_combinations << " combinations (pairs of corners, w/o repeat)" << std::endl;
            return combinations;
        }

        void _setupWriteCombinations(const std::filesystem::path fp_vg, const std::vector<std::vector<mapkey_t>> &combinations)
        {
            std::filesystem::create_directories(fp_vg.parent_path()); // create directory if it doesn't exist

            std::ofstream file(fp_vg, std::ios::out | std::ios::binary);
            if (!file)
                throw std::runtime_error("writeCombinations: Cannot write to '" + fp_vg.string() + "'");

            // ======= Write map size =============
            const V2 &size_vert = grid->getSize<false>();
            const mapkey_t size_vert_x = size_vert.x;
            const mapkey_t size_vert_y = size_vert.y;
            file.write((char *)&size_vert_x, FILE_SZ);
            file.write((char *)&size_vert_y, FILE_SZ);

            // ======= Write combinations ===========
            for (const std::vector<mapkey_t> &combination : combinations)
            {
                for (const mapkey_t &key : combination)
                    file.write((char *)&key, FILE_SZ);
                mapkey_t delim = FILE_DELIMITER;
                file.write((char *)&delim, FILE_SZ);
            }

            file.close();
            std::cout << "writeCombinations: Wrote results to '" << fp_vg << "'" << std::endl;
        }

        std::vector<std::vector<mapkey_t>> _setupReadCombinations(const std::filesystem::path fp_vg)
        {
            std::cout << "readCombinations: Reading .vg file from '" << fp_vg.string() << "'... " << std::endl;

            std::ifstream file(fp_vg, std::ios::ate | std::ios::binary);
            if (!file)
                throw std::runtime_error("readCombinations: Cannot read '" + fp_vg.string() + "'");

            auto file_size = file.tellg();
            file.seekg(0);

            // ======= read map size and validate =============
            mapkey_t size_vert_x, size_vert_y;
            file.read((char *)&size_vert_x, FILE_SZ);
            file.read((char *)&size_vert_y, FILE_SZ);
            if (grid->getSize<false>() != V2(size_vert_x, size_vert_y))
                throw std::out_of_range("readCombinations: Read size (" + V2(size_vert_x, size_vert_y).repr() + ") is different from grid size(" + grid->getSize<false>().repr() + ")");

            // ======= Read combinations ===========
            size_t num_combinations = 0;
            std::vector<std::vector<mapkey_t>> combinations;
            while (file.tellg() < file_size)
            {
                std::vector<mapkey_t> combination;
                // read key
                while (true)
                {
                    mapkey_t key;
                    file.read((char *)&key, FILE_SZ);
                    if (key == FILE_DELIMITER)
                        break;
                    else if (file.tellg() >= file_size)
                        throw std::out_of_range("readCombinations: File did not end properly. Corrupted or wrong file.");
                    combination.push_back(key);
                    ++num_combinations;
                }
                --num_combinations;

                if (combination.empty() == false)
                    combinations.push_back(std::move(combination));

                std::cout << "\rreadCombinations: " << file.tellg() << " / " << file_size << " bytes" << std::flush;
            }
            std::cout << std::endl;
            std::cout << "There were " << num_combinations << " combinations (pairs of corners, w/o repeat)" << std::endl;

            file.close();
            return combinations;
        }

        // use when map data changes
        // opens the vg file at fp_vg and loads the vg, otherwise, builds the vg and write to vg file at fp_vg
        void setup(const std::filesystem::path fp_vg)
        {
            crns.clear();
            assert(nodes.empty() == true);
            assert(open_list.empty() == true);

            std::vector<std::vector<mapkey_t>> combinations;
            if (std::filesystem::exists(fp_vg))
            { // load the vg
                //  ========= Load from File ================
                combinations = _setupReadCombinations(fp_vg);
            }
            else
            { // build the vg and write into file

                // ========= find corners =================
                std::vector<mapkey_t> crn_keys = _setupFindCorners();

                // ========= find combinations ============
                combinations = _setupFindCombinations(crn_keys);

                //  ========= Write combinations to File ================
                _setupWriteCombinations(fp_vg, combinations);
            }

            // convert combinations into unordered map
            std::cout << "Constructing VG from combinations..." << std::flush;
            for (const std::vector<mapkey_t> &combination : combinations)
            {
                const mapkey_t &crn_key_a = combination.back();
                const V2 crn_coord_a = grid->keyToCoord<false>(crn_key_a);
                Corner *const &crn_a = &(crns.try_emplace(crn_key_a, crn_key_a, crn_coord_a).first->second);

                for (auto key_ = combination.begin(); key_ != combination.end() - 1; ++key_)
                {
                    const V2 crn_coord_b = grid->keyToCoord<false>(*key_);
                    Corner *const &crn_b = &(crns.try_emplace(*key_, *key_, crn_coord_b).first->second);
                    crn_a->neighbors.push_back(crn_b);
                    crn_b->neighbors.push_back(crn_a);
                }
            }
            std::cout << " done!" << std::endl;
        }

    public:
        VG2(Grid *const &grid, const std::filesystem::path fp_vg) : grid(grid), los(grid) { setup(fp_vg); }
        VG2 &operator=(const VG2 &) = delete; // Disallow copying
        VG2(const VG2 &) = delete;
        ~VG2() {}

        std::vector<V2> run(const V2 &p_start, const V2 &p_goal)
        {
            mapkey_t k_start = grid->coordToKey<false>(p_start);
            mapkey_t k_goal = grid->coordToKey<false>(p_goal);

            // ==== Return if direct LOS to goal =====
            if (los.template cast<false>(k_start, p_start, k_goal, p_goal))
            { // has direct los
                return {p_goal, p_start};
            }

            // ===== Create start and goal "corners" and build into vg =======
            Corner crn_start(k_start, p_start), crn_goal(k_goal, p_goal);
            for (auto crn_ = crns.begin(); crn_ != crns.end(); crn_ = std::next(crn_))
            {
                Corner &crn = crn_->second;

                if (los.template cast<false>(crn.key, crn.coord, k_start, p_start))
                    crn_start.neighbors.push_back(&crn); // has los to start

                if (los.template cast<false>(crn.key, crn.coord, k_goal, p_goal))
                {
                    crn.neighbors.push_back(&crn_goal);
                    crn_goal.neighbors.push_back(&crn);
                }
            }

            // ==== Do A* =====
            Node *node = nodes.emplace(&crn_start, nullptr, 0, norm(p_start, p_goal));
            open_list.queue(node);
            std::vector<V2> path = {};
            while (true)
            {
                // ====== Poll Node ======
                node = open_list.poll();

                // ====== Openlist empty ======
                if (node == nullptr)
                    break; // no path found;

                // ====== Path found ======
                if (node->crn == &crn_goal)
                {
                    do
                    {
                        path.push_back(node->crn->coord);
                        node = node->parent;
                    } while (node != nullptr);
                    break;
                }

                // ====== Skip if already expanded ======
                if (node->is_visited == true)
                    continue; // is visited. continue;
                node->is_visited = true;

                // ====== Queue neighbors if possible ======
                Corner *const &crn = node->crn;
                for (Corner *const &crn_nb : node->crn->neighbors)
                {
                    Node *&node_nb = crn_nb->node;
                    float_t test_g = node->g + norm(crn_nb->coord, crn->coord);

                    if (node_nb == nullptr) // emplace directly if no nodes at crn
                    {
                        node_nb = nodes.emplace(crn_nb, node, test_g, norm(crn_nb->coord, p_goal));
                        open_list.queue(node_nb);
                    }
                    else if (node_nb != nullptr)
                    { // test for g cost if a node already exists at crn
                        if (node_nb->is_visited == false && approxGt(node_nb->g, test_g) == true)
                        {                               // not visited and cheapest
                            open_list.unqueue(node_nb); // remove from ol
                            node_nb->g = test_g;
                            node_nb->parent = node;
                            node_nb->f = node_nb->g + node_nb->h;
                            open_list.queue(node_nb);
                        }
                    }
                }
            }

            // ==== Remove nodes and pointer to node from each corner ====
            open_list.clear();
            nodes.clear();

            // ===== Remove Goal from corners ========
            for (Corner *const &crn : crn_goal.neighbors)
                crn->neighbors.pop_back();

            return path;
        }
    };
}
