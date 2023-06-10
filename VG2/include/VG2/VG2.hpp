#include "P2D/P2D.hpp"
#include "node.hpp"

#pragma once
namespace P2D::VG2
{
    // template <bool block = false>
    // class VG2
    // {
    //     using LosType = Los2::Simple<block>;

    // private:
    //     const int_t FILE_DEL = std::numeric_limits<int_t>::min();
    //     OcGrid<false> *oc_grid;
    //     struct Combination
    //     {
    //         mapkey_t crn_key;
    //         std::vector<mapkey_t> nb_crn_keys;
    //         Combination() : crn_key(0), nb_crn_keys() {}
    //         Combination(mapkey_t crn_key) : crn_key(crn_key), nb_crn_keys() {}
    //     };
    //     std::unordered_map<mapkey_t, Corner> corners;

    //     void threadCombLOS(unsigned int *th_idx1, int *th_state, std::vector<Combination> *combs, std::mutex *mtx_state)
    //     {
    //         std::unique_lock<std::mutex> lock(*mtx_state, std::defer_lock);

    //         int state = *th_state;
    //         int idx1 = *th_idx1;
    //         LosType los(oc_grid);
    //         const unsigned int combs_size = combs->size();
    //         while (state == 1)
    //         {
    //             Combination &comb = (*combs)[idx1];
    //             mapkey_t &key1 = comb.crn_key; // size of combs and keys do not change while threading, so no lock
    //             V2 coord1 = oc_grid->keyToCoord<V2>(key1);
    //             for (unsigned int idx2 = idx1 + 1; idx2 < combs_size; ++idx2)
    //             {
    //                 mapkey_t &key2 = (*combs)[idx2].crn_key; // size of combs and keys do not change while threading, so no lock
    //                 V2 coord2 = oc_grid->keyToCoord<V2>((*combs)[idx2].crn_key);

    //                 if (los.cast(coord1, coord2) == Los2::reached)
    //                     comb.nb_crn_keys.push_back(key2);
    //             }

    //             // signal complete
    //             lock.lock();
    //             *th_state = 0; // inactive
    //             lock.unlock();

    //             // wait until active
    //             while (1)
    //             {
    //                 lock.lock();
    //                 state = *th_state;
    //                 idx1 = *th_idx1;
    //                 lock.unlock();
    //                 if (state == 1)
    //                     break;
    //                 else if (state == -1)
    //                     return;
    //             } // wait until change
    //         }
    //     }
    //     void buildVGfromCombs(const std::vector<Combination> &combs)
    //     {
    //         corners.clear();
    //         corners.reserve(combs.size());

    //         // Instantiate corners
    //         for (const Combination &comb : combs)
    //             corners.try_emplace(comb.crn_key, comb.crn_key, oc_grid->keyToCoord<V2>(comb.crn_key));

    //         // Link the corners
    //         unsigned int prog = 0;
    //         for (const Combination &comb : combs)
    //         {
    //             const mapkey_t &key1 = comb.crn_key;
    //             const std::vector<mapkey_t> &keys2 = comb.nb_crn_keys;
    //             Corner &crn1 = corners.at(key1);
    //             std::vector<Corner *> &nbs1 = crn1.neighbors;
    //             for (const mapkey_t &key2 : keys2)
    //             {
    //                 Corner &crn2 = corners.at(key2);
    //                 std::vector<Corner *> &nbs2 = crn2.neighbors;
    //                 nbs1.push_back(&crn2);
    //                 nbs2.push_back(&crn1);
    //             }

    //             std::cout << "\rbuildVGFromCombs: Building VG " << ++prog << "/" << combs.size() << std::flush;
    //         }
    //         std::cout << std::endl;
    //     }
    //     bool writeCombsToFile(const std::vector<Combination> &combs, std::string output_path)
    //     {
    //         // inits
    //         std::ofstream file(output_path, std::ios::out | std::ios::binary);
    //         if (!file)
    //         {
    //             std::cout << "writeCombsToFile: Error writing to file '" << output_path << "'" << std::endl;
    //             return false;
    //         }

    //         // write version.
    //         char version = 1;
    //         file.write((char *)&version, 1);

    //         // write map
    //         unsigned int num_comb = 0;
    //         unsigned int num_complete = 0;
    //         for (const Combination &comb : combs)
    //         {
    //             const mapkey_t &key1 = comb.crn_key;
    //             const std::vector<mapkey_t> &keys2 = comb.nb_crn_keys;

    //             int_t x, y;
    //             oc_grid->keyToCoord(key1, x, y);
    //             file.write((char *)&x, sizeof(int_t));
    //             file.write((char *)&y, sizeof(int_t));
    //             for (const mapkey_t &key2 : keys2)
    //             {
    //                 oc_grid->keyToCoord(key2, x, y);
    //                 file.write((char *)&x, sizeof(int_t));
    //                 file.write((char *)&y, sizeof(int_t));
    //                 ++num_comb;
    //             }
    //             file.write((char *)&FILE_DEL, sizeof(int_t));

    //             std::cout << "\rwriteCombsToFile: Saving '" << output_path << "' " << ++num_complete << "/" << combs.size() << std::flush;
    //         }
    //         std::cout << std::endl;
    //         std::cout << "writeCombsToFile: Combinations: " << num_comb << std::endl;
    //         file.close();

    //         return true;
    //     }
    //     bool readCombsFromFile(std::vector<Combination> &combs, std::string input_path)
    //     {
    //         std::ifstream file(input_path, std::ios::ate | std::ios::binary);
    //         if (!file)
    //         {
    //             std::cout << "readCombsFromFile: Cannot access file '" << input_path << "' containing preprocessed VG" << std::endl;
    //             return false;
    //         }

    //         unsigned int num_data = file.tellg();
    //         file.seekg(0);

    //         // read version
    //         char version;
    //         file.read((char *)&version, sizeof(char));
    //         if (version != 1)
    //         {
    //             std::cout << "readCombsFromFile: Version " << version << " not recognized for preprocessed VG file" << std::endl;
    //             return false;
    //         }

    //         // read all keys
    //         unsigned int num_combs = 0;
    //         combs.clear();
    //         while (file.tellg() < num_data)
    //         {
    //             // read key
    //             int_t x, y;
    //             file.read((char *)&x, sizeof(int_t));
    //             file.read((char *)&y, sizeof(int_t));

    //             combs.emplace_back(oc_grid->coordToKey(x, y));
    //             std::vector<mapkey_t> &nb_crn_keys = combs.back().nb_crn_keys;

    //             while (1)
    //             {
    //                 file.read((char *)&x, sizeof(int_t));
    //                 if (x == FILE_DEL)
    //                     break;
    //                 file.read((char *)&y, sizeof(int_t));
    //                 nb_crn_keys.push_back(oc_grid->coordToKey(x, y));
    //                 num_combs++;
    //             }
    //             nb_crn_keys.shrink_to_fit();

    //             std::cout << "\rreadCombsFromFile: Reading file " << file.tellg() << "/" << num_data << " bytes " << std::flush;
    //         }
    //         std::cout << std::endl;
    //         std::cout << "readCombsFromFile: Combinations: " << num_combs << std::endl;
    //         file.close();

    //         combs.shrink_to_fit();

    //         return true;
    //     }

    //     void processNb(Corner *nb_crn, Node *node, const V2 &end, std::unordered_map<mapkey_t, Node> &nodes, OpenList &open_list)
    //     {
    //         auto emp_res = nodes.try_emplace(nb_crn->key, nb_crn, std::move(norm(end, nb_crn->coord)), open_list.end());
    //         Node *nb_node = &(emp_res.first->second);

    //         // find the cost
    //         float_t g = norm(nb_crn->coord, node->crn->coord) + node->g;

    //         // if cheaper, modify nb node and push to open
    //         if (g < nb_node->g)
    //         {
    //             if (nb_node->open_ != open_list.end())
    //                 open_list.erase(nb_node->open_); // remove from open list if it is there
    //             nb_node->g = g;
    //             nb_node->f = nb_node->h + g;
    //             nb_node->parent = node;
    //             open_list.push(nb_node);
    //         }
    //     }
    //     void threadLOS(Corner **th_from_crn, Corner **th_to_crn, char *th_reached, int *th_state, std::mutex *mtx_state)
    //     {
    //         std::unique_lock<std::mutex> lock(*mtx_state, std::defer_lock);
    //         int state = *th_state;
    //         V2 from_coord = (*th_from_crn)->coord;
    //         V2 to_coord = (*th_to_crn)->coord;
    //         LosType los(oc_grid);
    //         while (state == 1)
    //         {
    //             const Los2::State res = los.cast(from_coord, to_coord);

    //             lock.lock();
    //             *th_state = 0; // inactive
    //             *th_reached = res == Los2::reached;
    //             lock.unlock();

    //             // wait until active
    //             while (1)
    //             {
    //                 lock.lock();
    //                 state = *th_state;
    //                 from_coord = (*th_from_crn)->coord;
    //                 to_coord = (*th_to_crn)->coord;
    //                 lock.unlock();
    //                 if (state == 1)
    //                     break;
    //                 else if (state == -1)
    //                     return;
    //             } // wait until change
    //         }
    //     }

    // public:
    //     unsigned int num_threads; // for checking LOS if begin or end does not exist in the existing VG

    //     VG2(OcGrid<false> *oc_grid, std::string file_path, bool try_load = false, unsigned int num_threads = 1) : oc_grid(oc_grid), num_threads(num_threads)
    //     {
    //         if (try_load)
    //         {
    //             if (buildVGFromFile(oc_grid, file_path))
    //                 return;
    //             else
    //                 std::cout << "VG2: Build directly from oc_grid, as file is not accessible!" << std::endl;
    //         }
    //         if (!buildVGFromOcGrid(oc_grid, file_path))
    //             std::cout << "VG2: Build failed!" << std::endl;
    //         std::cout << "VG2: Build VG successful!" << std::endl;
    //     }
    //     bool buildVGFromOcGrid(OcGrid<false> *oc_grid, std::string output_path)
    //     {
    //         // save the oc_grid
    //         this->oc_grid = oc_grid;

    //         // Make LUT for relative cell keys
    //         struct vertexNbLUTEntry
    //         {
    //             std::array<int_t, 2> rel_cell_coord = {0, 0};
    //             mapkey_t rel_cell_key = 0;
    //             vertexNbLUTEntry(int_t rel_x, int_t rel_y, OcGrid<false> *oc_grid)
    //             {
    //                 vertexCoordToCellCoord<int_t>(0, 0, rel_x, rel_y, rel_cell_coord[0], rel_cell_coord[1]);
    //                 // rel_cell_coord = vertexCoordToCellCoord(std::array<int_t, 2>{0, 0}, std::array<int_t, 2>{rel_x, rel_y});
    //                 rel_cell_key = oc_grid->relKeyInDir(rel_cell_coord);
    //             }
    //         };
    //         std::array<vertexNbLUTEntry, 4> vertexNbLUT = {
    //             vertexNbLUTEntry(1, 1, oc_grid),
    //             vertexNbLUTEntry(-1, 1, oc_grid),
    //             vertexNbLUTEntry(-1, -1, oc_grid),
    //             vertexNbLUTEntry(1, -1, oc_grid)};

    //         int_t num_vertices_x = oc_grid->getSize()[0] - 1;            // does not include perimeter vertices
    //         int_t num_vertices_y = oc_grid->getSize()[1] - 1;            // does not include perimeter vertices
    //         unsigned int num_vertices = num_vertices_x * num_vertices_y; // does not include perimeter vertices
    //         std::vector<Combination> combs;
    //         unsigned int prog = 0;
    //         for (int_t x = 1; x <= num_vertices_x; ++x)
    //         {
    //             for (int_t y = 1; y <= num_vertices_y; ++y)
    //             {
    //                 mapkey_t vertex_key = oc_grid->coordToKey(x, y);
    //                 if constexpr (block)
    //                 {
    //                     unsigned int num_cells_accessible = 0;
    //                     for (auto &entry : vertexNbLUT)
    //                     {
    //                         mapkey_t cell_key = oc_grid->addKeyToRelKey(vertex_key, entry.rel_cell_key);
    //                         int_t cell_x = x + entry.rel_cell_coord[0];
    //                         int_t cell_y = y + entry.rel_cell_coord[1];
    //                         num_cells_accessible += oc_grid->isAccessible(cell_key, cell_x, cell_y);
    //                     }
    //                     if (num_cells_accessible == 3)
    //                     { // convex corner identified
    //                         combs.emplace_back(vertex_key);
    //                     }
    //                 }
    //                 else
    //                 {
    //                     std::array<vertexNbLUTEntry, 4> absolute = vertexNbLUT;
    //                     for (auto & entry : absolute)
    //                     {
    //                         entry.rel_cell_key = oc_grid->addKeyToRelKey(vertex_key, entry.rel_cell_key);
    //                         entry.rel_cell_coord[0] += x;
    //                         entry.rel_cell_coord[1] += y;
    //                     }
    //                     // check diagonally opposite pairs
    //                     for (unsigned int i = 0; i < 2; ++i)
    //                     {
    //                         auto &entry_a = absolute[i];
    //                         auto &entry_b = absolute[i + 2];
    //                         if (oc_grid->isAccessible(entry_a.rel_cell_key, entry_a.rel_cell_coord[0], entry_a.rel_cell_coord[1]) && oc_grid->isAccessible(entry_b.rel_cell_key, entry_b.rel_cell_coord[0], entry_b.rel_cell_coord[1]))
    //                         { // corner at this location if both diagonally opposite pairs are free and at least one cell is blocked
    //                             auto &entry_c = absolute[i + 1];
    //                             auto &entry_d = absolute[i + 3 > 3 ? i - 1 : i + 3];
    //                             if (!oc_grid->isAccessible(entry_c.rel_cell_key, entry_c.rel_cell_coord[0], entry_c.rel_cell_coord[1]) || !oc_grid->isAccessible(entry_d.rel_cell_key, entry_d.rel_cell_coord[0], entry_d.rel_cell_coord[1]))
    //                             {   // at least one adjacent cell is oc
    //                                 combs.emplace_back(vertex_key);
    //                                 break;
    //                             }
    //                         }
    //                     }
    //                 }
    //                 std::cout << "\rbuildVGFromOcGrid: Finding corners " << ++prog << "/" << num_vertices << std::flush;
    //             }
    //         }
    //         std::cout << std::endl;

    //         ///////////////////////////////// VIS PAIRS /////////////////////////////////////////
    //         std::cout << "buildVGFromOcGrid: Finding combinations 0/" << combs.size() << std::flush;

    //         std::vector<std::thread> threads;
    //         std::vector<int> th_states(num_threads, -1);
    //         std::vector<std::mutex> mtx_states(num_threads);
    //         std::vector<std::unique_lock<std::mutex>> locks;
    //         std::vector<unsigned int> th_ind1(num_threads);

    //         for (unsigned int i = 0; i < num_threads && i < combs.size(); ++i)
    //         {
    //             th_ind1[i] = i;
    //             th_states[i] = 1;
    //             threads.emplace_back(&VG2::threadCombLOS, this, &th_ind1[i], &th_states[i], &combs, &mtx_states[i]);
    //             locks.emplace_back(mtx_states[i], std::defer_lock);
    //         }

    //         unsigned int idx1 = num_threads;
    //         unsigned int num_shutdown = 0;
    //         prog = 0;
    //         while (num_shutdown < threads.size())
    //         {
    //             for (unsigned int i = 0; i < threads.size(); ++i)
    //             {
    //                 locks[i].lock();
    //                 if (th_states[i] == 0)
    //                 {
    //                     if (idx1 >= combs.size())
    //                     { // send shutdown
    //                         th_states[i] = -1;
    //                         ++num_shutdown;
    //                     }
    //                     else
    //                     { // continue searching index
    //                         th_states[i] = 1;
    //                         th_ind1[i] = idx1++; // assign new index
    //                     }
    //                     std::cout << "\rbuildVGFromOcGrid: Finding combinations " << ++prog << "/" << combs.size() << std::flush;
    //                 }
    //                 locks[i].unlock();
    //             }
    //         }
    //         std::cout << std::endl;
    //         for (auto &th : threads)
    //             th.join();

    //         // Write to binary file
    //         while (!writeCombsToFile(combs, output_path))
    //         {
    //             std::cout << "buildVGFromOcGrid: cannot write file! Key in a new output_path:" << std::endl;
    //             try
    //             {
    //                 std::cin >>
    //                     output_path;
    //             }
    //             catch (const std::exception &e)
    //             {
    //                 std::cout << "buildVGFromOcGrid: Exception while taking new output_path: " << e.what() << std::endl;
    //             }
    //         }

    //         // build VG
    //         buildVGfromCombs(combs);

    //         // Test
    //         // test();

    //         return true;
    //     }
    //     bool buildVGFromFile(OcGrid<false> *oc_grid, std::string input_path)
    //     {
    //         this->oc_grid = oc_grid;

    //         std::vector<Combination> combs;

    //         if (!readCombsFromFile(combs, input_path))
    //         {
    //             std::cout << "buildVGFromFile: readCombsFromFile reports that it cannot access file!" << std::endl;
    //             return false;
    //         }

    //         buildVGfromCombs(combs);

    //         // test();
    //         return true;
    //     }
    //     OcGrid<false> *getOcGrid()
    //     {
    //         return oc_grid;
    //     }
    //     std::vector<V2> run(const V2 begin, const V2 end, unsigned int num_threads = 8)
    //     {
    //         if (num_threads < 1)
    //             num_threads = 1;

    //         std::vector<V2> path = {};
    //         // -------------- CHECK IF BEGIN and END are EQUAL --------------------
    //         if (begin == end)
    //         {
    //             path.push_back(begin);
    //             return path;
    //         }
    //         // --------------- CHECK LOS betw BEGIN and END -----------------------
    //         LosType los(oc_grid);
    //         if (los.cast(begin, end) == Los2::reached)
    //         {
    //             // LOS will not be checked later bcos both are not in corners for threading multiple runs
    //             path.push_back(begin);
    //             path.push_back(end);
    //             return path;
    //         }
    //         mapkey_t end_key = oc_grid->coordToKey(end);
    //         mapkey_t begin_key = oc_grid->coordToKey(begin);

    //         // -------------------------- PROCESS BEGIN--------------------------------
    //         Corner *begin_crn;
    //         auto corners_p_ = corners.find(begin_key);
    //         bool begin_exists = corners_p_ != corners.end();
    //         if (begin_exists)
    //         {
    //             begin_crn = &(corners_p_->second);
    //         }
    //         else // begin not in corners
    //         {
    //             // if begin is not in corners, add begin to corners, and add visible neighbors to it.
    //             begin_crn = new Corner(begin_key, begin);

    //             // use threads to find neighbors
    //             std::vector<std::thread> threads;
    //             std::vector<int> th_states(num_threads, -1);
    //             std::vector<Corner *> th_crns(num_threads, nullptr);
    //             std::vector<char> th_reached(num_threads, 0);
    //             std::vector<std::mutex> mtx_states(num_threads);
    //             std::vector<std::unique_lock<std::mutex>> locks;

    //             corners_p_ = corners.begin();
    //             for (unsigned int i = 0; i < num_threads && corners_p_ != corners.end(); ++i)
    //             {
    //                 th_states[i] = 1;
    //                 th_crns[i] = &(corners_p_->second);
    //                 locks.emplace_back(mtx_states[i], std::defer_lock);
    //                 threads.emplace_back(&VG2::threadLOS, this,
    //                                      &th_crns[i], &begin_crn, &th_reached[i], &th_states[i], &mtx_states[i]);
    //                 ++corners_p_;
    //             }

    //             unsigned int num_shutdown = 0;
    //             while (num_shutdown < threads.size())
    //             {
    //                 for (unsigned int i = 0; i < threads.size(); ++i)
    //                 {
    //                     locks[i].lock();
    //                     // add los
    //                     if (th_states[i] == 0)
    //                     {
    //                         if (th_reached[i])
    //                         {
    //                             begin_crn->neighbors.push_back(th_crns[i]);
    //                         }
    //                         if (corners_p_ == corners.end())
    //                         { // send shutdown
    //                             th_states[i] = -1;
    //                             ++num_shutdown;
    //                         }
    //                         else
    //                         { // continue searching los
    //                             th_states[i] = 1;
    //                             th_crns[i] = &(corners_p_->second);
    //                             // reached[i] = false;
    //                             ++corners_p_;
    //                         }
    //                     }
    //                     locks[i].unlock();
    //                 }
    //             }
    //             for (auto &th : threads)
    //                 th.join();
    //         }
    //         // -------------------------- PROCESS END--------------------------------
    //         Corner *end_crn;
    //         corners_p_ = corners.find(end_key);
    //         bool end_exists = corners_p_ != corners.end();
    //         std::unordered_set<Corner *> end_nbs;
    //         if (end_exists)
    //         {
    //             end_crn = &(corners_p_->second);
    //             end_nbs.rehash(0);
    //         }
    //         else
    //         {
    //             // if end is not in corners, add end to corners, and add visible neighbors to end, and end to visible neighbors
    //             end_crn = new Corner(end_key, end);
    //             end_crn->neighbors.reserve(0); // insignificant

    //             // use threads to find neighbors
    //             std::vector<std::thread> threads;
    //             std::vector<int> th_states(num_threads, -1);
    //             std::vector<Corner *> th_crns(num_threads, nullptr);
    //             std::vector<char> th_reached(num_threads, 0); // vector bool does not work
    //             std::vector<std::mutex> mtx_states(num_threads);
    //             std::vector<std::unique_lock<std::mutex>> locks;

    //             corners_p_ = corners.begin();
    //             for (unsigned int i = 0; i < num_threads && corners_p_ != corners.end(); ++i)
    //             {
    //                 th_states[i] = 1;
    //                 th_crns[i] = &(corners_p_->second);
    //                 locks.emplace_back(mtx_states[i], std::defer_lock);
    //                 threads.emplace_back(&VG2::threadLOS, this,
    //                                      &th_crns[i], &end_crn, &th_reached[i], &th_states[i], &mtx_states[i]);
    //                 ++corners_p_;
    //             }

    //             unsigned int num_shutdown = 0;
    //             while (num_shutdown < threads.size())
    //             {
    //                 for (unsigned int i = 0; i < threads.size(); ++i)
    //                 {
    //                     locks[i].lock();
    //                     // add los
    //                     if (th_states[i] == 0)
    //                     {
    //                         if (th_reached[i])
    //                         {
    //                             end_nbs.insert(th_crns[i]);
    //                         }
    //                         if (corners_p_ == corners.end())
    //                         { // send shutdown
    //                             th_states[i] = -1;
    //                             ++num_shutdown;
    //                         }
    //                         else
    //                         { // continue searching los
    //                             th_states[i] = 1;
    //                             th_crns[i] = &(corners_p_->second);
    //                             // reached[i] = false;
    //                             ++corners_p_;
    //                         }
    //                     }
    //                     locks[i].unlock();
    //                 }
    //             }
    //             for (auto &th : threads)
    //                 th.join();
    //         }

    //         // ------------------------------- ASTAR -----------------------------------------------

    //         OpenList open_list;

    //         // create start node
    //         std::unordered_map<mapkey_t, Node> nodes;
    //         auto emp_res = nodes.try_emplace(begin_key, begin_crn, norm(end, begin), open_list.end()); // add starting node
    //         Node *node = &(emp_res.first->second);
    //         node->g = 0; // node->f = node->h when initialised

    //         // add starting node to open list
    //         open_list.push(node);

    //         // main loop
    //         while (!open_list.empty())
    //         {
    //             // poll node
    //             Node *node = open_list.poll();
    //             Corner *crn = node->crn;

    //             // skip if visited
    //             if (node->visited)
    //                 continue;
    //             node->visited = true;

    //             // check if goal found
    //             if (crn == end_crn)
    //             {
    //                 // goal found, construct path
    //                 do
    //                 {
    //                     path.push_back(node->crn->coord);
    //                     node = node->parent;
    //                 } while (node != nullptr);
    //                 break;
    //             }

    //             // check if there is direct LOS to end (if end is not in corners)
    //             if (!end_exists && end_nbs.find(crn) != end_nbs.end())
    //             {
    //                 // direct LOS represents the int_test path betw node and end, so can skip checking neighbors
    //                 processNb(end_crn, node, end, nodes, open_list);
    //                 continue; // skip neighbors
    //             }

    //             // for each neighbor
    //             for (auto &nb_crn : crn->neighbors)
    //                 processNb(nb_crn, node, end, nodes, open_list);
    //         }

    //         // destruction
    //         if (!begin_exists)
    //             delete begin_crn;
    //         if (!end_exists)
    //             delete end_crn;
    //         path.shrink_to_fit();
    //         return path;
    //     }
    // };

}