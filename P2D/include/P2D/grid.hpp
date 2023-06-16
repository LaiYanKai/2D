#include <fstream>
#include <assert.h>
#include "types.hpp"
#include "math.hpp"
#include "Vec2.hpp"
#include "dir_idx.hpp"

#pragma once
namespace P2D
{
    class Grid
    {
    public:
        struct Relative
        {
            mapkey_t key;
            V2 coord;
            Relative(const mapkey_t &key, const V2 &coord) : key(key), coord(coord) {}
            Relative() : key(0), coord() {}
        };
        using AdjLUT = Relative[8];

    private:
        V2 size_cell = V2(0, 0);
        V2 size_vert = V2(0, 0);
        AdjLUT ADJS_CELL, ADJS_VERT;
        bool *_data = nullptr;

        void initLUT()
        {
            for (unsigned int dir_idx = 0; dir_idx < 8; ++dir_idx)
            {
                // ADJ: adj vertex of vertex; adj cell of cell
                Relative &ADJ_CELL = ADJS_CELL[dir_idx];
                ADJ_CELL.coord = dirIdxToDir(dir_idx);
                ADJ_CELL.key = getDiffKey<true>(ADJ_CELL.coord);

                Relative &ADJ_VERT = ADJS_VERT[dir_idx];
                ADJ_VERT.coord = dirIdxToDir(dir_idx);
                ADJ_VERT.key = getDiffKey<false>(ADJ_VERT.coord);
            }
        }

    public:
        // _data in row major form, 1 indicating occupied, 0 indicating free.
        // size_cell.x is num x (height), size_cell.y is num y (width)
        Grid(const bool *const &_data, const V2 &size_cell) : _data(nullptr) { init(_data, size_cell); }
        Grid() {}
        ~Grid()
        {
            if (_data != nullptr)
                delete[] _data;
        }

        template <bool is_cell>
        inline const V2 &getSize() const { return is_cell ? size_cell : size_vert; }

        void init(const bool *const &_data, const V2 &size_cell)
        {
            this->size_cell = size_cell;
            this->size_vert = size_cell + 1;
            assert(this->_data == nullptr);
            assert(size_cell.x > 0); // x is larger than zero
            assert(size_cell.y > 0); // y is larger than zero
            const int_t num = size_cell.x * size_cell.y;
            this->_data = {new bool[size_cell.x * size_cell.y]};
            for (int_t i = 0; i < num; ++i) // copy _data
                this->_data[i] = _data[i];

            initLUT();
        }
        void clear()
        {
            delete[] _data;
            _data = nullptr;
            size_cell = 0;
        }

        template <bool is_cell>
        inline mapkey_t coordToKey(const int_t &x, const int_t &y) const
        {
            if constexpr (is_cell)
                return ((mapkey_t)x) * size_cell.y + ((mapkey_t)y);
            else
                return ((mapkey_t)x) * size_vert.y + ((mapkey_t)y);
        }

        template <bool is_cell>
        inline mapkey_t coordToKey(const V2 &coord) const { return coordToKey<is_cell>(coord.x, coord.y); }

        template <bool is_cell>
        inline void keyToCoord(const mapkey_t &key, int_t &x, int_t &y) const
        {
            const V2 &size = (is_cell) ? size_cell : size_vert;
            x = key / size.y;
            y = key - x * size.y;
        }

        template <bool is_cell>
        inline V2 keyToCoord(const mapkey_t &key) const
        {
            V2 coord;
            keyToCoord<is_cell>(key, coord.x, coord.y);
            return coord;
        }

        template <bool is_cell>
        inline void keyToCoord(const mapkey_t &key, V2 &coord) const { keyToCoord<is_cell>(key, coord.x, coord.y); }

        // returns the difference in key of a point b and point a, where (diff_x, diff_y) = (b - a)
        // add the returned value to the key of point a using addKeyToRelKey to get the key of point b
        template <bool is_cell>
        inline mapkey_t getDiffKey(const int_t &diff_x, const int_t &diff_y) const
        {
            const V2 &size = (is_cell) ? size_cell : size_vert;
            return diff_x * size.y + diff_y;
        }
        // returns the difference in key of a point b and point a, where diff = (b - a)
        // add the returned value to the key of point a using addKeyToRelKey to get the key of point b
        template <bool is_cell>
        inline mapkey_t getDiffKey(const V2 &diff) const { return getDiffKey<is_cell>(diff.x, diff.y); }

        // returns key in dir_x and dir_y, direction.
        template <bool is_cell>
        inline const mapkey_t &getRelKey(const int_t &dir_x, const int_t &dir_y) const
        {
            const dir_idx_t di = dirToDirIdx(dir_x, dir_y);
            return getRelKey<is_cell>(di);
        }
        // returns key in rel_coord direction.
        template <bool is_cell>
        inline const mapkey_t &getRelKey(const V2 &rel_coord) const { return getRelKey<is_cell>(rel_coord.x, rel_coord.y); }

        // returns relative key of an adjacent vertex/coord in direction dir_idx
        template <bool is_cell>
        inline const mapkey_t &getRelKey(const dir_idx_t &dir_idx) const
        {
            assert(inRange(dir_idx) == true);
            const AdjLUT &ADJS = (is_cell) ? ADJS_CELL : ADJS_VERT;
            return ADJS[dir_idx].key;
        }

        // return the relative coordinates of an adjacent vertex/coord in direction dir_idx
        template <bool is_cell>
        inline const V2 &getRelCoord(const dir_idx_t &dir_idx) const
        {
            assert(inRange(dir_idx) == true);
            const AdjLUT &ADJS = (is_cell) ? ADJS_CELL : ADJS_VERT;
            return ADJS[dir_idx].coord;
        }

        // returns key of a vertex/coordinate from a current vertex/coord (key) from the relative key (rel_key)
        static inline mapkey_t addKeyToRelKey(const mapkey_t &key, const mapkey_t &rel_key) { return key + rel_key; }

        // returns the key of a vertex/coordinate from a current vertex/coord (key) in the direction (dir_x, dir_y).
        template <bool is_cell>
        inline mapkey_t getKey(const mapkey_t &key, const int_t &dir_x, const int_t &dir_y) const { return addKeyToRelKey(key, getRelKey<is_cell>(dir_x, dir_y)); }

        // returns relative cell key in dir_idx (1,3,5,7) depending on the vertex x position
        inline mapkey_t getCellRelKey(const dir_idx_t &dir_idx, const int_t &vert_x) const
        {
            assert(isOrdinal(dir_idx) == true);
            assert(inRange(dir_idx) == true);

            mapkey_t rel_key_cell;
            if (dir_idx == 1)
                rel_key_cell = (0 - vert_x);
            else if (dir_idx == 3)
                rel_key_cell = (-size_cell.y - vert_x);
            else if (dir_idx == 5)
                rel_key_cell = (-1 - size_cell.y - vert_x);
            else
            {
                assert(dir_idx == 7);
                rel_key_cell = (-1 - vert_x);
            }

            return rel_key_cell;
        }

        // returns relative cell coord (rel_x, rel_y) in dir_idx (1,3,5,7) from vertex
        inline void getCellRelCoord(const dir_idx_t &dir_idx, int_t &rel_x, int_t &rel_y) const
        { // inline constexpr doesnt make a difference. //inline to put the function in header file explicitly
            assert(isOrdinal(dir_idx) == true);
            assert(inRange(dir_idx) == true);

            if (dir_idx == 1)
            {
                rel_x = 0;
                rel_y = 0;
            }
            else if (dir_idx == 3)
            {
                rel_x = -1;
                rel_y = 0;
            }
            else if (dir_idx == 5)
            {
                rel_x = -1;
                rel_y = -1;
            }
            else if (dir_idx == 7)
            {
                rel_x = 0;
                rel_y = -1;
            }
            else
                assert(false);
        }
        // returns relative cell coord (rel_x, rel_y) in dir_idx (1,3,5,7) from vertex
        inline V2 getCellRelCoord(const dir_idx_t &dir_idx) const
        {
            V2 cell;
            getCellRelCoord(dir_idx, cell.x, cell.y);
            return cell;
        }

        // returns the cell key and coordinate in dir_idx (1,3,5,7) from a vertex
        inline void getCellKeyAndCoord(const dir_idx_t &dir_idx, const mapkey_t &vert_key, const V2 &vert_coord, mapkey_t &cell_key, V2 &cell_coord) const
        {
            cell_key = addKeyToRelKey(vert_key, getCellRelKey(dir_idx, vert_coord.x));
            cell_coord = vert_coord + getCellRelCoord(dir_idx);
        }

        // returns relative cell key in dir_idx (1,3,5,7) depending on the vertex x position
        inline mapkey_t getVertexRelKey(const dir_idx_t &dir_idx, const int_t &cell_x) const
        {
            assert(isOrdinal(dir_idx) == true);
            assert(inRange(dir_idx) == true);

            mapkey_t rel_key_vert;
            if (dir_idx == 1)
                rel_key_vert = (size_vert.y + 1 + cell_x);
            else if (dir_idx == 3)
                rel_key_vert = (1 + cell_x);
            else if (dir_idx == 5)
                rel_key_vert = (cell_x);
            else
            {
                assert(dir_idx == 7);
                rel_key_vert = (size_vert.y + cell_x);
            }

            return rel_key_vert;
        }

        // returns relative vertex coord (rel_x, rel_Y) in dir_idx (1,3,5,7) from cell
        inline void getVertexRelCoord(const dir_idx_t &dir_idx, int_t &rel_x, int &rel_y) const
        {
            assert(isOrdinal(dir_idx) == true);
            assert(inRange(dir_idx) == true);
            if (dir_idx == 1)
            {
                rel_x = 1;
                rel_y = 1;
            }
            else if (dir_idx == 3)
            {
                rel_x = 0;
                rel_y = 1;
            }
            else if (dir_idx == 5)
            {
                rel_x = 0;
                rel_y = 0;
            }
            else if (dir_idx == 7)
            {
                rel_x = 1;
                rel_y = 0;
            }
            else
                assert(false);
        }
        // returns relative vertex coord (rel_x, rel_Y) in dir_idx (1,3,5,7) from cell
        inline V2 getVertexRelCoord(const dir_idx_t &dir_idx) const
        {
            V2 vert;
            getVertexRelCoord(dir_idx, vert.x, vert.y);
            return vert;
        }

        // returns the vertex key and coordinate in dir_idx (1,3,5,7) from a cell
        inline void getVertexKeyAndCoord(const dir_idx_t &dir_idx, const mapkey_t &cell_key, const V2 &cell_coord, mapkey_t &vert_key, V2 &vert_coord) const
        {
            vert_key = addKeyToRelKey(cell_key, getVertexRelKey(dir_idx, cell_coord.x));
            vert_coord = cell_coord + getVertexRelCoord(dir_idx);
        }

        template <bool is_cell>
        inline int_t getBoundary(const dir_idx_t &dir_idx) const
        {
            assert(inRange(dir_idx) == true);
            const V2 &size = is_cell ? size_cell : size_vert;
            if (dir_idx == 4 || dir_idx == 6)
                return 0;
            else if (dir_idx == 0)
                return size.x - 1;
            else
                return size.y - 1;
        }

        // checks if test coordinate (x or y) of vertex or cell is in the map by comparing the coordinate and the boundary value dir_idx
        // dir_idx has to be cardinal
        template <bool is_cell>
        inline bool inMap(const int_t &test, const dir_idx_t &dir_idx) const
        {
            assert(isCardinal(dir_idx) == true);
            assert(inRange(dir_idx) == true);
            switch (dir_idx)
            {
            case 0:
            case 2:
                return test <= getBoundary<is_cell>(dir_idx);
            case 4:
            case 6:
                return test >= 0; // >= getBoundary(dir_idx);
            }
        }

        // checks if coordinate of vertex or cell is in the map
        template <bool is_cell>
        inline bool inMap(const int_t &x, const int_t &y) const
        {
            // returns true if (x,y) is in map
            if constexpr (is_cell)
                return x >= 0 && y >= 0 && x < size_cell.x && y < size_cell.y;
            else
                return x >= 0 && y >= 0 && x < size_vert.x && y < size_vert.y;
        }

        // checks if coordinate of vertex or cell is in the map
        template <bool is_cell>
        inline bool inMap(const V2 &coord) const { return inMap<is_cell>(coord.x, coord.y); }

        // returns occupancy without checking map boundaries.
        inline bool isOc(const mapkey_t &cell_key) const { return _data[cell_key]; }

        // returns true if cell is free and within map boundary
        inline bool isAccessible(const mapkey_t &cell_key, const int_t &cell_x, const int_t &cell_y) const { return inMap<true>(cell_x, cell_y) ? !isOc(cell_key) : false; }

        // returns true if cell is free and within map boundary.
        inline bool isAccessible(const mapkey_t &cell_key, const V2 &cell_coord) const { return isAccessible(cell_key, cell_coord.x, cell_coord.y); }

        // returns true if cell is free and within map boundary.
        inline bool isAccessible(const int_t &cell_x, const int_t &cell_y) const
        {
            mapkey_t cell_key = coordToKey<true>(cell_x, cell_y);
            return isAccessible(cell_key, cell_x, cell_y);
        }

        // returns true if cell is free and within map BOUNDS_CELL.
        inline bool isAccessible(const V2 &cell_coord) const { return isAccessible(cell_coord.x, cell_coord.y); }
    };
}