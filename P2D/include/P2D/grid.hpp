#include <fstream>
#include "types.hpp"
#include "math.hpp"
#include "Vec2.hpp"

#pragma once
namespace P2D
{
    using mapkey_t = uint32_t;

    class Grid
    {
    public:
        struct Adjacent
        {
            mapkey_t rel_key;
            V2 rel_coord;
            Adjacent(const mapkey_t &rel_key, const V2 &rel_coord) : rel_key(rel_key), rel_coord(rel_coord) {}
            Adjacent() : rel_key(0), rel_coord() {}
        };
        using AdjLUT = Adjacent[8];
        using BoundsLUT = int_t[8];

    private:
        V2 size_cell = V2(0, 0);
        V2 size_vert = V2(0, 0);
        AdjLUT ADJS_CELL, ADJS_VERT;
        BoundsLUT BOUNDS_CELL = {0}, BOUNDS_VERT = {0};
        bool *data = nullptr;

        void initLUT()
        {
            for (unsigned int dir_idx = 0; dir_idx < 8; ++dir_idx)
            {
                // ADJ: adj vertex of vertex; adj cell of cell
                Adjacent &ADJ_CELL = ADJS_CELL[dir_idx];
                ADJ_CELL.rel_coord = dirIdxToDir<V2>(dir_idx);
                ADJ_CELL.rel_key = getRelKey<true>(ADJ_CELL.rel_coord);

                Adjacent &ADJ_VERT = ADJS_VERT[dir_idx];
                ADJ_VERT.rel_coord = dirIdxToDir<V2>(dir_idx);
                ADJ_VERT.rel_key = getRelKey<false>(ADJ_VERT.rel_coord);

                // // ADJ_CELL_FROM_VERTEX: adj cell of vertex. Not defined for cardinal directions (dir_idx is even)
                // if ((dir_idx & 1) == 1)
                // {
                //     V2 &rel_cell_coord = ADJ_CELL_FROM_VERTEX[dir_idx].rel_coord;
                //     rel_cell_coord = vertexCoordToCellCoord(V2(0, 0), rel_coord);
                //     ADJ_CELL_FROM_VERTEX[dir_idx].rel_key = relKeyInDir(rel_cell_coord);
                // }
            }
            BOUNDS_CELL[0] = size_cell[0];
            BOUNDS_CELL[2] = size_cell[1];
            BOUNDS_VERT[0] = size_vert[0];
            BOUNDS_VERT[2] = size_vert[1];
        }

    public:
        // data in row major form, 1 indicating occupied, 0 indicating free.
        // size_cell[0] is num x (height), size_cell[1] is num y (width)
        Grid(const bool *const &data, const V2 &size_cell) : data(nullptr) { init(data, size_cell); }
        Grid() {}
        ~Grid()
        {
            if (data != nullptr)
                delete[] data;
        }

        inline const V2 &getSize() const { return size_cell; }
        void init(const bool *const &data, const V2 &size_cell)
        {
            this->size_cell = size_cell;
            this->size_vert = size_cell + 1;
            assert(this->data == nullptr);
            assert(size_cell[0] > 0); // x is larger than zero
            assert(size_cell[1] > 0); // y is larger than zero
            const int_t num = size_cell[0] * size_cell[1];
            this->data = {new bool[size_cell[0] * size_cell[1]]};
            for (int_t i = 0; i < num; ++i) // copy data
                this->data[i] = data[i];

            initLUT();
        }
        void clear()
        {
            delete[] data;
            data = nullptr;
            size_cell = 0;
        }

        template <bool is_cell>
        inline mapkey_t coordToKey(const int_t &x, const int_t &y) const
        {
            if constexpr (is_cell)
                return ((mapkey_t)x) * size_cell[1] + ((mapkey_t)y);
            else
                return ((mapkey_t)x) * size_vert[1] + ((mapkey_t)y);
        }

        template <bool is_cell>
        inline mapkey_t coordToKey(const V2 &coord) const { return coordToKey<is_cell>(coord.x, coord.y); }

        template <bool is_cell>
        inline void keyToCoord(const mapkey_t &key, int_t &x, int_t &y) const
        {
            if constexpr (is_cell)
            {
                x = key / size_cell[1];
                y = key - x * size_cell[1];
            }
            else
            {
                x = key / size_vert[1];
                y = key - x * size_vert[1];
            }
        }

        template <bool is_cell>
        inline V2 keyToCoord(const mapkey_t &key) const
        {
            V2 coord;
            keyToCoord<is_cell>(key, coord[0], coord[1]);
            return coord;
        }

        template <bool is_cell>
        inline void keyToCoord(const mapkey_t &key, V2 &coord) const { keyToCoord<is_cell>(key, coord[0], coord[1]); }

        template <bool is_cell>
        inline mapkey_t getRelKey(const int_t &dir_x, const int_t &dir_y) const
        {
            if constexpr (is_cell)
                return mapkey_t(dir_x) * mapkey_t(size_cell[1]) + mapkey_t(dir_y);
            else
                return mapkey_t(dir_x) * mapkey_t(size_vert[1]) + mapkey_t(dir_y);
        }

        // keys must be all vertex or all cells
        static inline mapkey_t addKeyToRelKey(const mapkey_t &key, const mapkey_t &rel_key) { return key + rel_key; }

        template <bool is_cell>
        inline mapkey_t getKey(const mapkey_t &key, const int_t &dir_x, const int_t &dir_y) const { return addKeyToRelKey(key, relKeyInDir<is_cell>(dir_x, dir_y)); }

        template <bool is_cell>
        inline mapkey_t getRelKey(const V2 &rel_coord) const { return getRelKey<is_cell>(rel_coord[0], rel_coord[1]); }

        // return the relative coordinates for an adjacent vertex from a vertex, or adjacent cell from a cell, in direction dir_idx
        template <bool is_cell>
        inline const V2 &getRelCoord(const dir_idx_t &dir_idx) const
        {
            if constexpr (is_cell)
                return ADJS_CELL[dir_idx].rel_coord;
            else
                return ADJS_VERT[dir_idx].rel_coord;
        }

        // return the relative mapkey for an adjacent vertex from a vertex, or adjacent cell from a cell, in direction dir_idx
        template <bool is_cell>
        inline const mapkey_t &getRelKey(const dir_idx_t &dir_idx) const
        {
            if constexpr (is_cell)
                return ADJS_CELL[dir_idx].rel_key;
            else
                return ADJS_VERT[dir_idx].rel_key;
        }

        // returns cell key in dir_idx (1,3,5,7) depending on the vertex y position
        inline mapkey_t getRelKeyCFV(const dir_idx_t &dir_idx, const int_t &vert_y)
        {
            assert((dir_idx & 1) == 1);         // must be odd (ordinal)
            assert(dir_idx > 0 && dir_idx < 8); // must be in range

            mapkey_t rel_key_cell;
            if (dir_idx == 1)
                rel_key_cell = (0 - vert_y);
            else if (dir_idx == 3)
                rel_key_cell = (-1 - vert_y);
            else if (dir_idx == 5)
                rel_key_cell = (-1 - vert_y + 1);
            else if (dir_idx == 7)
                rel_key_cell = (0 - vert_y + 1);
            else
                assert(false);

            return rel_key_cell;
        }

        inline constexpr mapkey_t getRelCoordCFV(const dir_idx_t &dir_idx, const int_t &vert_x, const int_t & vert_y) 
        {   // inline constexpr doesnt make a difference. //inline to put the function in header file explicitly


        }

        
        template <bool is_cell>
        inline const int_t &getBoundary(const dir_idx_t &dir_idx) const
        {
            if constexpr (is_cell)
                return BOUNDS_CELL[dir_idx];
            else
                return BOUNDS_VERT[dir_idx];
        }

        // checks if test coordinate (x or y) of vertex or cell is in the map by comparing the coordinate and the boundary value in_dir_idx
        template <bool is_cell>
        inline bool inMap(const int_t &test, const dir_idx_t &in_dir_idx) const
        {
            switch (in_dir_idx)
            {
            case 0:
            case 2:
                if constexpr (is_cell)
                    return test < getBoundaryVertexValue(in_dir_idx);
                else
                    return test <= getBoundaryVertexValue(in_dir_idx);
            case 4:
            case 6:
                return test >= 0; // >= getBoundaryVertexValue(in_dir_idx);
            default:
                throw std::out_of_range("inMap received invalid in_dir_idx (" + std::to_string(int(in_dir_idx)) + ")");
            }
        }

        // checks if coordinate of vertex or cell is in the map
        template <bool is_cell>
        inline bool inMap(const int_t &x, const int_t &y) const
        {
            // returns true if (x,y) is in map
            if constexpr (is_cell)
                return x >= 0 && y >= 0 && x < size_cell[0] && y < size_cell[1];
            else
                return x >= 0 && y >= 0 && x <= size_cell[0] && y <= size_cell[1];
        }

        // checks if coordinate of vertex or cell is in the map
        template <bool is_cell>
        inline bool inMap(const V2 &coord) const { return inMap<is_cell>(coord[0], coord[1]); }

        // returns occupancy without checking BOUNDS_CELL.
        inline bool isOc(const mapkey_t &key) const { return data[key]; }

        // returns true if cell is free and within map BOUNDS_CELL
        inline bool isAccessible(const mapkey_t &key, const int_t &x, const int_t &y) const { return inMap<true>(x, y) ? !isOc(key) : false; }

        // returns true if cell is free and within map BOUNDS_CELL.
        inline bool isAccessible(const mapkey_t &key, const V2 &coord) const { return isAccessible(key, coord[0], coord[1]); }

        // returns true if cell is free and within map BOUNDS_CELL.
        inline bool isAccessible(const mapkey_t &key) const
        {
            int_t x, y;
            keyToCoord(key, x, y);
            return isAccessible(key, x, y);
        }

        // returns true if cell is free and within map BOUNDS_CELL.
        inline bool isAccessible(const int_t &x, const int_t &y) const
        {
            // returns true if pixel (x,y) is free and in map.
            mapkey_t key = coordToKey(x, y);
            return isAccessible(key, x, y);
        }

        // returns true if cell is free and within map BOUNDS_CELL.
        inline bool isAccessible(const V2 &coord) const { return isAccessible(coord[0], coord[1]); }
    };

    // template <class T, class U>
    // constexpr T vertexCoordToCellCoord(const U &vertex_coord, const T &coord_is_whole, const T &dir)
    // {
    //     // vertex_coord is already floored  when converting from V2f to V2.
    //     // vertex_coord must already be rounded to whole if it is whole.
    //     T cell_coord = (T)vertex_coord;

    //     // vertex_is_whole[dim] must be true if vertex_coord[dim] is whole, false otherwise.
    //     for (unsigned int dim = 0; dim < 2; ++dim)
    //     {
    //         if (coord_is_whole[dim])
    //         { // convert  positive direction to zero, negative to -1 (since the cells coordinates are at the floor).
    //             cell_coord[dim] += (dir[dim] >= 0 ? 0 : -1);
    //         }
    //         else
    //         { // keep only the direction where the vertex is whole.
    //         }
    //     }
    //     return cell_coord;
    // }
    // template <typename T>
    // constexpr void vertexCoordToCellCoord(const T &whole_vertex_coord_x, const T &whole_vertex_coord_y, const T &dir_x, const T &dir_y, T &cell_coord_x, T &cell_coord_y)
    // {
    //     cell_coord_x = whole_vertex_coord_x + (dir_x >= 0 ? 0 : -1);
    //     cell_coord_y = whole_vertex_coord_y + (dir_y >= 0 ? 0 : -1);
    // }
    // template <class T>
    // constexpr T vertexCoordToCellCoord(const T &whole_vertex_coord, const T &dir)
    // {
    //     T cell_coord;
    //     vertexCoordToCellCoord(whole_vertex_coord[0], whole_vertex_coord[1],
    //                            dir[0], dir[1], cell_coord[0], cell_coord[1]);
    //     return cell_coord;
    // }

}