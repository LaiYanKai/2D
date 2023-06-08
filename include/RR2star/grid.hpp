#include <fstream>
#include <unordered_set>

#include "math.hpp"
#include "Vec2.hpp"

#pragma once
namespace RR2star
{
    using mapkey_t = uint32_t;

    template <class T, class U>
    constexpr T vertexCoordToCellCoord(const U &vertex_coord, const T &coord_is_whole, const T &dir)
    {
        // vertex_coord is already floored  when converting from V2f to V2.
        // vertex_coord must already be rounded to whole if it is whole.
        T cell_coord = (T)vertex_coord;

        // vertex_is_whole[dim] must be true if vertex_coord[dim] is whole, false otherwise.
        for (unsigned int dim = 0; dim < 2; ++dim)
        {
            if (coord_is_whole[dim])
            { // convert  positive direction to zero, negative to -1 (since the cells coordinates are at the floor).
                cell_coord[dim] += (dir[dim] >= 0 ? 0 : -1);
            }
            else
            { // keep only the direction where the vertex is whole.
            }
        }
        return cell_coord;
    }
    template <typename T>
    constexpr void vertexCoordToCellCoord(const T &whole_vertex_coord_x, const T &whole_vertex_coord_y, const T &dir_x, const T &dir_y, T &cell_coord_x, T &cell_coord_y)
    {
        cell_coord_x = whole_vertex_coord_x + (dir_x >= 0 ? 0 : -1);
        cell_coord_y = whole_vertex_coord_y + (dir_y >= 0 ? 0 : -1);
    }
    template <class T>
    constexpr T vertexCoordToCellCoord(const T &whole_vertex_coord, const T &dir)
    {
        T cell_coord;
        vertexCoordToCellCoord(whole_vertex_coord[0], whole_vertex_coord[1],
                               dir[0], dir[1], cell_coord[0], cell_coord[1]);
        return cell_coord;
    }

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

    private:
        V2 size = V2(0, 0);
        Adjacent ADJ[8], ADJ_CELL_FROM_VERTEX[8];
        int_t BOUNDS[8] = {0};
        bool *data = nullptr;

        void initLUT()
        {
            for (unsigned int dir_idx = 0; dir_idx < 8; ++dir_idx)
            {
                // ADJ: adj vertex of vertex; adj cell of cell
                V2 &rel_coord = ADJ[dir_idx].rel_coord;
                rel_coord = dirIdxToDir<V2>(dir_idx);
                ADJ[dir_idx].rel_key = relKeyInDir(rel_coord);

                // ADJ_CELL_FROM_VERTEX: adj cell of vertex. Not defined for cardinal directions (dir_idx is even)
                if ((dir_idx & 1) == 1)
                {
                    V2 &rel_cell_coord = ADJ_CELL_FROM_VERTEX[dir_idx].rel_coord;
                    rel_cell_coord = vertexCoordToCellCoord(V2(0, 0), rel_coord);
                    ADJ_CELL_FROM_VERTEX[dir_idx].rel_key = relKeyInDir(rel_cell_coord);
                }
            }
            BOUNDS[0] = size[0];
            BOUNDS[2] = size[1];
        }

    public:
        using AdjLUTType = Adjacent[8];
        using BoundsLUTType = int_t[8];

        // data in row major form, 1 indicating occupied, 0 indicating free.
        // size[0] is num x (height), size[1] is num y (width)
        Grid(const bool *const &data, const V2 &size) : data(nullptr) { init(data, size); }
        Grid() : size(0, 0), data(nullptr) {}
        ~Grid()
        {
            if (data != nullptr)
                delete[] data;
        }

        inline const V2 &getSize() const { return size; }
        void init(const bool *const &data, const V2 &size)
        {
            assert(data == nullptr);
            assert(size[0] > 0); // x is larger than zero
            assert(size[1] > 0); // y is larger than zero
            const int_t num = size[0] * size[1];
            this->data = {new bool[size[0] * size[1]]};
            for (int_t i = 0; i < num; ++i) // copy data
                this->data[i] = data[i];
            this->size = size;
        }
        void clear()
        {
            delete[] data;
            data = nullptr;
            size = 0;
        }


        inline mapkey_t coordToKey(const int_t &x, const int_t &y) const { return ((mapkey_t)x) * size[1] + ((mapkey_t)y); }
        inline mapkey_t coordToKey(const V2 &coord) const { return coordToKey(coord.x, coord.y); }
        inline void keyToCoord(const mapkey_t &key, int_t &x, int_t &y) const
        {
            x = key / size[1];
            y = key - x * size[1];
        }
        inline V2 keyToCoord(const mapkey_t &key) const
        {
            V2 coord;
            keyToCoord(key, coord[0], coord[1]);
            return coord;
        }
        inline void keyToCoord(const mapkey_t &key, V2 &coord) const
        {
            keyToCoord(key, coord[0], coord[1]);
        }
        inline mapkey_t relKeyInDir(const int_t &dir_x, const int_t &dir_y) const { return mapkey_t(dir_x) * mapkey_t(size[1]) + mapkey_t(dir_y); }
        static inline mapkey_t addKeyToRelKey(const mapkey_t &key, const mapkey_t &rel_key) { return key + rel_key; }
        inline mapkey_t keyInDir(const mapkey_t &key, const int_t &dir_x, const int_t &dir_y) const { return addKeyToRelKey(key, relKeyInDir(dir_x, dir_y)); }
        inline mapkey_t relKeyInDir(const V2 &rel_coord) const { return relKeyInDir(rel_coord[0], rel_coord[1]); }

        // return the relative coordinates for an adjacent vertex from a vertex, or adjacent cell from a cell, in direction dir_idx
        inline const V2 &getAdjRelCoord(dir_idx_t dir_idx) const { return ADJ[dir_idx].rel_coord; }

        // return the relative mapkey for an adjacent vertex from a vertex, or adjacent cell from a cell, in direction dir_idx
        inline const mapkey_t &getAdjRelKey(dir_idx_t dir_idx) const { return ADJ[dir_idx].rel_key; }

        // return the relative coordinates for an adjacent cell from a vertex, in direction dir_idx
        // NOT DEFINED FOR CARDINAL dir_idx (even: 0, 2, 4, 6)
        inline const V2 &getAdjRelCoordCFV(dir_idx_t dir_idx) const { return ADJ_CELL_FROM_VERTEX[dir_idx].rel_coord; }

        // return the relative mapkey for an adjacent cell from a vertex, in direction dir_idx
        // NOT DEFINED FOR CARDINAL dir_idx (even: 0, 2, 4, 6)
        inline const mapkey_t &getAdjRelKeyCFV(dir_idx_t dir_idx) const { return ADJ_CELL_FROM_VERTEX[dir_idx].rel_key; }

        // returns the vertex value at the boundary of the map, in the direction in_dir_idx
        inline const int_t &getBoundaryVertexValue(const dir_idx_t &in_dir_idx) const { return BOUNDS[in_dir_idx]; }

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
                return x >= 0 && y >= 0 && x < size[0] && y < size[1];
            else
                return x >= 0 && y >= 0 && x <= size[0] && y <= size[1];
        }

        // checks if coordinate of vertex or cell is in the map
        template <bool is_cell>
        inline bool inMap(const V2 &coord) const { return inMap<is_cell>(coord[0], coord[1]); }

        // returns occupancy without checking bounds.
        inline bool isOc(const mapkey_t &key) const { return data[key]; }

        // returns true if cell is free and within map bounds
        inline bool isAccessible(const mapkey_t &key, const int_t &x, const int_t &y) const { return inMap<true>(x, y) ? !isOc(key) : false; }

        // returns true if cell is free and within map bounds.
        inline bool isAccessible(const mapkey_t &key, const V2 &coord) const { return isAccessible(key, coord[0], coord[1]); }

        // returns true if cell is free and within map bounds.
        inline bool isAccessible(const mapkey_t &key) const
        {
            int_t x, y;
            keyToCoord(key, x, y);
            return isAccessible(key, x, y);
        }

        // returns true if cell is free and within map bounds.
        inline bool isAccessible(const int_t &x, const int_t &y) const
        {
            // returns true if pixel (x,y) is free and in map.
            mapkey_t key = coordToKey(x, y);
            return isAccessible(key, x, y);
        }

        // returns true if cell is free and within map bounds.
        inline bool isAccessible(const V2 &coord) const { return isAccessible(coord[0], coord[1]); }
    };

}