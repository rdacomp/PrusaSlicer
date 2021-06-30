#ifndef slic3r_PointGrid3D_hpp_
#define slic3r_PointGrid3D_hpp_

#include <unordered_map>
#include "Point.hpp"

namespace Slic3r {

/// <summary>
/// Store space points inside grid
/// Speed up for detection close points
/// </summary>
class PointGrid3D {
    struct GridHash {
        std::size_t operator()(const Vec3i &cell_id) const {
            return std::hash<int>()(cell_id.x()) ^ std::hash<int>()(cell_id.y() * 593) ^ std::hash<int>()(cell_id.z() * 7919);
        }
    };
    typedef std::unordered_multimap<Vec3i, Vec3f, GridHash> Grid;
        
    Vec3f   cell_size;
    Grid    grid;

    static const std::vector<Vec3i> sorted_neighbors;
    static const std::vector<Vec3i> sorted17neighbors;
public:
    PointGrid3D(Vec3f cell_size): cell_size(cell_size) {}
    void insert(const Vec3f &pos) { grid.emplace(cell_id(pos), pos); }

    /// <summary>
    /// Check when exist point inside grid around pos with radius
    /// </summary>
    /// <param name="pos">Position in 3d space</param>
    /// <param name="radius">Radius to check</param>
    /// <returns>TRUE when exist point in distance radius otherwise FALSE</returns>
    bool collides_with(const Vec3f &pos, float radius) const{
        return collides_with(pos, radius, sorted_neighbors);
    }

    /// <summary>
    /// Check when exist point with lower z inside grid around pos with radius
    /// Use for layer add of points
    /// </summary>
    /// <param name="pos">Position in 3d space</param>
    /// <param name="radius">Radius to check</param>
    /// <returns>TRUE when exist point in distance radius otherwise FALSE</returns>
    bool collides_with_lower_z(const Vec3f &pos, float radius) const {
        return collides_with(pos, radius, sorted17neighbors);
    }        
private:
    Vec3i cell_id(const Vec3f &pos) const;
    bool collides_with(const Vec3f &             pos,
                       float                     radius,
                       const std::vector<Vec3i> &sorted_neighbors) const;
    bool collides_with(const Vec3f &                          pos,
                       float                                  squered_radius,
                       const std::pair<Grid::const_iterator,
                                       Grid::const_iterator> &pair) const;
};
} // namespace Slic3r

#endif