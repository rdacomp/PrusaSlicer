#include "PointGrid3D.hpp"

using namespace Slic3r;

const std::vector<Vec3i> PointGrid3D::sorted_neighbors = {
        // neighbor indexes
        Vec3i( 1,  0,  0), // left
        Vec3i(-1,  0,  0), // right
        Vec3i( 0,  1,  0), // top
        Vec3i( 0, -1,  0), // bottom
        Vec3i( 0,  0,  1), // above
        Vec3i( 0,  0, -1), // under
        // diagonal neighbor - current
        Vec3i( 1,  1,  0),
        Vec3i( 1, -1,  0),
        Vec3i(-1, -1,  0),
        Vec3i(-1,  1,  0),
        // diagonal neighbor - above
        Vec3i( 1,  0,  1),
        Vec3i( 0,  1,  1),
        Vec3i(-1,  0,  1),
        Vec3i( 0, -1,  1),
        // diagonal neighbor - under
        Vec3i( 1,  0, -1),
        Vec3i( 0,  1, -1),
        Vec3i(-1,  0, -1),
        Vec3i( 0, -1, -1),
        // space diagonal neighbor
        Vec3i( 1,  1,  1),
        Vec3i( 1, -1,  1),
        Vec3i(-1, -1,  1),
        Vec3i(-1,  1,  1),
        Vec3i( 1,  1, -1),
        Vec3i( 1, -1, -1),
        Vec3i(-1, -1, -1),
        Vec3i(-1,  1, -1)
};

const std::vector<Vec3i> PointGrid3D::sorted17neighbors = {
        // neighbor indexes
        Vec3i( 1,  0,  0), // left
        Vec3i(-1,  0,  0), // right
        Vec3i( 0,  1,  0), // top
        Vec3i( 0, -1,  0), // bottom
        Vec3i( 0,  0, -1), // under
        // diagonal neighbor
        Vec3i( 1,  1,  0),
        Vec3i( 1, -1,  0),
        Vec3i(-1, -1,  0),
        Vec3i(-1,  1,  0),
        Vec3i( 1,  0, -1),
        Vec3i( 0,  1, -1),
        Vec3i(-1,  0, -1),
        Vec3i( 0, -1, -1),
        // space diagonal neighbor
        Vec3i( 1,  1, -1),
        Vec3i( 1, -1, -1),
        Vec3i(-1, -1, -1),
        Vec3i(-1,  1, -1)
};

Vec3i PointGrid3D::cell_id(const Vec3f &pos) const
{
    return Vec3i(static_cast<int>(floor(pos.x() / cell_size.x())),
                 static_cast<int>(floor(pos.y() / cell_size.y())),
                 static_cast<int>(floor(pos.z() / cell_size.z())));
}

bool PointGrid3D::collides_with(const Vec3f &             pos,
                                float                     radius,
                                const std::vector<Vec3i> &sorted_neighbors) const
{
    // check neighbor only to distance cell size
    assert(radius < cell_size.x() && radius < cell_size.y() &&
           radius < cell_size.z());
    Vec3i cell           = cell_id(pos);
    float squered_radius = radius * radius;
    if (collides_with(pos, squered_radius, grid.equal_range(cell)))
        return true;
    for (const Vec3i &diffCoor : sorted_neighbors)
        if (collides_with(pos, squered_radius,
                          grid.equal_range(cell + diffCoor)))
            return true;
    return false;
}

bool PointGrid3D::collides_with(
    const Vec3f &pos,
    float        squered_radius,
    const std::pair<Grid::const_iterator, Grid::const_iterator> &pair) const
{
    for (Grid::const_iterator it = pair.first; it != pair.second; ++it) {
        float dist2 = (it->second - pos).squaredNorm();
        if (dist2 < squered_radius) return true;
    }
    return false;
}