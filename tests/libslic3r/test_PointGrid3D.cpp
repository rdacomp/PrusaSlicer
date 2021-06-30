#include <catch2/catch.hpp>
#include <test_utils.hpp>
#include <libslic3r/PointGrid3D.hpp>

using namespace Slic3r;

TEST_CASE("Collision of points", "[PointGrid3D]") { 
    Vec3f       cell_size(1.f, 1.f, 1.f);
	PointGrid3D pg(cell_size);
    Vec3f       point(.5f, .5f, .5f);
    pg.insert(point);

    float              radius = 0.9f;
    float              move   = 0.8f;
    float              diagonal_move = sqrt(0.8 * 0.8 / 2); 
    float              space_diagonal = sqrt(0.8 * 0.8 / 3);
    std::vector<Vec3f> diffs = {
        Vec3f(move, 0.f, 0.f),
        Vec3f(0.f, move, 0.f),
        Vec3f(0.f, 0.f, move),
        Vec3f(diagonal_move, diagonal_move, 0.f),
        Vec3f(diagonal_move, -diagonal_move, 0.f),
        Vec3f(diagonal_move, 0.f, diagonal_move),
        Vec3f(diagonal_move, 0.f, -diagonal_move),
        Vec3f(0.f, diagonal_move, diagonal_move),
        Vec3f(0.f, diagonal_move, -diagonal_move),
        Vec3f(space_diagonal, space_diagonal, space_diagonal),
        Vec3f(space_diagonal, space_diagonal, -space_diagonal),
        Vec3f(space_diagonal, -space_diagonal, space_diagonal),
        Vec3f(-space_diagonal, space_diagonal, space_diagonal)
    };

    for (auto &diff : diffs)
    {
	    CHECK(pg.collides_with(point + diff, radius));
        CHECK(pg.collides_with(point - diff, radius));
    }
}
