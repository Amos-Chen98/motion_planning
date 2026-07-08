// Offline reproduction of the safe-flight-corridor vertex-enumeration
// degeneracy behind the 2026-06-23 Naraha "bad trajectory" flight.
//
// The corridor H-representation produced by sfc_gen::convexCover() is always
// clipped to the map bound, but both the trajectory optimizer
// (GCOPTER_PolytopeSFC::processCorridor) and the corridor visualization
// (Visualizer::visualizePolytope) consume a V-representation computed by
// geo_utils::enumerateVs(). With the nearly parallel / duplicated planes that
// short horizon-truncated routes and voxel-grid surface points produce,
// enumerateVs() silently returns spurious vertices located far outside the
// polytope. This program runs the exact corridor pipeline from the gcopter
// package on synthetic scenes and counts those invalid vertices.
//
// See gcopter/test/README_GCOPTER_BAD_TRAJ.md for the full causal chain.
//
// Exit code: 0 if every enumerated vertex satisfies its H-constraints,
// 1 if the degeneracy was reproduced (expected with the current gcopter).

#include "gcopter/geo_utils.hpp"
#include "gcopter/sfc_gen.hpp"

#include <Eigen/Eigen>

#include <iostream>
#include <random>
#include <vector>

namespace
{

struct EnumerationCheck
{
    bool enumerationSucceeded = false;
    int vertexCount = 0;
    double maxAbsCoordinate = 0.0;
    // Largest violation of the polytope's own half-space constraints by an
    // enumerated vertex. A correct V-representation keeps this near zero;
    // meters-scale values are numerically invalid vertices.
    double maxConstraintViolation = 0.0;
};

EnumerationCheck checkEnumeration(const Eigen::MatrixX4d &hPoly)
{
    EnumerationCheck result;
    Eigen::Matrix3Xd vPoly;
    result.enumerationSucceeded = geo_utils::enumerateVs(hPoly, vPoly);
    if (!result.enumerationSucceeded || vPoly.cols() == 0)
    {
        return result;
    }
    result.vertexCount = vPoly.cols();
    result.maxAbsCoordinate = vPoly.cwiseAbs().maxCoeff();
    for (int i = 0; i < vPoly.cols(); ++i)
    {
        const double violation =
            (hPoly.leftCols<3>() * vPoly.col(i) + hPoly.rightCols<1>())
                .maxCoeff();
        result.maxConstraintViolation =
            std::max(result.maxConstraintViolation, violation);
    }
    return result;
}

// Row normalization applied by GCOPTER_PolytopeSFC::setup() before
// processCorridor() enumerates the polytopes.
void normalizeRows(Eigen::MatrixX4d &hPoly)
{
    const Eigen::ArrayXd norms = hPoly.leftCols<3>().rowwise().norm();
    hPoly.array().colwise() /= norms;
}

// Voxel-grid ground slab surface as voxel_map::VoxelMap::getSurf() returns
// it after dilation: voxel centers on a regular grid.
std::vector<Eigen::Vector3d> groundSurface(const double halfExtent,
                                           const double voxelWidth)
{
    std::vector<Eigen::Vector3d> surf;
    for (double x = -halfExtent; x <= halfExtent; x += voxelWidth)
    {
        for (double y = -halfExtent; y <= halfExtent; y += voxelWidth)
        {
            surf.emplace_back(x + 0.5 * voxelWidth,
                              y + 0.5 * voxelWidth,
                              0.5 * voxelWidth);
        }
    }
    return surf;
}

struct ScenarioStats
{
    int enumerations = 0;
    int failures = 0;
    int invalid = 0; // any vertex violating its constraints beyond tolerance
    int gross = 0;   // violation above 1 m: vertices on the map scale or worse
    double worstViolation = 0.0;
    double worstCoordinate = 0.0;

    void add(const EnumerationCheck &check, const double violationTol)
    {
        ++enumerations;
        if (!check.enumerationSucceeded)
        {
            ++failures;
            return;
        }
        worstViolation = std::max(worstViolation, check.maxConstraintViolation);
        worstCoordinate = std::max(worstCoordinate, check.maxAbsCoordinate);
        if (check.maxConstraintViolation > violationTol)
        {
            ++invalid;
        }
        if (check.maxConstraintViolation > 1.0)
        {
            ++gross;
        }
    }

    void print(const char *name) const
    {
        std::cout << name << ": " << enumerations << " enumerations, "
                  << failures << " hard failures, "
                  << invalid << " with invalid vertices "
                  << "(" << gross << " violating by > 1 m), "
                  << "worst constraint violation " << worstViolation << " m, "
                  << "worst |vertex coordinate| " << worstCoordinate << " m\n";
    }
};

// Corridor built exactly as PlannerBackend::buildCorridor() does for the
// horizon-truncated online routes flown at Naraha: convexCover with
// progress 7.0 and range 3.0, then shortCut (which duplicates a lone
// polytope). Routes are 3 m long with 1-2 segments, matching a 3 m
// planning horizon.
std::vector<Eigen::MatrixX4d> buildCorridor(
    const std::vector<Eigen::Vector3d> &route,
    const std::vector<Eigen::Vector3d> &surf,
    const Eigen::Vector3d &lowCorner,
    const Eigen::Vector3d &highCorner)
{
    std::vector<Eigen::MatrixX4d> hPolys;
    sfc_gen::convexCover(route, surf, lowCorner, highCorner, 7.0, 3.0, hPolys);
    sfc_gen::shortCut(hPolys);
    return hPolys;
}

} // namespace

int main()
{
    const Eigen::Vector3d lowCorner(-8.0, -8.0, -0.5);
    const Eigen::Vector3d highCorner(8.0, 8.0, 6.0);
    const double violationTol = 1.0e-3; // meters

    std::mt19937 rng(1);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    // Scenario A: the optimizer path. processCorridor() stacks each pair of
    // consecutive polytopes and enumerates the overlap; the MINCO inner
    // waypoint is then parameterized inside the convex hull of the vertices
    // it returns, so one spurious vertex lets the optimizer place the
    // waypoint far outside the map (x = -161 m in the recorded bag).
    ScenarioStats overlapStats;
    {
        const std::vector<Eigen::Vector3d> surf = groundSurface(8.0, 0.1);
        for (int trial = 0; trial < 60; ++trial)
        {
            const Eigen::Vector3d start(-0.09 + 0.1 * u(rng),
                                        -0.12 + 0.1 * u(rng),
                                        0.4 + 0.4 * (u(rng) + 1.0));
            const Eigen::Vector3d target(-0.35 + 0.3 * u(rng),
                                         -1.14 + 0.3 * u(rng),
                                         2.73 + 0.3 * u(rng));
            std::vector<Eigen::Vector3d> route{start, target};
            if (trial % 2 == 1)
            {
                // intermediate RRT* state, as most recorded routes have
                Eigen::Vector3d mid = 0.5 * (start + target);
                mid += Eigen::Vector3d(0.3 * u(rng), 0.3 * u(rng), 0.3 * u(rng));
                route = {start, mid, target};
            }

            std::vector<Eigen::MatrixX4d> hPolys =
                buildCorridor(route, surf, lowCorner, highCorner);
            for (Eigen::MatrixX4d &hPoly : hPolys)
            {
                normalizeRows(hPoly);
            }
            for (size_t i = 0; i + 1 < hPolys.size(); ++i)
            {
                Eigen::MatrixX4d stacked(
                    hPolys[i].rows() + hPolys[i + 1].rows(), 4);
                stacked.topRows(hPolys[i].rows()) = hPolys[i];
                stacked.bottomRows(hPolys[i + 1].rows()) = hPolys[i + 1];
                const EnumerationCheck check = checkEnumeration(stacked);
                overlapStats.add(check, violationTol);
                if (check.maxConstraintViolation > 1.0)
                {
                    std::cout << "  [overlap] trial " << trial
                              << ": planes=" << stacked.rows()
                              << " vertices=" << check.vertexCount
                              << " maxViolation="
                              << check.maxConstraintViolation
                              << " m, farthest vertex coordinate "
                              << check.maxAbsCoordinate << " m\n";
                }
            }
        }
    }

    // Scenario B: the visualization path. visualizePolytope() enumerates
    // every corridor polytope on its own; sparse airborne returns (rain,
    // dust, vegetation) make single-polytope enumeration degenerate too,
    // which is why the corridor mesh in RViz appeared to extend beyond the
    // map bound.
    ScenarioStats singleStats;
    {
        for (int trial = 0; trial < 200; ++trial)
        {
            std::vector<Eigen::Vector3d> surf = groundSurface(6.0, 0.1);
            const int clutterCount = 30 + static_cast<int>(100 * u01(rng));
            for (int i = 0; i < clutterCount; ++i)
            {
                surf.emplace_back(6.0 * u(rng), 6.0 * u(rng),
                                  0.3 + 5.0 * u01(rng));
            }
            const Eigen::Vector3d start(-0.09 + 0.1 * u(rng),
                                        -0.12 + 0.1 * u(rng),
                                        0.4 + 0.4 * (u(rng) + 1.0));
            const Eigen::Vector3d target(-0.35 + 0.3 * u(rng),
                                         -1.14 + 0.3 * u(rng),
                                         2.73 + 0.3 * u(rng));

            const std::vector<Eigen::MatrixX4d> hPolys = buildCorridor(
                {start, target}, surf, lowCorner, highCorner);
            for (const Eigen::MatrixX4d &hPoly : hPolys)
            {
                const EnumerationCheck check = checkEnumeration(hPoly);
                singleStats.add(check, violationTol);
                if (check.maxConstraintViolation > 1.0 &&
                    singleStats.gross <= 5)
                {
                    std::cout << "  [single] trial " << trial
                              << ": planes=" << hPoly.rows()
                              << " vertices=" << check.vertexCount
                              << " maxViolation="
                              << check.maxConstraintViolation
                              << " m, farthest vertex coordinate "
                              << check.maxAbsCoordinate << " m\n";
                }
            }
        }
    }

    std::cout << "\n";
    overlapStats.print("Scenario A (processCorridor overlap enumeration)");
    singleStats.print("Scenario B (visualizePolytope single enumeration)");

    const bool reproduced =
        overlapStats.invalid + singleStats.invalid +
            overlapStats.failures + singleStats.failures > 0;
    std::cout << (reproduced
                      ? "\nDegeneracy REPRODUCED: enumerateVs returned "
                        "vertices violating their own half-space constraints.\n"
                      : "\nNo degeneracy observed.\n");
    return reproduced ? 1 : 0;
}
