"""Offline boundary regression tests; do not start the server or fetch OSM data."""
import ast
from pathlib import Path
import unittest

import numpy as np
from shapely import contains_xy, covers, linestrings
from shapely.geometry import Point, Polygon, box
from shapely.ops import nearest_points


class ParticleBoundaryTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).resolve().parents[1] / "hanriver.py"
        tree = ast.parse(source.read_text(encoding="utf-8-sig"))
        # Import only the actual movement functions: module startup fetches OSM
        # and initializes the live server, neither of which belongs in this test.
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name in {"filter_in_river", "advance_particles"}]
        self.ns = dict(np=np, contains_xy=contains_xy, covers=covers,
                       linestrings=linestrings, Point=Point,
                       nearest_points=nearest_points)
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), "exec"), self.ns)

    def configure(self, river, points):
        coords = np.array(points, dtype=float)
        self.ns.update(hangang_union=river, particles_lon=coords[:, 0].copy(),
                       particles_lat=coords[:, 1].copy(),
                       in_river=contains_xy(river, coords[:, 0], coords[:, 1]),
                       stranded_lons=[], stranded_lats=[])

    def move(self, dx, dy):
        self.ns["advance_particles"](np.array(dx), np.array(dy))

    def test_open_water_keeps_original_displacement(self):
        self.configure(box(0, 0, 10, 10), [(2, 2), (4, 4)])
        self.move([0.25, -0.5], [0.1, 0])
        np.testing.assert_allclose(self.ns["particles_lon"], [2.25, 3.5])
        np.testing.assert_allclose(self.ns["particles_lat"], [2.1, 4])
        self.assertTrue(self.ns["in_river"].all())
        self.assertEqual(self.ns["stranded_lons"], [])

    def test_bank_contact_stays_fixed_and_is_counted_once(self):
        self.configure(box(0, 0, 10, 10), [(9, 5), (3, 5)])
        self.move([3, 1], [0, 0])
        np.testing.assert_allclose(self.ns["particles_lon"], [10, 4])
        np.testing.assert_array_equal(self.ns["in_river"], [False, True])
        self.move([-5, 1], [2, 0])
        np.testing.assert_allclose(self.ns["particles_lon"], [10, 5])
        np.testing.assert_allclose(self.ns["particles_lat"], [5, 5])
        self.assertEqual(self.ns["stranded_lons"], [10])

    def test_cannot_jump_across_island_into_water(self):
        river = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)],
                        holes=[[(4, 4), (6, 4), (6, 6), (4, 6)]])
        self.configure(river, [(2, 5)])
        self.move([6], [0])  # Endpoint (8, 5) is water, path crosses island.
        np.testing.assert_allclose(self.ns["particles_lon"], [4])
        self.assertFalse(self.ns["in_river"][0])
        self.assertEqual(self.ns["stranded_lons"], [4])

    def test_exact_boundary_contact_and_zero_motion(self):
        self.configure(box(0, 0, 10, 10), [(9, 5), (3, 5)])
        self.move([1, 0], [0, 0])
        np.testing.assert_array_equal(self.ns["in_river"], [False, True])
        np.testing.assert_allclose(self.ns["particles_lon"], [10, 3])

    def test_initial_land_particles_never_move_into_water(self):
        self.configure(box(0, 0, 10, 10), [(11, 5)])
        self.move([-5], [0])
        np.testing.assert_allclose(self.ns["particles_lon"], [11])
        self.assertFalse(self.ns["in_river"][0])
        self.assertEqual(self.ns["stranded_lons"], [])


if __name__ == "__main__":
    unittest.main()
