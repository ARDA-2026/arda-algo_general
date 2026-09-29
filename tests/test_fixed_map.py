"""Map bounds and takeoff position must not follow a new drop location."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


class FixedMapTests(unittest.TestCase):
    def test_drop_reset_preserves_map_and_takeoff(self):
        source = Path(__file__).resolve().parents[1] / "hanriver.py"
        tree = ast.parse(source.read_text(encoding="utf-8-sig"))
        names = {"_rebuild_map", "takeoff_from_offset", "_reset_to_origin"}
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
        ns = dict(np=np, basemap=SimpleNamespace(warm=lambda *args: None),
                  DEFAULT_MAPO_LON=126.9364, DEFAULT_MAPO_LAT=37.5336,
                  MAPO_LON=126.9364, MAPO_LAT=37.5336,
                  MAP_SCALE=600, MAP_PRINT_W=3, MAP_PRINT_H=2,
                  MAP_EAST_M=442.5, MAP_SOUTH_M=126,
                  M_PER_DEG_LON=88000, M_PER_DEG_LAT=111000,
                  GRID_CELL_M=15, takeoff_off_e=0.5, takeoff_off_n=0.2,
                  N=200, RADIUS_DEG=10 / 111000, TURBULENCE=0.3,
                  velocity_x=-1.5, velocity_y=0.05, PRINT_INTERVAL=180,
                  particles_lon=np.zeros(200), particles_lat=np.zeros(200),
                  pvlon=np.zeros(200), pvlat=np.zeros(200),
                  filter_in_river=lambda x, y: np.ones(len(x), dtype=bool))
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), ns)
        ns["_rebuild_map"]()
        keys = ("map_lon_min", "map_lon_max", "map_lat_min", "map_lat_max",
                "takeoff_lon", "takeoff_lat")
        initial = [ns[k] for k in keys]
        grid_x, grid_y = ns["GRID_XEDGES"].copy(), ns["GRID_YEDGES"].copy()
        for lon, lat in [(126.932, 37.540), (126.938, 37.535), (126.9364, 37.5336)]:
            ns["accumulated_hist"][:] = 10
            ns["_reset_to_origin"](lon, lat)
            self.assertEqual([ns[k] for k in keys], initial)
            np.testing.assert_array_equal(ns["GRID_XEDGES"], grid_x)
            np.testing.assert_array_equal(ns["GRID_YEDGES"], grid_y)
            self.assertEqual((ns["MAPO_LON"], ns["MAPO_LAT"]), (lon, lat))
            self.assertTrue(np.all(abs(ns["particles_lon"] - lon) <= ns["RADIUS_DEG"]))
            self.assertTrue(np.all(abs(ns["particles_lat"] - lat) <= ns["RADIUS_DEG"]))
            self.assertEqual(ns["accumulated_hist"].sum(), 0)
        # Explicit map settings remain effective, independently of drop position.
        ns["_rebuild_map"](scale=300)
        self.assertAlmostEqual(ns["map_lon_max"] - ns["map_lon_min"], 900 / 88000)


if __name__ == "__main__":
    unittest.main()
