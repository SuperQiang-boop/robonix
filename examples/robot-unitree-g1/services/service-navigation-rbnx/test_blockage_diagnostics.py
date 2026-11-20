"""summarize_blockage turns a robot-centred grid into one actionable line."""
import unittest

from nav2_wrapper.diagnostics import summarize_blockage


class TestSummarizeBlockage(unittest.TestCase):
    def test_empty_grid_reports_no_blockage(self):
        line = summarize_blockage([0] * 25, 5, 5, 0.1)
        self.assertIn("no cells", line)

    def test_nearest_cell_distance_and_bearing(self):
        grid = [0] * 25
        grid[2 * 5 + 4] = 100  # due east of centre, 2 cells away
        line = summarize_blockage(grid, 5, 5, 0.1)
        self.assertIn("0.20m", line)
        self.assertIn("0deg", line)
        self.assertIn("1 blocked cells", line)

    def test_inscribed_inflation_counts_as_blocked(self):
        grid = [0] * 25
        grid[0] = 99  # inscribed-inflation value, top-left corner
        line = summarize_blockage(grid, 5, 5, 0.1)
        self.assertIn("nearest blocked cell", line)

    def test_degenerate_grid_is_silent(self):
        self.assertEqual(summarize_blockage([], 0, 0, 0.1), "")
        self.assertEqual(summarize_blockage([0] * 10, 5, 5, 0.1), "")


if __name__ == "__main__":
    unittest.main()
