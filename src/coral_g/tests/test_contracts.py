import unittest

from coral_g.contract_check import build_contract_fixtures
from coral_g.contracts import REQUIRED_FIELDS, validate


class ContractTests(unittest.TestCase):
    def test_all_contract_fixtures_validate(self):
        fixtures = build_contract_fixtures()
        self.assertEqual(set(REQUIRED_FIELDS), set(fixtures))
        for kind, message in fixtures.items():
            validate(kind, message)

    def test_material_distribution_starts_with_no_observations(self):
        material = build_contract_fixtures()["material_distribution"]
        self.assertEqual(material["status"], "no_observations")
        self.assertTrue(all(value == 0 for value in material["counts"].values()))
        self.assertTrue(all(value == 0.0 for value in material["probabilities"].values()))


if __name__ == "__main__":
    unittest.main()
