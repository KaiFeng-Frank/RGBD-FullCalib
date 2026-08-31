#!/usr/bin/env python3
"""v0.2 verdict pipeline integration test (offline, no hardware required).

Run directly from any working directory:

    python3 verdicts/test_v02_integration.py
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
RULES = ROOT / 'verdicts' / 'rules_d435i.yaml'
REPORT = ROOT / 'REPORT.md'
VERDICTS_JSON = ROOT / 'results' / 'verdicts.json'

sys.path.insert(0, str(ROOT))
from verdicts.engine import evaluate, for_gui, render_md  # noqa: E402


class VerdictPipelineIntegrationTest(unittest.TestCase):
    """Keep rules, CLI, committed report, and GUI verdicts in lockstep."""

    @classmethod
    def setUpClass(cls):
        cls.rule_doc = yaml.safe_load(RULES.read_text(encoding='utf-8'))
        cls.results = evaluate(str(RULES))

    def test_rules_have_26_unique_checks(self):
        checks = self.rule_doc['checks']
        ids = [check['id'] for check in checks]

        self.assertEqual(len(checks), 26)
        self.assertEqual(len(ids), len(set(ids)), f'duplicate rule ids: {ids}')
        self.assertEqual([result['id'] for result in self.results], ids)

    def test_committed_markdown_matches_rules(self):
        self.assertEqual(
            render_md(self.results),
            REPORT.read_text(encoding='utf-8'),
            'REPORT.md is stale; regenerate it from verdicts/rules_d435i.yaml',
        )

    def test_committed_json_matches_rules(self):
        self.assertEqual(
            json.loads(VERDICTS_JSON.read_text(encoding='utf-8')),
            self.results,
            'results/verdicts.json is stale; regenerate it from the verdict CLI',
        )

    def test_gui_projection_matches_evaluate(self):
        gui = for_gui(self.results)
        expected_stages = list(dict.fromkeys(r['stage'] for r in self.results))

        self.assertEqual(list(gui), expected_stages)
        self.assertEqual(sum(len(rows) for rows in gui.values()), len(self.results))

        projected = [
            (stage, name, status)
            for stage, rows in gui.items()
            for name, _text, status in rows
        ]
        expected = [
            (r['stage'], r['name'], 'warn' if r['status'] == 'pending' else r['status'])
            for r in self.results
        ]
        self.assertEqual(projected, expected)

        # Exercise the actual GUI consumer, not only the projection helper.
        from viewer import calib_summary

        self.assertEqual(Path(calib_summary._RULES).resolve(), RULES)
        # The viewer re-evaluates on each collection so newly completed
        # calibration artifacts appear without restarting the server.
        self.assertEqual(calib_summary._current_verdicts(), gui)

    def test_stereo_validation_stays_inside_the_existing_ir_card(self):
        from viewer import calib_summary

        summary = calib_summary.collect()
        ir_cards = [row for row in summary['stages'] if row['id'] == 'ir']
        self.assertEqual(len(ir_cards), 1)
        labels = [row[0] for row in ir_cards[0]['rows']]
        self.assertEqual(labels.count('原样 IR 硬件校正验收'), 1)
        self.assertNotIn('ir', {row['id'] for row in summary['pending']})
        check_names = [row[0] for row in ir_cards[0]['checks']]
        self.assertEqual(sum('原样 IR 双目' in name for name in check_names), 2)

    def test_cli_json_matches_engine_evaluate(self):
        with tempfile.TemporaryDirectory(prefix='d435i-verdict-test-') as tmp:
            json_path = Path(tmp) / 'verdicts.json'
            env = os.environ.copy()
            env['PYTHONDONTWRITEBYTECODE'] = '1'
            proc = subprocess.run(
                [sys.executable, '-m', 'verdicts', '--rules', str(RULES),
                 '--json', str(json_path)],
                cwd=ROOT,
                env=env,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            cli_results = json.loads(json_path.read_text(encoding='utf-8'))
            self.assertEqual(cli_results, self.results)


if __name__ == '__main__':
    unittest.main(verbosity=2)
