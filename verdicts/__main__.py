import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from verdicts.engine import evaluate, render_text, render_md

ap = argparse.ArgumentParser(description='标定判决报告')
ap.add_argument('--rules', default=os.path.join(os.path.dirname(__file__), 'rules_d435i.yaml'))
ap.add_argument('--md', help='写 Markdown 报告到此路径')
ap.add_argument('--json', dest='json_out', help='写 JSON 到此路径')
a = ap.parse_args()
res = evaluate(a.rules)
print(render_text(res))
if a.md:
    with open(a.md, 'w', encoding='utf-8') as f:
        f.write(render_md(res))
    print(f'\n已写 {a.md}')
if a.json_out:
    with open(a.json_out, 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print(f'已写 {a.json_out}')
