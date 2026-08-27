"""引擎单测:伪造数据源,覆盖 提取/三种比较/容差两形态/缺文件 pending/表达式失败 pending。"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from verdicts.engine import evaluate

d = tempfile.mkdtemp()
os.makedirs(f'{d}/data'); os.makedirs(f'{d}/rules')
open(f'{d}/data/a.yaml', 'w').write("cam1:\n  T: [[1,0,0,0.0501],[0,1,0,0],[0,0,1,0]]\n")
json.dump(dict(base=50.0, vals=[1.0, 1.2, 0.9]), open(f'{d}/data/b.json', 'w'))
open(f'{d}/data/c.txt', 'w').write("mean 4.087, median 1.9\nmean 6.416, median 2.6\n")
rules = """
sources:
  a: {file: data/a.yaml, format: yaml}
  b: {file: data/b.json, format: json}
  c: {file: data/c.txt, format: text}
  gone: {file: data/nope.yaml, format: yaml}
checks:
  - {id: t1, stage: s1, name: within-rel-pass,
     value: {expr: "abs(s['a']['cam1']['T'][0][3])*1000", unit: mm},
     reference: {expr: "s['b']['base']", label: fac},
     compare: within, tol: {rel: 1.0}, verdict_pass: FREEZE, verdict_fail: NO}
  - {id: t2, stage: s1, name: within-abs-fail-bad,
     value: {expr: "abs(s['a']['cam1']['T'][0][3])*1000"},
     reference: {expr: "s['b']['base']"},
     compare: within, tol: {abs: 0.05}, severity_fail: bad, verdict_fail: RECAP}
  - {id: t3, stage: s2, name: le-pass,
     value: {expr: "spread(s['b']['vals'])"}, reference: {expr: "0.5"}, compare: le}
  - {id: t4, stage: s2, name: ge-fail,
     value: {expr: "mean(s['b']['vals'])"}, reference: {expr: "2.0"}, compare: ge}
  - {id: t5, stage: s2, name: rx-extract,
     value: {expr: "rx(s['c'], 'mean ([0-9.]+)', 1)"}, reference: {expr: "6.4"},
     compare: within, tol: {abs: 0.1}}
  - {id: t6, stage: s3, name: missing-file,
     value: {expr: "s['gone']['x']"}, reference: {expr: "1"}, compare: within, tol: {abs: 1}}
  - {id: t7, stage: s3, name: bad-key,
     value: {expr: "s['b']['no_such']"}, reference: {expr: "1"}, compare: within, tol: {abs: 1}}
"""
open(f'{d}/rules/r.yaml', 'w').write(rules)
res = {r['id']: r for r in evaluate(f'{d}/rules/r.yaml')}
assert res['t1']['status'] == 'ok' and res['t1']['verdict'] == 'FREEZE', res['t1']
assert abs(res['t1']['value'] - 50.1) < 1e-6
assert res['t2']['status'] == 'bad' and res['t2']['verdict'] == 'RECAP'
assert res['t3']['status'] == 'ok' and abs(res['t3']['value'] - 0.3) < 1e-9
assert res['t4']['status'] == 'warn'
assert res['t5']['status'] == 'ok' and abs(res['t5']['value'] - 6.416) < 1e-9
assert res['t6']['status'] == 'pending' and 'nope.yaml' in res['t6']['detail']
assert res['t7']['status'] == 'pending'
print('7/7 断言全过')
