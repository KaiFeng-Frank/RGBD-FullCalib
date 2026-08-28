#!/usr/bin/env python3
"""判决引擎:标定结果的外部参照核查,规则在数据里,不在代码里。

为什么存在:
  重投影误差只说明模型拟合了喂给它的数据。"这套参数能不能用"需要对照
  优化之外的参照 —— 出厂值、物理常数、多次独立测量的散布。此前这些判决
  以散文形式写在 CALIBRATION.md 和 GUI 代码里:别人 clone 下来拿到的是
  我们的故事,不是他们自己的判决。规则化之后,加一台设备 = 写一份 yaml,
  不是 fork 一份代码。

规则文件结构(yaml):
  sources:                # 命名数据源
    ir_chain: {file: data/cam_ir-camchain.yaml, format: yaml}
    results_txt: {file: data/camimu-results-imucam.txt, format: text}
  checks:
    - id: baseline
      stage: stereo_ir            # 分组键,GUI 卡片按它归位
      name: 基线 vs 出厂
      value: {expr: "abs(s['ir_chain']['cam1']['T_cn_cnm1'][0][3])*1000", unit: mm}
      reference: {expr: "s['factory']['baseline_mm']", label: 出厂}
      compare: within             # within | le | ge
      tol: {rel: 0.5}             # 百分比;或 {abs: 0.5}(与 value 同单位)
      severity_fail: warn         # warn | bad
      verdict_pass: 可冻结
      verdict_fail: 先查靶标尺寸再重采(坑 #4)

表达式在受限命名空间求值:s(全部源)、abs/min/max/len、norm(向量模)、
spread(max-min)、mean、rx(text, pattern[, idx])(正则取数)。
任一源文件缺失或表达式取不到值 → 该条判 pending,不是报错:
只标了一半的仓库也能出报告,pending 就是占位。
"""
import json
import math
import os
import re
import sys

import yaml


def _norm(v):
    return math.sqrt(sum(float(x) * float(x) for x in v))


def _spread(v):
    v = [float(x) for x in v]
    return max(v) - min(v)


def _mean(v):
    v = [float(x) for x in v]
    return sum(v) / len(v)


def _rx(text, pattern, idx=0, group=0):
    """从文本源正则取数:第 idx 个匹配的第 group 个捕获组。"""
    m = re.findall(pattern, text)
    if not m:
        raise KeyError(f'regex 无匹配: {pattern}')
    g = m[idx]
    return float(g if isinstance(g, str) else g[group])


class Pending(Exception):
    pass


def load_sources(spec, base):
    out = {}
    missing = {}
    for name, cfg in spec.items():
        path = os.path.join(base, cfg['file'])
        if not os.path.exists(path):
            missing[name] = cfg['file']
            continue
        with open(path, encoding='utf-8') as f:
            raw = f.read()
        fmt = cfg.get('format', 'yaml')
        if fmt == 'yaml':
            out[name] = yaml.safe_load(raw)
        elif fmt == 'json':
            out[name] = json.loads(raw)
        else:
            out[name] = raw
    return out, missing


def _eval(expr, sources):
    ns = dict(s=sources, abs=abs, min=min, max=max, len=len,
              norm=_norm, spread=_spread, mean=_mean, rx=_rx)
    try:
        # 命名空间必须进 globals:listcomp 有独立作用域,只查 globals,
        # 放 locals 会让 "for c in [...]" 里的 norm/spread 变 NameError
        ns['__builtins__'] = {}
        return float(eval(expr, ns))
    except Pending:
        raise
    except Exception as e:
        raise Pending(f'{type(e).__name__}: {e}')


def run_check(ck, sources, missing):
    res = dict(id=ck['id'], stage=ck.get('stage', ''), name=ck['name'],
               unit=ck.get('value', {}).get('unit', ''))
    # 该条引用的源缺文件 -> pending
    need = set(re.findall(r"s\['([^']+)'\]",
                          ck['value']['expr'] + ck.get('reference', {}).get('expr', '')))
    lost = need & set(missing)
    if lost:
        res.update(status='pending', detail='缺 ' + ', '.join(missing[k] for k in lost))
        return res
    try:
        v = _eval(ck['value']['expr'], sources)
        r = _eval(ck['reference']['expr'], sources) if 'reference' in ck else None
    except Pending as e:
        res.update(status='pending', detail=str(e))
        return res
    res['value'] = v
    res['reference'] = r
    res['ref_label'] = ck.get('reference', {}).get('label', '')
    cmp_ = ck.get('compare', 'within')
    if cmp_ == 'within':
        tol = ck.get('tol', {})
        lim = tol['abs'] if 'abs' in tol else abs(r) * tol['rel'] / 100.0
        ok = abs(v - r) <= lim
        res['delta'] = v - r
        res['limit'] = lim
    elif cmp_ == 'le':
        ok = v <= r
    elif cmp_ == 'ge':
        ok = v >= r
    else:
        raise ValueError(f'未知 compare: {cmp_}')
    res['status'] = 'ok' if ok else ck.get('severity_fail', 'warn')
    res['verdict'] = ck.get('verdict_pass' if ok else 'verdict_fail', '')
    return res


def evaluate(rules_path):
    base = os.path.dirname(os.path.dirname(os.path.abspath(rules_path)))
    with open(rules_path, encoding='utf-8') as f:
        rules = yaml.safe_load(f)
    sources, missing = load_sources(rules.get('sources', {}), base)
    return [run_check(c, sources, missing) for c in rules.get('checks', [])]


def fmt_num(x, unit=''):
    if x is None:
        return '—'
    a = abs(x)
    s = f'{x:.4g}' if (a >= 0.01 or a == 0) else f'{x:.2e}'
    return f'{s} {unit}'.strip()


MARK = dict(ok='✓', warn='!', bad='✗', pending='…')


def render_text(results):
    lines = []
    stage = None
    n_ok = sum(1 for r in results if r['status'] == 'ok')
    n_bad = sum(1 for r in results if r['status'] in ('warn', 'bad'))
    n_pend = sum(1 for r in results if r['status'] == 'pending')
    for r in results:
        if r['stage'] != stage:
            stage = r['stage']
            lines.append(f'\n── {stage} ──')
        m = MARK[r['status']]
        if r['status'] == 'pending':
            lines.append(f'  {m} {r["name"]}: 待数据({r["detail"]})')
            continue
        body = f'{fmt_num(r.get("value"), r["unit"])}'
        if r.get('reference') is not None:
            body += f' vs {fmt_num(r["reference"], r["unit"])}({r.get("ref_label","")})'
        lines.append(f'  {m} {r["name"]}: {body} → {r.get("verdict","")}')
    lines.append(f'\n{n_ok} ✓   {n_bad} 需注意   {n_pend} 待数据')
    return '\n'.join(lines)


def render_md(results, title='Calibration Verdict Report'):
    out = [f'# {title}\n',
           '| | 检查 | 实测 | 参照 | 判决 |', '|---|---|---|---|---|']
    stage = None
    for r in results:
        if r['stage'] != stage:
            stage = r['stage']
            out.append(f'| | **{stage}** | | | |')
        m = MARK[r['status']]
        if r['status'] == 'pending':
            out.append(f'| {m} | {r["name"]} | 待数据 | | {r["detail"]} |')
        else:
            ref = fmt_num(r.get('reference'), r['unit'])
            if r.get('ref_label'):
                ref += f'({r["ref_label"]})'
            out.append(f'| {m} | {r["name"]} | {fmt_num(r.get("value"), r["unit"])} '
                       f'| {ref} | {r.get("verdict","")} |')
    return '\n'.join(out) + '\n'


def for_gui(results):
    """GUI 卡片格式:{stage: [[name, text, status], ...]}"""
    out = {}
    for r in results:
        if r['status'] == 'pending':
            txt = f'待数据({r["detail"]})'
        else:
            txt = fmt_num(r.get('value'), r['unit'])
            if r.get('reference') is not None:
                txt += f' vs {fmt_num(r["reference"], r["unit"])}'
            if r.get('verdict'):
                txt += f' —— {r["verdict"]}'
        st = {'pending': 'warn'}.get(r['status'], r['status'])
        out.setdefault(r['stage'], []).append([r['name'], txt, st])
    return out
