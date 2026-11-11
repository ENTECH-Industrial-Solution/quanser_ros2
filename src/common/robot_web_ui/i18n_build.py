#!/usr/bin/env python3
"""Build a dictionary for i18n.js from a .tsv (Thai phrase <TAB> English, '#' lines are comments).

    python3 i18n_build.py                                   # robot_web_ui: i18n_en.tsv -> static/ui_en.js
    python3 i18n_build.py my.tsv static/i18n_en.js RUI_EN   # a robot's own dictionary

Then copy the .js into the install (or colcon build). A robot's page loads its dictionary
(window.RUI_EN) before /ui/i18n.js; robot_web_ui's own (window.RUI_EN_CORE) is /ui/ui_en.js.
"""
import json
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, 'i18n_en.tsv')
dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, 'static', 'ui_en.js')
var = sys.argv[3] if len(sys.argv) > 3 else 'RUI_EN_CORE'

pairs = {}
with open(src, encoding='utf-8') as f:
    for line in f:
        line = line.rstrip('\n')
        if not line or line.startswith('#') or '\t' not in line:
            continue
        th, en = line.split('\t', 1)
        pairs[th] = en
with open(dst, 'w', encoding='utf-8') as f:
    f.write(f'// Generated from {os.path.basename(src)} by i18n_build.py: Thai phrase -> English (used by i18n.js).\n')
    f.write(f'window.{var} = ' + json.dumps(pairs, ensure_ascii=False, indent=0) + ';\n')
print(f'{len(pairs)} phrases -> {dst}')
