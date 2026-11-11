#!/usr/bin/env python3
"""Rebuild static/i18n_en.js from i18n_en.tsv (Thai phrase <TAB> English, '#' lines are comments).

    python3 i18n_build.py      # then copy static/i18n_en.js to the install, or colcon build

The page also loads robot_web_ui's own dictionary (/ui/ui_en.js); this one wins over it.
Phrases mixing Thai with numbers/English words: static/i18n_pre.js.
"""
import json
import os

here = os.path.dirname(os.path.abspath(__file__))
pairs = {}
with open(os.path.join(here, 'i18n_en.tsv'), encoding='utf-8') as f:
    for line in f:
        line = line.rstrip('\n')
        if not line or line.startswith('#') or '\t' not in line:
            continue
        th, en = line.split('\t', 1)
        pairs[th] = en
with open(os.path.join(here, 'static', 'i18n_en.js'), 'w', encoding='utf-8') as f:
    f.write('// Generated from i18n_en.tsv by i18n_build.py: Thai phrase -> English (used by i18n.js).\n')
    f.write('window.RUI_EN = ' + json.dumps(pairs, ensure_ascii=False, indent=0) + ';\n')
print(f'{len(pairs)} phrases')
