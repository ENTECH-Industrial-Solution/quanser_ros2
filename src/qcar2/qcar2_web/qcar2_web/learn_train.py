"""Training / export worker for the web page's "เรียนรู้" (learn) page. Runs as its own process.

    python3 learn_train.py train  <run_dir>   (reads <run_dir>/job.json, writes progress.json)
    python3 learn_train.py export <run_dir>

A separate process so a long training never blocks the web node, and stopping it is a kill.
Everything stays on this machine: ultralytics runs offline (no downloads, no usage reports).
"""

import json
import os
import sys
import time
import traceback

os.environ['YOLO_OFFLINE'] = 'true'   # before ultralytics is imported


def _write(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f)
    os.replace(tmp, path)


def _num(v):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return round(v, 5) if v == v else None


def train(run_dir):
    with open(os.path.join(run_dir, 'job.json')) as f:
        job = json.load(f)
    prog_path = os.path.join(run_dir, 'progress.json')
    prog = {'state': 'starting', 'epoch': 0, 'epochs': job['epochs'], 'history': [],
            'started': time.time(), 'error': None}
    _write(prog_path, prog)

    import ultralytics.engine.trainer as trainer_mod
    from ultralytics import YOLO
    # The AMP self-test downloads yolov8n.pt; AMP works on the Jetson, so skip the test.
    trainer_mod.check_amp = lambda model: True

    if job.get('weights_mode') == 'yaml+load':
        model = YOLO(job['yaml']).load(job['weights'])
    else:
        model = YOLO(job['weights'])

    def on_epoch(tr):
        m = tr.metrics or {}
        loss = tr.tloss.tolist() if hasattr(tr.tloss, 'tolist') else []
        prog['history'].append({
            'epoch': tr.epoch + 1,
            'box_loss': _num(loss[0]) if len(loss) > 0 else None,
            'cls_loss': _num(loss[1]) if len(loss) > 1 else None,
            'map50': _num(m.get('metrics/mAP50(B)')),
            'map50_mask': _num(m.get('metrics/mAP50(M)')),
            'map': _num(m.get('metrics/mAP50-95(B)')),
            'precision': _num(m.get('metrics/precision(B)')),
            'recall': _num(m.get('metrics/recall(B)')),
        })
        prog.update(state='training', epoch=tr.epoch + 1, epochs=tr.epochs)
        _write(prog_path, prog)

    def on_train_start(tr):
        prog.update(state='training', epochs=tr.epochs)
        _write(prog_path, prog)

    model.add_callback('on_train_start', on_train_start)
    model.add_callback('on_fit_epoch_end', on_epoch)
    model.train(data=job['data'], epochs=job['epochs'], imgsz=job['imgsz'], batch=job['batch'],
                device=0, workers=2, project=os.path.dirname(run_dir),
                name=os.path.basename(run_dir), exist_ok=True, plots=False, verbose=False,
                patience=job.get('patience', max(10, job['epochs'] // 3)), amp=True, cache=False)
    prog.update(state='done', finished=time.time())
    _write(prog_path, prog)


def export(run_dir):
    prog_path = os.path.join(run_dir, 'export.json')
    _write(prog_path, {'state': 'exporting', 'started': time.time()})
    from ultralytics import YOLO
    with open(os.path.join(run_dir, 'info.json')) as f:
        info = json.load(f)
    best = os.path.join(run_dir, 'weights', 'best.pt')
    out = YOLO(best).export(format='engine', half=True, imgsz=info.get('imgsz', 640), device=0,
                            verbose=False)
    _write(prog_path, {'state': 'done', 'engine': str(out), 'finished': time.time()})


def main():
    what, run_dir = sys.argv[1], sys.argv[2]
    try:
        (train if what == 'train' else export)(run_dir)
    except BaseException as e:  # noqa: BLE001 - reported to the page
        name = 'progress.json' if what == 'train' else 'export.json'
        path = os.path.join(run_dir, name)
        try:
            with open(path) as f:
                prog = json.load(f)
        except (OSError, ValueError):
            prog = {}
        prog.update(state='error', error=f'{type(e).__name__}: {e}', finished=time.time())
        _write(path, prog)
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
