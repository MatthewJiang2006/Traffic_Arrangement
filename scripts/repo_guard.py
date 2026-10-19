#!/usr/bin/env python3
"""Protect the Demo repository against accidental modification.

Locking makes every tracked file read-only. Any tool that tries to write into
the repository then fails at the filesystem level, and a commit is refused by
the pre-commit hook. Unlocking is a deliberate, logged action that must name
the reason and repeat the confirmation word.

This is a strong accident barrier, not a security boundary: every process runs
as the same user, so a script that runs ``unlock`` explicitly can still edit
the tree. The guard exists to make that impossible to do by accident.

Usage:
    ./scripts/repo_guard.py status
    ./scripts/repo_guard.py lock
    ./scripts/repo_guard.py unlock --reason "..." --confirm UNLOCK
"""

import argparse
import datetime
import json
import os
import stat
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MARKER = ROOT / '.repo-lock'
AUDIT = ROOT / '.git' / 'repo_access.log'
SKIP_PARTS = {'.git', 'runs', '__pycache__', '.pytest_cache', '.venv'}


def tracked_files():
    result = subprocess.run(['git', 'ls-files', '-z'], cwd=ROOT,
                            capture_output=True, text=True, check=True)
    names = [n for n in result.stdout.split('\0') if n]
    # untracked but intentional source/config files are protected too
    for pattern in ('config/**/*.json', 'scripts/**/*.py', 'src/**/*.py',
                    'tests/**/*.py', 'docs/**/*.md', 'docs/**/*.svg', 'docs/**/*.png',
                    'config/validation/**/*.json', 'run.sh', 'README.md', '.gitignore'):
        for path in ROOT.glob(pattern):
            if path.is_file():
                names.append(str(path.relative_to(ROOT)))
    return sorted(set(names))


def guarded(name):
    return not any(part in SKIP_PARTS for part in Path(name).parts)


def audit(action, reason):
    AUDIT.parent.mkdir(parents=True, exist_ok=True)
    row = dict(time=datetime.datetime.now().isoformat(timespec='seconds'),
               action=action, reason=reason, user=os.environ.get('USER', '?'))
    with AUDIT.open('a') as handle:
        handle.write(json.dumps(row, ensure_ascii=False) + '\n')


def lock(reason=''):
    files = [ROOT / n for n in tracked_files() if guarded(n)]
    changed = 0
    for path in files:
        if not path.is_file():
            continue
        mode = path.stat().st_mode
        if mode & stat.S_IWUSR:
            path.chmod(mode & ~(stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH))
            changed += 1
    MARKER.write_text(json.dumps(dict(locked=True,
                                      time=datetime.datetime.now().isoformat(timespec='seconds'),
                                      files=len(files), reason=reason), ensure_ascii=False) + '\n')
    audit('lock', reason)
    print('仓库已锁定：%d 个文件设为只读（本次改动 %d 个）' % (len(files), changed))
    print('任何脚本写入这些文件都会失败；解锁必须显式执行：')
    print('  ./scripts/repo_guard.py unlock --reason "原因" --confirm UNLOCK')


def unlock(reason, confirm):
    if confirm != 'UNLOCK':
        raise SystemExit('拒绝解锁：必须提供 --confirm UNLOCK')
    if not reason.strip():
        raise SystemExit('拒绝解锁：必须用 --reason 说明原因')
    print('=' * 72)
    print('警告：你正在解除 Demo 仓库的写保护。')
    print('原因：%s' % reason)
    print('解除后任何脚本或程序都可以直接修改本仓库，请确认这是你的显式授权。')
    print('=' * 72)
    files = [ROOT / n for n in tracked_files() if guarded(n)]
    for path in files:
        if not path.is_file():
            continue
        mode = path.stat().st_mode
        if not mode & stat.S_IWUSR:
            path.chmod(mode | stat.S_IWUSR | stat.S_IRUSR)
    if MARKER.exists():
        MARKER.unlink()
    audit('unlock', reason)
    print('已解锁 %d 个文件。修改完成后请重新执行 lock。' % len(files))


def status():
    files = [ROOT / n for n in tracked_files() if guarded(n)]
    writable = [p for p in files if p.is_file() and p.stat().st_mode & stat.S_IWUSR]
    locked = MARKER.exists() and not writable
    print(json.dumps(dict(locked=locked, marker=MARKER.exists(),
                          files=len(files), writable=len(writable)), ensure_ascii=False))
    return 0 if locked else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('status', 'lock', 'unlock'))
    parser.add_argument('--reason', default='')
    parser.add_argument('--confirm', default='')
    args = parser.parse_args()
    if args.action == 'lock':
        lock(args.reason)
    elif args.action == 'unlock':
        unlock(args.reason, args.confirm)
    else:
        raise SystemExit(status())


if __name__ == '__main__':
    main()
