"""Which code a run used: git short hash, plus a marker when the source tree has uncommitted changes.

Kept free of ROS imports so it can be unit tested.
"""
import hashlib
import subprocess


def code_state(ws_dir, paths=('src',)):
    """'<hash>' for a clean tree, '<hash>+dirty:<8 chars of a hash of the uncommitted diff>' when tracked files under
    `paths` differ from HEAD, None when ws_dir is not a git checkout. Untracked files are not counted."""
    def git(*args):
        return subprocess.run(['git', '-C', ws_dir, *args], capture_output=True, text=True, timeout=10).stdout

    try:
        head = git('rev-parse', '--short', 'HEAD').strip()
        if not head:
            return None
        if not git('status', '--porcelain', '--untracked-files=no', '--', *paths).strip():
            return head
        diff = git('diff', 'HEAD', '--', *paths)
        return f"{head}+dirty:{hashlib.sha1(diff.encode()).hexdigest()[:8]}"
    except Exception:
        return None


def is_dirty(code):
    return bool(code) and '+dirty' in code
