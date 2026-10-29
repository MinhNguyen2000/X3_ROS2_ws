import subprocess

from x3_nav_bringup.code_state import code_state, is_dirty


def run(cwd, *args):
    subprocess.run(args, cwd=cwd, check=True, capture_output=True)


def make_repo(tmp_path):
    (tmp_path / 'src').mkdir()
    (tmp_path / 'src' / 'a.py').write_text('x = 1\n')
    (tmp_path / 'notes.txt').write_text('n\n')
    run(tmp_path, 'git', 'init', '-q')
    run(tmp_path, 'git', '-c', 'user.name=t', '-c', 'user.email=t@t', 'add', '-A')
    run(tmp_path, 'git', '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-q', '-m', 'init')
    return str(tmp_path)


def test_clean_tree_is_just_the_hash(tmp_path):
    c = code_state(make_repo(tmp_path))
    assert c and '+' not in c and not is_dirty(c)


def test_changed_tracked_file_under_src_is_dirty_and_the_diff_is_identified(tmp_path):
    ws = make_repo(tmp_path)
    (tmp_path / 'src' / 'a.py').write_text('x = 2\n')
    c1 = code_state(ws)
    assert is_dirty(c1) and '+dirty:' in c1
    assert code_state(ws) == c1                          # same diff, same marker
    (tmp_path / 'src' / 'a.py').write_text('x = 3\n')
    assert code_state(ws) != c1                          # another diff, another marker


def test_untracked_files_and_changes_outside_src_do_not_count(tmp_path):
    ws = make_repo(tmp_path)
    (tmp_path / 'src' / 'new.py').write_text('y = 1\n')
    (tmp_path / 'notes.txt').write_text('changed\n')
    assert not is_dirty(code_state(ws))


def test_not_a_git_checkout(tmp_path):
    assert code_state(str(tmp_path)) is None
    assert not is_dirty(None)
