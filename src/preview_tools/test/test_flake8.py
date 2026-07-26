from ament_flake8.main import main_with_errors


def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, 'Found %d code style errors:\n' % len(errors) + \
        '\n'.join(errors)
