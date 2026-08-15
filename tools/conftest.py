# Presence of this file puts tools/ on sys.path for the test session (pytest imports
# conftest.py from its own directory before collection), so tools/tests/*.py can
# `import gen_params` without tools/ being an installed/packaged distribution.
