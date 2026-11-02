# Testing and CI

How the headers and this site are checked on every pull request.

## Compile check

[`tests/docs/quickstart.cpp`](../tests/docs/quickstart.cpp) holds every code block on this site. Compile it
against each board before changing a header:

```bash
for b in VORTEX_V1 FLUX_V1; do
  c++ -std=c++20 -fsyntax-only -Wall -Wextra -Isrc -DHARDFOC_BOARD_$b tests/docs/quickstart.cpp
done
```

## Generated tables

The pin maps and the board table are regions filled by
[`tools/docs/generators/pins.py`](../tools/docs/generators/pins.py) from the X-macro rows; the README example is
copied from the compiled file by [`tools/docs/generators/snippets.py`](../tools/docs/generators/snippets.py).
`hardfoc-docs check` fails when a header changed and the page did not:

```bash
pip install "git+https://github.com/hardfoc/hf-general-ci-tools@v1#subdirectory=packages/hardfoc-mkdocs-theme"
hardfoc-docs gen          # refresh after editing a header
hardfoc-docs check        # what CI runs
mkdocs serve              # preview at http://127.0.0.1:8000
```

## CI workflows

| Workflow | What it checks |
| --- | --- |
| Docs | Generated-table and figure drift, strict site build, Doxygen, offline link check, Lighthouse, screenshots; publishes on `main` and tags |
| C++ lint | clang-format and clang-tidy on the headers |
| C++ analysis | cppcheck on `src/` |
| Markdown lint, YAML lint, link check | Repository text files |
| Release | GitHub release on a `v*` tag |

The workflows call the shared, versioned workflows of
[hf-general-ci-tools](https://github.com/hardfoc/hf-general-ci-tools) at `@v1`.
