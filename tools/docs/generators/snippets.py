"""``<!-- generated: quickstart-code -->``: the README example, copied from ``tests/docs/quickstart.cpp``.

The site includes the file directly; the README cannot, so it carries this copy and ``hardfoc-docs gen --check``
fails when the two differ.
"""

from __future__ import annotations

import re

from hardfoc_mkdocs_theme.paths import repo_root


def _section(path: str, name: str) -> str:
    text = (repo_root() / path).read_text()
    m = re.search(rf"--8<-- \[start:{name}\]\n(.*?)\n[^\n]*--8<-- \[end:{name}\]", text, re.S)
    if not m:
        raise SystemExit(f"[gen] no section {name!r} in {path}")
    return m.group(1)


def quickstart_code() -> str:
    return "```cpp\n" + _section("tests/docs/quickstart.cpp", "quickstart") + "\n```\n"


REGIONS = {"quickstart-code": quickstart_code}
