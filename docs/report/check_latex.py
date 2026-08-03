#!/usr/bin/env python3
"""Pre-compile sanity checks for the report sources.

There is no LaTeX toolchain on every machine this report is edited from,
and a missing brace only shows up as a wall of errors 200 lines into a
compile. This runs in a tenth of a second with nothing but Python and
catches the three mistakes that actually happen when writing LaTeX by
hand:

    1. an \\input{} pointing at a file that does not exist;
    2. an unbalanced \\begin/\\end pair;
    3. an unbalanced brace.

It does NOT replace compiling -- it just makes the first compile likely
to succeed. Run it before `make`.

    python3 check_latex.py
"""

from __future__ import annotations

import os
import re
import sys

# Directory to check. Defaults to this script's own directory (the main
# report), but takes a path argument so the same checks run over
# docs/status_report/ -- two documents, one checker, no second copy to drift.
ROOT = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 \
    else os.path.dirname(os.path.abspath(__file__))


def strip(text: str) -> str:
    """Remove everything that must not be counted, in the right order.

    Order matters and got this wrong twice while writing it:
      * `\\%` is an escaped percent, NOT a comment -- neutralise it
        first, or the comment stripper eats the rest of the line and
        takes the closing brace of `\\measured{100\\%}` with it;
      * `\\\\` is a line break, NOT an escaped brace -- neutralise it
        before `\\{`, or `\\\\{` is misread as an escaped brace;
      * verbatim/listing bodies are full of unbalanced braces on
        purpose, so they are dropped wholesale.
    """
    text = text.replace(r"\%", " ")          # escaped percent
    text = re.sub(r"(?<!\\)%.*", "", text)   # real comments
    text = text.replace("\\\\", " ")         # line breaks
    text = re.sub(r"\\[{}]", "", text)       # escaped braces
    text = re.sub(r"\\begin\{lstlisting\}.*?\\end\{lstlisting\}",
                  "", text, flags=re.S)      # listing bodies
    text = re.sub(r"\\verb\|[^|]*\|", "", text)
    return text


def tex_files() -> list[str]:
    out = []
    for base, _, files in os.walk(ROOT):
        for fn in sorted(files):
            if fn.endswith(".tex"):
                out.append(os.path.relpath(os.path.join(base, fn), ROOT))
    return sorted(out)


def main() -> int:
    problems = []
    files = tex_files()

    for rel in files:
        raw = open(os.path.join(ROOT, rel)).read()
        text = strip(raw)

        # 1. \input targets ------------------------------------------------
        for m in re.finditer(r"\\input\{([^}]+)\}", text):
            target = m.group(1)
            if not any(os.path.exists(os.path.join(ROOT, target + ext))
                       for ext in (".tex", "")):
                problems.append(f"{rel}: \\input{{{target}}} does not exist")

        # 2. environments --------------------------------------------------
        stack = []
        for m in re.finditer(r"\\(begin|end)\{([^}]+)\}", text):
            kind, name = m.group(1), m.group(2)
            if kind == "begin":
                stack.append((name, raw[:m.start()].count("\n") + 1))
            elif not stack:
                problems.append(f"{rel}: \\end{{{name}}} with no \\begin")
            elif stack[-1][0] != name:
                problems.append(
                    f"{rel}: \\end{{{name}}} closes \\begin{{{stack[-1][0]}}} "
                    f"opened at line {stack[-1][1]}")
                stack.pop()
            else:
                stack.pop()
        for name, line in stack:
            problems.append(f"{rel}: \\begin{{{name}}} at line {line} "
                            "is never closed")

        # 3. braces ---------------------------------------------------------
        delta = text.count("{") - text.count("}")
        if delta:
            sign = "unclosed {" if delta > 0 else "extra }"
            problems.append(f"{rel}: brace imbalance {delta:+d} ({sign})")

    print(f"checked {len(files)} .tex files")
    if problems:
        print(f"\n{len(problems)} problem(s):")
        for p in problems:
            print(f"  - {p}")
        return 1
    print("no missing inputs, no unbalanced environments, no stray braces.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
