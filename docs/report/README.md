# PFA technical report — sources

LaTeX sources for the report *An Autonomous Mobile-Manipulation Stack for
Greenhouse Strawberry Harvesting: from a Webots Prototype to a ROS 2 /
Gazebo Digital Twin*.

IEEEtran, two columns, no page limit, English.

## Building

```bash
sudo apt install texlive-latex-extra texlive-publishers texlive-science
cd docs/report
make            # -> report.pdf
```

`make` runs `check_latex.py` first (pure Python, no LaTeX needed), then
`pdflatex → bibtex → pdflatex → pdflatex`, then reports any unresolved
cross-reference or citation. Other targets:

| target | effect |
|---|---|
| `make quick` | one pass, fast, references will read `??` |
| `make check` | list undefined references/citations from the last build |
| `make clean` | remove build artefacts, keep the PDF |
| `make purge` | remove everything generated |

If you prefer Overleaf: upload the whole `docs/report/` directory and set
`report.tex` as the main document. No further configuration is needed.

## Layout

```
report.tex          main document: title, abstract, section order
preamble.tex        packages, colour palette, listing styles,
                    THE FLOWCHART LANGUAGE, SWOT table environments
references.bib      IEEE bibliography (IEEEtran.bst)
check_latex.py      pre-compile sanity checks
sections/           one file per section, numbered in reading order
figures/            TikZ figures, one file per figure
```

## Conventions

**Flowcharts.** Every flowchart uses the fixed visual language defined
in `figures/fig_legend.tex` and rendered as Fig. 1 — six node shapes and
three colour bands (INITIALISATION / MAIN LOOP / TERMINATION). Do not
invent new shapes; add them to `preamble.tex` so the legend stays true.

**Wide floats.** Two columns are narrow. Flowcharts and large tables go
in `figure*` / `table*` so they span both columns. The `swottablewide`
environment is the spanning variant of `swottable`.

**Numbers.** Any quantitative claim must be traceable to a simulation
log. Wrap it in `\measured{...}` so it stands out and can be audited.

**Macros.** Use `\topicname{scan}` for a ROS topic, `\nodename{...}` for
a node, `\filename{...}` for a source file, `\param{...}` for a
parameter. This keeps the typography consistent and makes a global
change possible later.

## Status

| part | state |
|---|---|
| Preamble, flowchart language, SWOT environments | done |
| §I Introduction + Fig. 1 legend | done |
| §III Environment, installation, launch procedures | done |
| Bibliography (33 IEEE entries) | done |
| §II Related work | stub |
| §IV Phase A — Webots (9 subsections) | stub |
| §V Phase B — ROS 2 / Gazebo (9 subsections) | stub |
| §VI–IX SWOT, results, discussion, conclusion | stub |
| Appendices A–D | stub |

Every stub compiles and carries its `\label`, so cross-references
already resolve and the section order can be reviewed before the content
is written.
