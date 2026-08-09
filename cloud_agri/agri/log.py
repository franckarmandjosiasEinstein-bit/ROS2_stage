"""log -- coloured, structured terminal output for all three processes.

Three terminals, three voices, one palette:

    CLOUD   magenta prefix, because the dashboard already paints the Cloud
            that colour in its dialogue panel
    ROBOT   cyan prefix, same reason
    BROKER  yellow, the colour mosquitto itself uses for warnings

Within each, the verbs use a second colour:

    action verbs (request, report, connect)  bright white / bold
    data   (a label, a value, a path)        green
    error / refused                          red
    dim context (a hash, a timestamp)        grey
    ok / accepted / verified                 green

The rule is the same one the dashboard uses: colour means something, and
the same something everywhere.
"""

from __future__ import annotations

import os
import sys

_NO_COLOUR = (not hasattr(sys.stdout, "isatty")
              or not sys.stdout.isatty()
              or os.environ.get("NO_COLOR"))

# ANSI SGR sequences.
_RST = "\033[0m"
_BOLD = "\033[1m"
_DIM = "\033[2m"
_RED = "\033[31m"
_GREEN = "\033[32m"
_YELLOW = "\033[33m"
_MAGENTA = "\033[35m"
_CYAN = "\033[36m"
_WHITE = "\033[97m"
_GREY = "\033[90m"
_BG_RED = "\033[41m"
_BG_GREEN = "\033[42m"
_BG_CYAN = "\033[46m"
_BG_MAG = "\033[45m"


def _c(code: str, text: str) -> str:
    if _NO_COLOUR:
        return text
    return f"{code}{text}{_RST}"


def bold(t: str) -> str:
    return _c(_BOLD, t)


def dim(t: str) -> str:
    return _c(_DIM + _GREY, t)


def ok(t: str) -> str:
    return _c(_GREEN, t)


def err(t: str) -> str:
    return _c(_RED, t)


def warn(t: str) -> str:
    return _c(_YELLOW, t)


def data(t: str) -> str:
    return _c(_GREEN, t)


def label(t: str) -> str:
    return _c(_BOLD + _WHITE, t)


# ------------------------------------------------------------ prefixes

def _prefix(bg: str, fg: str, tag: str) -> str:
    if _NO_COLOUR:
        return f"{tag}:"
    return f"{bg}{fg}{_BOLD} {tag} {_RST}"


CLOUD = _prefix(_BG_MAG, _WHITE, "CLOUD")
ROBOT = _prefix(_BG_CYAN, _WHITE, "ROBOT")
BROKER = _prefix("", _YELLOW + _BOLD, "MQTT")


def cloud(*parts: str) -> None:
    print(CLOUD, *parts)


def robot(*parts: str) -> None:
    print(ROBOT, *parts)


def broker(*parts: str) -> None:
    print(BROKER, *parts)


# ------------------------------------------------------- banner / box

def banner(role: str, lines: list[str]) -> None:
    if _NO_COLOUR:
        for ln in lines:
            print(f"{role}: {ln}")
        return
    w = max((len(ln) for ln in lines), default=20)
    bar = "─" * (w + 2)
    fg = _MAGENTA if role == "cloud" else _CYAN if role == "robot" else _YELLOW
    print(f"{fg}┌{bar}┐{_RST}")
    for ln in lines:
        print(f"{fg}│{_RST} {ln:<{w}} {fg}│{_RST}")
    print(f"{fg}└{bar}┘{_RST}")


# ------------------------------------------------ arrow for dialogue

def arrow_out(text: str) -> str:
    return f" {_c(_MAGENTA, '→')} {text}" if not _NO_COLOUR else f" -> {text}"


def arrow_in(text: str) -> str:
    return f" {_c(_CYAN, '←')} {text}" if not _NO_COLOUR else f" <- {text}"


# ------------------------------------------------ table row

def kv(key: str, value: str) -> str:
    return f"  {dim(key + ':')} {value}"
