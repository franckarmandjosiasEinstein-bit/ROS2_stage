"""trust -- naming a key, retiring a key, and noticing an old one.

THE GAP THIS FILLS, AND THE PART OF IT THAT STAYS OPEN

The system runs on two long-lived keypairs generated once and never
touched again. That works until one of two things happens:

    the wrong key    somebody copies the wrong .pem, or copies it to the
                     wrong machine, and every message is refused with a
                     signature error that says nothing about which key was
                     actually used
    a lost key       a robot is stolen, decommissioned, or handed to a
                     contractor, and its key stays as valid as it was on
                     the first day, forever, with no way to say otherwise

Three things here, and they are worth very different amounts:

    fingerprints   REAL. A key gets a short name. Both programs print the
                   counterparty's at startup, so "is this the right key"
                   becomes a thing two people can check over the phone in
                   ten seconds instead of an afternoon of signature errors.

    revocation     REAL, within one trust boundary. The Cloud keeps a list
                   and refuses anything signed by a key on it. The Cloud
                   owns that file, so from the Cloud's side this genuinely
                   retires a key. What it is NOT is a PKI: there is no
                   distribution, so revoking a key on the Cloud does not
                   tell the robot anything, and each side's list has to be
                   copied to the other by hand.

    age            A REMINDER, and no more than that. A raw PEM public key
                   carries no validity period -- that is exactly what an
                   X.509 certificate adds and this is not one. The creation
                   date lives in a sidecar file that nobody signs, so the
                   holder of a key can edit their own. It is here so that a
                   key quietly in use for three years gets mentioned at
                   every startup, which is the whole of what an unsigned
                   date can honestly do.

Saying which is which matters more than the code. A revocation list
described as a PKI would be worse than none, because somebody would then
stop worrying about the thing it does not do.
"""

from __future__ import annotations

import hashlib
import json
from datetime import datetime, timedelta, timezone
from pathlib import Path

from agri.crypto_ecc import load_public

#: How long a key may be in service before both programs start saying so at
#: startup. A year: long enough not to be noise on a project that runs for
#: months, short enough that a deployment left alone gets told.
MAX_AGE_DAYS = 365

REVOKED_FILE = "revoked.txt"
META_SUFFIX = "_public.meta.json"


class RevocationError(Exception):
    """A key that verifies perfectly and must not be trusted anyway."""


# ---------------------------------------------------------- fingerprints
def fingerprint(pem: bytes | str) -> str:
    """A short, stable, readable name for a public key.

    SHA-256 over the DER SubjectPublicKeyInfo -- not over the PEM text, so
    a file that gained a trailing newline or was re-wrapped still has the
    same fingerprint. The alternative would produce two names for one key
    and send somebody hunting for a substitution that never happened.

    Sixty-four bits, in four groups. Long enough that two keys will not
    collide; short enough to read aloud, which is the entire point.
    """
    from cryptography.hazmat.primitives import serialization  # noqa: PLC0415

    der = load_public(pem).public_bytes(
        encoding=serialization.Encoding.DER,
        format=serialization.PublicFormat.SubjectPublicKeyInfo)
    h = hashlib.sha256(der).hexdigest()[:16]
    return ":".join(h[i:i + 4] for i in range(0, 16, 4))


# ------------------------------------------------------------- key age
def meta_path(role: str, directory: Path) -> Path:
    return Path(directory) / f"{role}{META_SUFFIX}"


def record_birth(role: str, pem: bytes, directory: Path) -> Path:
    """Note when a key was generated, and under what name.

    Written beside the key at generation. NOT signed, and that is why age
    below is a reminder rather than a check -- see the module docstring.
    """
    p = meta_path(role, directory)
    p.write_text(json.dumps({
        "role": role,
        "fingerprint": fingerprint(pem),
        "created": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "note": "unsigned: this is a reminder, not a certificate",
    }, indent=2) + "\n")
    return p


def age_days(role: str, directory: Path) -> float | None:
    """How long this key has been in service, or None if nobody wrote it down.

    None for every key generated before this file existed, and that is
    deliberately not an error: a working deployment must not start refusing
    to run because a bookkeeping file is missing.
    """
    p = meta_path(role, directory)
    if not p.exists():
        return None
    try:
        born = datetime.strptime(json.loads(p.read_text())["created"],
                                 "%Y-%m-%dT%H:%M:%SZ").replace(
                                     tzinfo=timezone.utc)
    except Exception:                                # noqa: BLE001
        return None
    return (datetime.now(timezone.utc) - born).total_seconds() / 86400.0


def age_warning(role: str, directory: Path,
                max_age_days: int = MAX_AGE_DAYS) -> str:
    """One line if the key is old enough to be worth rotating, else ''."""
    days = age_days(role, directory)
    if days is None or days < max_age_days:
        return ""
    return (f"{role} key has been in service {days:.0f} days "
            f"(over {max_age_days}). Rotate it:\n"
            f"    python3 -m agri.keys --rotate {role} --dir {directory}\n"
            f"    then copy {role}_public.pem to the other machine.")


# -------------------------------------------------------------- the list
class TrustStore:
    """The keys this machine refuses, whatever their signatures say.

    A flat text file, one fingerprint per line, reason after a #. Chosen
    over anything structured because the operation it has to support is
    "an engineer adds a line at 2 a.m. and restarts the Cloud", and every
    format that needs a tool to edit fails that.
    """

    def __init__(self, directory: Path) -> None:
        self.directory = Path(directory)
        self.path = self.directory / REVOKED_FILE

    def entries(self) -> dict[str, str]:
        """{fingerprint: reason}. Empty when there is no list, which is the
        normal state and not a problem worth reporting."""
        out: dict[str, str] = {}
        if not self.path.exists():
            return out
        for line in self.path.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fp, _, reason = line.partition("#")
            fp = fp.strip()
            if fp:
                out[fp] = reason.strip() or "no reason recorded"
        return out

    def reason(self, pem: bytes | str) -> str | None:
        """Why this key is refused, or None."""
        return self.entries().get(fingerprint(pem))

    def check(self, pem: bytes | str, who: str) -> None:
        """Raise if this key has been retired. Called at startup, once.

        Once, and not per message, on purpose. There is one counterparty
        key per program here, so re-reading the file for every report would
        be forty-eight stat() calls a campaign to answer a question whose
        answer cannot change without somebody editing a file on this disk.
        `--revoke` says to restart, and that is the honest contract rather
        than a hot-reload that works most of the time.
        """
        why = self.reason(pem)
        if why is None:
            return
        raise RevocationError(
            f"the {who} key {fingerprint(pem)} is REVOKED: {why}\n"
            f"    Listed in {self.path}\n"
            f"    Refusing to run. A revoked key that still works is a\n"
            f"    revocation list that means nothing. Install the "
            f"replacement\n"
            f"    key, or remove the line if it was added in error.")

    def revoke(self, pem: bytes | str, reason: str) -> str:
        """Add a key to the list. Returns its fingerprint."""
        fp = fingerprint(pem)
        if fp in self.entries():
            return fp
        self.directory.mkdir(parents=True, exist_ok=True)
        new = not self.path.exists()
        with self.path.open("a") as fh:
            if new:
                fh.write(
                    "# Keys this machine refuses, whatever their signatures\n"
                    "# say. One fingerprint per line, reason after a #.\n"
                    "#\n"
                    "# This list is LOCAL. Nothing distributes it: revoking\n"
                    "# a key here does not tell the other machine anything,\n"
                    "# so copy this file across when you add a line.\n"
                    "#\n"
                    "# Takes effect at startup. Restart the program.\n")
            stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d")
            fh.write(f"{fp}  # {reason} ({stamp})\n")
        return fp


def summarise(role: str, pem: bytes, directory: Path) -> str:
    """The line each program prints at startup about a key it trusts.

    Printed rather than merely checked, because the failure this catches is
    two machines holding two different keys and neither saying which -- and
    a fingerprint on both screens settles that in ten seconds.
    """
    days = age_days(role, directory)
    age = f", {days:.0f} days old" if days is not None else ""
    return f"{role} key {fingerprint(pem)}{age}"


def expiry_hint(days: float | None) -> str:
    """When this key is due for rotation, in words."""
    if days is None:
        return "age unknown (generated before this was recorded)"
    left = MAX_AGE_DAYS - days
    if left <= 0:
        return f"overdue for rotation by {-left:.0f} days"
    return (f"due for rotation "
            f"{(datetime.now(timezone.utc) + timedelta(days=left)):%Y-%m-%d}")
