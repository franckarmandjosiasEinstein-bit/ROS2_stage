"""keys -- generate and load the two keypairs the system runs on.

WHO HOLDS WHAT

    the Cloud  its own PRIVATE key      (opens every report)
               the robot's PUBLIC key   (verifies who sent it)

    the robot  its own PRIVATE key      (signs every report)
               the Cloud's PUBLIC key   (seals reports to it)

Neither side ever needs the other's private half. That is the whole point of
using asymmetric keys here rather than a shared password: capturing the robot
gives an attacker the ability to sign as that robot, and nothing else -- not
the ability to read other robots' traffic, and not the ability to read its
own past traffic, because the session keys were ephemeral.

WHERE THEY LIVE

`keys/` beside the Cloud, one PEM per role, generated on first use and never
overwritten. They are NOT in git: a repository with a private key in it is a
private key that has to be considered public forever, and a student project
that ships one teaches the wrong reflex. .gitignore enforces it and the test
suite checks that it does.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from agri.crypto_ecc import KeyPair, generate_keypair

DEFAULT_DIR = Path("keys")
ROLES = ("cloud", "robot")


def paths(role: str, directory: Path = DEFAULT_DIR) -> tuple[Path, Path]:
    if role not in ROLES:
        raise ValueError(f"role must be one of {ROLES}, not {role!r}")
    return directory / f"{role}_private.pem", directory / f"{role}_public.pem"


def ensure(role: str, directory: Path = DEFAULT_DIR,
           generate: bool = True) -> KeyPair:
    """Load the role's keypair, generating it if it is not there yet.

    Never regenerates over an existing key. A fresh Cloud key would make
    every stored report undecryptable and every robot unable to seal to it,
    and doing that silently on a restart is not a failure anyone enjoys
    diagnosing.

    `generate=False` refuses to create anything and says what to copy
    instead. Both running programs use it, through bootstrap() below; only
    the command line ever generates.
    """
    priv_path, pub_path = paths(role, directory)
    if priv_path.exists() and pub_path.exists():
        return KeyPair(priv_path.read_bytes(), pub_path.read_bytes())
    if priv_path.exists() != pub_path.exists():
        raise FileNotFoundError(
            f"{role}: found one half of the keypair and not the other. "
            f"Delete {priv_path.parent}/ and regenerate, or restore the "
            "missing file -- do not let it generate a mismatched pair.")
    if not generate:
        raise FileNotFoundError(_missing(role, directory))
    directory.mkdir(parents=True, exist_ok=True)
    kp = generate_keypair()
    priv_path.write_bytes(kp.private_pem)
    pub_path.write_bytes(kp.public_pem)
    priv_path.chmod(0o600)
    return kp


def public_of(role: str, directory: Path = DEFAULT_DIR,
              generate: bool = True) -> bytes:
    """The role's public key.

    With generate=False this is how you ask for the OTHER side's key, and it
    is the only safe way to ask for it when the two sides are on two
    machines: it reads the file or explains itself, and never invents one.
    """
    _, pub_path = paths(role, directory)
    if pub_path.exists():
        return pub_path.read_bytes()
    if not generate:
        raise FileNotFoundError(_missing(role, directory))
    return ensure(role, directory).public_pem


def bootstrap(directory: Path = DEFAULT_DIR) -> bool:
    """Generate BOTH keypairs, and only into a directory that holds none.

    THE TWO-MACHINE TRAP, WHICH THIS EXISTS TO CLOSE.

    On one machine the Cloud and the robot share keys/, so whichever starts
    first generated both pairs and everything matched. Split them across two
    machines and each side, finding no key for the OTHER role, quietly
    generated one -- a private half included. Nothing failed loudly: the
    Cloud signed requests the robot could not verify, so the robot refused
    every order; the robot sealed reports the Cloud could not open, so the
    Cloud rejected every one. Two silent halves of one forgotten `scp`, and
    both symptoms read as a network fault.

    So: an EMPTY directory means a fresh single-machine install, and both
    pairs are generated together, which is the only way they can match. A
    directory with anything already in it is a deployment somebody set up on
    purpose, and a key missing from it is a mistake, not an invitation.

    Returns True if it generated anything.
    """
    if any(p.exists() for role in ROLES for p in paths(role, directory)):
        return False
    for role in ROLES:
        ensure(role, directory)
    return True


def _missing(role: str, directory: Path) -> str:
    priv, pub = paths(role, directory)
    return (
        f"{pub} is missing.\n"
        f"\n"
        f"Nothing will be generated for you here. A key invented on this\n"
        f"machine cannot match the one the other side is using, and the\n"
        f"result is not an error but silence: every request refused, every\n"
        f"report rejected, and nothing anywhere saying why.\n"
        f"\n"
        f"ON ONE MACHINE -- generate both pairs into an empty directory:\n"
        f"    python3 -m agri.keys --dir {directory}\n"
        f"\n"
        f"ON TWO MACHINES -- copy the PUBLIC half across (never the private\n"
        f"one), from the machine that holds {priv.name}:\n"
        f"    scp {pub.name} <this-machine>:{directory}/")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", type=Path, default=DEFAULT_DIR)
    args = ap.parse_args()
    for role in ROLES:
        kp = ensure(role, args.dir)
        priv, pub = paths(role, args.dir)
        print(f"{role:6s} private {priv}  public {pub}")
        print(f"       {kp.public_pem.decode().splitlines()[1][:48]}...")
    print(f"\nkeys are in {args.dir}/ and are NOT tracked by git.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
