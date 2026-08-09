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
from datetime import datetime, timezone
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
    # Note the birthday beside the key. Unsigned, so it is a reminder and
    # not a validity period -- agri/trust.py says so at length.
    from agri.trust import record_birth                   # noqa: PLC0415
    record_birth(role, kp.public_pem, directory)
    return kp


def rotate(role: str, directory: Path = DEFAULT_DIR,
           reason: str = "rotated") -> tuple[str, str]:
    """Retire this role's keypair and generate a new one.

    Returns (old fingerprint, new fingerprint).

    THE ORDER OF OPERATIONS IS THE WHOLE FUNCTION.

    Archive the old pair before touching anything: a rotation that
    overwrites the only copy of a private key makes every report already
    sealed to it permanently unopenable, and there is no recovering from
    that. Then revoke the old public key BEFORE writing the new one, so a
    crash halfway through leaves a machine that refuses the retired key
    rather than one that still accepts it.

    What this cannot do is tell the other machine. Copy the new
    <role>_public.pem across, and copy revoked.txt with it.
    """
    from agri.trust import TrustStore, fingerprint        # noqa: PLC0415

    priv_path, pub_path = paths(role, directory)
    if not (priv_path.exists() and pub_path.exists()):
        raise FileNotFoundError(
            f"{role}: nothing to rotate -- {pub_path} does not exist.")

    old_public = pub_path.read_bytes()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    archive = directory / "retired" / stamp
    archive.mkdir(parents=True, exist_ok=True)
    for p in (priv_path, pub_path):
        (archive / p.name).write_bytes(p.read_bytes())
    (archive / priv_path.name).chmod(0o600)

    old_fp = TrustStore(directory).revoke(old_public, reason)

    priv_path.unlink()
    pub_path.unlink()
    kp = ensure(role, directory)
    return old_fp, fingerprint(kp.public_pem)


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


def main(argv: list[str] | None = None) -> int:
    from agri.trust import (TrustStore, age_days,         # noqa: PLC0415
                            expiry_hint, fingerprint)

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", type=Path, default=DEFAULT_DIR)
    ap.add_argument("--rotate", choices=ROLES, metavar="ROLE",
                    help="retire this role's keypair and generate a new one. "
                         "The old pair is archived under retired/ and its "
                         "public key is added to revoked.txt.")
    ap.add_argument("--revoke", choices=ROLES, metavar="ROLE",
                    help="refuse this role's CURRENT key from now on, "
                         "without generating a replacement")
    ap.add_argument("--reason", default="",
                    help="why, recorded beside the fingerprint")
    ap.add_argument("--list", action="store_true",
                    help="show the fingerprints and ages of the keys here")
    args = ap.parse_args(argv)

    store = TrustStore(args.dir)

    if args.revoke:
        _, pub = paths(args.revoke, args.dir)
        if not pub.exists():
            print(f"{pub} does not exist -- nothing to revoke.")
            return 1
        fp = store.revoke(pub.read_bytes(),
                          args.reason or "revoked by hand")
        print(f"{args.revoke} key {fp} is now refused.\n"
              f"  listed in {store.path}\n"
              f"  RESTART the program for this to take effect, and copy "
              f"that file\n  to the other machine -- nothing distributes it.")
        return 0

    if args.rotate:
        old, new = rotate(args.rotate, args.dir,
                          args.reason or "rotated")
        print(f"{args.rotate}: {old} retired -> {new} in service\n"
              f"  the old pair is under {args.dir / 'retired'}/ "
              f"(do not delete it: reports\n"
              f"  already sealed to it can only be opened with it)\n"
              f"  copy {paths(args.rotate, args.dir)[1].name} and "
              f"{store.path.name} to the other machine")
        return 0

    if args.list:
        for role in ROLES:
            _, pub = paths(role, args.dir)
            if not pub.exists():
                print(f"{role:6s} not present")
                continue
            pem = pub.read_bytes()
            print(f"{role:6s} {fingerprint(pem)}  "
                  f"{expiry_hint(age_days(role, args.dir))}")
        revoked = store.entries()
        print(f"\n{len(revoked)} revoked key(s)"
              + (f" in {store.path}" if revoked else ""))
        for fp, why in revoked.items():
            print(f"  {fp}  {why}")
        return 0

    for role in ROLES:
        kp = ensure(role, args.dir)
        priv, pub = paths(role, args.dir)
        print(f"{role:6s} private {priv}  public {pub}")
        print(f"       {fingerprint(kp.public_pem)}  "
              f"{expiry_hint(age_days(role, args.dir))}")
    print(f"\nkeys are in {args.dir}/ and are NOT tracked by git.")
    print("Compare the fingerprints on both machines. Two sides holding two\n"
          "different keys is the failure that otherwise shows up only as\n"
          "every message being refused, with nothing saying which key.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
