"""crypto_ecc -- ECC confidentiality and origin for everything the robot sends.

WHAT "CRYPTAGE ECC" ACTUALLY MEANS

Elliptic-curve cryptography does not encrypt bulk data directly. There is no
"encrypt this JSON with an EC public key" primitive the way there is with
RSA, and a project that claims one is either using RSA and calling it ECC or
has invented something. What ECC gives you is a *shared secret* between two
key holders, and the standard construction built on that is ECIES:

    1. the sender makes a throw-away ("ephemeral") EC keypair;
    2. ECDH between that private key and the recipient's public key yields a
       shared secret only those two can compute;
    3. HKDF stretches it into a symmetric key;
    4. AES-256-GCM encrypts the payload with it and authenticates it;
    5. the sender ships its ephemeral PUBLIC key alongside the ciphertext.

The recipient repeats step 2 with its own private key and the ephemeral
public key, and gets the same secret. The ephemeral key is discarded, so
recovering the robot's long-term key later still does not decrypt yesterday's
traffic -- that property is called forward secrecy and it is the reason the
key is ephemeral rather than static.

CONFIDENTIALITY IS NOT AUTHENTICITY

ECIES tells the Cloud that whoever sent this knew its public key. The public
key is public: *anyone* can send it a well-formed measurement. So the robot
also SIGNS the payload with a long-term ECDSA key whose public half the
Cloud holds, and the Cloud refuses anything it cannot verify.

Skipping this is the classic hole in a student IoT stack: the data is
unreadable in transit and completely forgeable at the same time. On a farm
that means an attacker cannot read your soil data but can tell you the
irrigation is fine.

CURVE AND SIZES

SECP256R1 (NIST P-256): 128-bit security, in every library and every HSM,
and the curve a jury will recognise. Keys are exchanged as PEM so they can
be read, diffed and pasted into a report.
"""

from __future__ import annotations

import base64
import json
from dataclasses import dataclass
from typing import Any

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec, utils as asym_utils
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives.kdf.hkdf import HKDF

CURVE = ec.SECP256R1()
KEY_BYTES = 32                 # AES-256
NONCE_BYTES = 12               # GCM standard nonce
#: Mixed into HKDF. Binds a ciphertext to THIS protocol and version: a
#: payload captured from another system that happens to share our keys will
#: not decrypt here, and a future v2 with different framing cannot be fed to
#: a v1 reader.
HKDF_INFO = b"agri-cloud/v1/measurement"


class CryptoError(Exception):
    """Anything that went wrong opening a sealed payload. Deliberately does
    NOT say which part failed -- a decryption oracle that distinguishes 'bad
    tag' from 'bad key' is a way in. The Cloud logs the station and the
    sender, which is what an operator actually needs."""


# --------------------------------------------------------------------- keys
@dataclass(frozen=True)
class KeyPair:
    private_pem: bytes
    public_pem: bytes

    def private(self):
        return serialization.load_pem_private_key(self.private_pem, None)

    def public(self):
        return serialization.load_pem_public_key(self.public_pem)


def generate_keypair() -> KeyPair:
    """A fresh P-256 keypair, PEM-encoded, unencrypted.

    Unencrypted because these are development keys generated into a working
    directory; a deployment would wrap the private half with a passphrase or
    keep it in a token. The README says so, and says it in the place someone
    would look before deploying.
    """
    priv = ec.generate_private_key(CURVE)
    return KeyPair(
        private_pem=priv.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption()),
        public_pem=priv.public_key().public_bytes(
            serialization.Encoding.PEM,
            serialization.PublicFormat.SubjectPublicKeyInfo))


def load_public(pem: bytes | str):
    if isinstance(pem, str):
        pem = pem.encode()
    key = serialization.load_pem_public_key(pem)
    if not isinstance(key, ec.EllipticCurvePublicKey):
        raise CryptoError("that PEM is not an EC public key")
    return key


def load_private(pem: bytes | str):
    if isinstance(pem, str):
        pem = pem.encode()
    key = serialization.load_pem_private_key(pem, None)
    if not isinstance(key, ec.EllipticCurvePrivateKey):
        raise CryptoError("that PEM is not an EC private key")
    return key


def _b64(raw: bytes) -> str:
    return base64.b64encode(raw).decode()


def _unb64(text: str) -> bytes:
    try:
        return base64.b64decode(text, validate=True)
    except Exception as exc:                       # noqa: BLE001
        raise CryptoError("field is not valid base64") from exc


def _derive(shared: bytes, salt: bytes) -> bytes:
    return HKDF(algorithm=hashes.SHA256(), length=KEY_BYTES,
                salt=salt, info=HKDF_INFO).derive(shared)


# ------------------------------------------------------------------ seal
def seal(plaintext: bytes, recipient_public_pem: bytes | str,
         signer_private_pem: bytes | str | None = None) -> dict[str, Any]:
    """Encrypt `plaintext` to the recipient, optionally signing it.

    Returns a JSON-safe dict -- this is what travels over MQTT. Everything
    in it is public: the ephemeral key, the nonce and the ciphertext reveal
    nothing without the recipient's private key.
    """
    recipient = load_public(recipient_public_pem)
    ephemeral = ec.generate_private_key(CURVE)
    shared = ephemeral.exchange(ec.ECDH(), recipient)

    eph_pem = ephemeral.public_key().public_bytes(
        serialization.Encoding.PEM,
        serialization.PublicFormat.SubjectPublicKeyInfo)
    # Salt the KDF with the ephemeral public key. Two payloads sealed to the
    # same recipient then never share an AES key even if the RNG repeats a
    # nonce, which is the failure mode GCM does not survive.
    key = _derive(shared, salt=eph_pem)

    nonce = AESGCM.generate_key(bit_length=128)[:NONCE_BYTES]
    ciphertext = AESGCM(key).encrypt(nonce, plaintext, None)

    out: dict[str, Any] = {
        "alg": "ECIES-P256-HKDF-SHA256-AES256GCM",
        "epk": eph_pem.decode(),
        "nonce": _b64(nonce),
        "ciphertext": _b64(ciphertext),
    }
    if signer_private_pem is not None:
        signer = load_private(signer_private_pem)
        # Sign the CIPHERTEXT, not the plaintext: the Cloud can then reject a
        # forgery before doing any private-key work at all, and the signature
        # itself leaks nothing about the contents.
        sig = signer.sign(_unb64(out["ciphertext"]),
                          ec.ECDSA(hashes.SHA256()))
        out["sig"] = _b64(sig)
        out["sig_alg"] = "ECDSA-P256-SHA256"
    return out


def unseal(envelope: dict[str, Any], recipient_private_pem: bytes | str,
           sender_public_pem: bytes | str | None = None) -> bytes:
    """Verify (if a sender key is given) and decrypt. Raises CryptoError.

    Passing `sender_public_pem` makes the signature MANDATORY: an envelope
    that arrives without one is refused rather than quietly accepted, so
    stripping the signature is not a way to bypass the check.
    """
    for field in ("epk", "nonce", "ciphertext"):
        if field not in envelope:
            raise CryptoError(f"envelope has no {field!r}")

    ciphertext = _unb64(envelope["ciphertext"])

    if sender_public_pem is not None:
        if "sig" not in envelope:
            raise CryptoError("a signature was required and none was sent")
        try:
            load_public(sender_public_pem).verify(
                _unb64(envelope["sig"]), ciphertext,
                ec.ECDSA(hashes.SHA256()))
        except InvalidSignature as exc:
            raise CryptoError("signature does not verify") from exc

    try:
        eph_pem = envelope["epk"].encode()
        shared = load_private(recipient_private_pem).exchange(
            ec.ECDH(), load_public(eph_pem))
        key = _derive(shared, salt=eph_pem)
        return AESGCM(key).decrypt(_unb64(envelope["nonce"]), ciphertext, None)
    except CryptoError:
        raise
    except Exception as exc:                       # noqa: BLE001
        raise CryptoError("payload could not be opened") from exc


# ------------------------------------------------------- JSON convenience
def seal_json(obj: Any, recipient_public_pem, signer_private_pem=None) -> dict:
    """Seal a JSON-serialisable object. `sort_keys` so the same measurement
    always produces the same plaintext, which makes the ciphertexts of two
    runs comparable when something looks wrong."""
    raw = json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()
    return seal(raw, recipient_public_pem, signer_private_pem)


def unseal_json(envelope: dict, recipient_private_pem, sender_public_pem=None):
    return json.loads(unseal(envelope, recipient_private_pem,
                             sender_public_pem).decode())


# ------------------------------------------------- signature without secrecy
#
# Not everything needs encrypting, and encrypting everything is a habit worth
# resisting because it hides which property actually matters.
#
# A Cloud->robot REQUEST says "measure P2,5R". There is nothing confidential
# in it -- an eavesdropper who learns the robot is about to visit plant 5
# learns nothing of value. But if anyone can ISSUE one, anyone can drive the
# robot around the greenhouse. So requests are signed and not encrypted, and
# the robot refuses an unsigned one.
#
# Reports need both: the readings are the asset, and a forged report is worse
# than a read one.

def sign_json(obj: Any, signer_private_pem) -> dict[str, Any]:
    """{payload, sig} -- readable by anyone, forgeable by nobody."""
    raw = json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()
    sig = load_private(signer_private_pem).sign(raw, ec.ECDSA(hashes.SHA256()))
    return {"payload": obj, "sig": _b64(sig), "sig_alg": "ECDSA-P256-SHA256"}


def verify_json(signed: dict[str, Any], signer_public_pem) -> Any:
    """Return the payload, or raise. Never returns unverified content."""
    if not isinstance(signed, dict) or "payload" not in signed:
        raise CryptoError("not a signed message")
    if "sig" not in signed:
        raise CryptoError("message carries no signature")
    raw = json.dumps(signed["payload"], sort_keys=True,
                     separators=(",", ":")).encode()
    try:
        load_public(signer_public_pem).verify(
            _unb64(signed["sig"]), raw, ec.ECDSA(hashes.SHA256()))
    except InvalidSignature as exc:
        raise CryptoError("signature does not verify") from exc
    return signed["payload"]
