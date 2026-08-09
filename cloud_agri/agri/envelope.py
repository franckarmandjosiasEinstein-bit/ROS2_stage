"""envelope -- the JSON the robot sends, and what the Cloud checks about it.

THE SHAPE

Two layers, and the distinction matters.

    the REPORT is the payload: station, readings, QR image, photograph.
      It is what the Cloud stores. It is sealed and nobody but the Cloud
      can read it.

    the ENVELOPE is what travels: the sealed report plus the little that
      has to stay readable for the message to be routed and verified at
      all -- who sent it, which request it answers, when.

Anything in the envelope is public. So the sender id appears in BOTH places:
outside so the Cloud knows whose signing key to check, and inside where it
is covered by that signature. The Cloud compares them and rejects a mismatch.
Without the inner copy, "who sent this" would be an unauthenticated string
that anybody could set, and the signature check would be verifying a message
against a key chosen by the attacker.

WHY THE QR TRAVELS WITH THE NUMBERS IT ENCODES

It looks redundant -- the readings are already in the report as JSON. It is
not, and it is the cheapest integrity check in the system: the Cloud decodes
the QR and compares it with the numbers beside it. They agree only if the
same measurement produced both. A robot with a corrupted buffer, a truncated
QR, or two measurements crossed between threads produces a mismatch, and the
Cloud says which station and how they differed rather than filing a number
that is quietly wrong.

The brief asks for the QR and the photo in a JSON file. This is that file.
"""

from __future__ import annotations

import base64
import io
import uuid
from typing import Any

from agri.catalogue import nearest_station, station
from agri.crypto_ecc import seal_json, unseal_json
from agri.measurement import Measurement, utc_now
from agri.qrcodec import QRError, decode_b64, encode_b64

SCHEMA = "agri-cloud/v1"

#: How far from the painted cross the robot may be and still be considered
#: parked ON it. Two stations sharing an inner aisle are 0.10 m apart, so
#: anything looser than half of that would let a visit be filed under its
#: neighbour's label; 0.04 m keeps a clear gap.
PARK_TOLERANCE = 0.04


class EnvelopeError(Exception):
    pass


# --------------------------------------------------------------- photo
def encode_photo(jpeg: bytes, width: int, height: int) -> dict[str, Any]:
    return {"format": "jpeg", "width": int(width), "height": int(height),
            "bytes": len(jpeg), "data_b64": base64.b64encode(jpeg).decode()}


def decode_photo(d: dict[str, Any]) -> bytes:
    try:
        return base64.b64decode(d["data_b64"], validate=True)
    except Exception as exc:                        # noqa: BLE001
        raise EnvelopeError("photo is not valid base64") from exc


def synthetic_photo(label: str, width: int = 320, height: int = 240) -> bytes:
    """A stand-in photograph, for running the Cloud without a simulator.

    Deliberately looks nothing like a real plant: a flat card with the label
    written across it. A placeholder that resembled the real thing would be
    mistaken for one in a screenshot, and the report would claim a camera it
    never used.
    """
    from PIL import Image, ImageDraw                 # noqa: PLC0415

    img = Image.new("RGB", (width, height), (24, 32, 28))
    d = ImageDraw.Draw(img)
    d.rectangle([8, 8, width - 8, height - 8], outline=(0, 200, 140), width=2)
    d.text((22, height // 2 - 16), f"{label}", fill=(0, 220, 150))
    d.text((22, height // 2 + 4), "SYNTHETIC - no camera", fill=(120, 140, 130))
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=80)
    return buf.getvalue()


# --------------------------------------------------------------- report
def build_report(m: Measurement, photo_jpeg: bytes, photo_size: tuple[int, int],
                 robot_id: str, request_id: str | None = None) -> dict[str, Any]:
    """The plaintext report for one visit."""
    s = station(m.label)
    park_error = None
    if m.pose is not None:
        park_error = round(((m.pose[0] - s.x) ** 2
                            + (m.pose[1] - s.y) ** 2) ** 0.5, 4)
    return {
        "schema": SCHEMA,
        "kind": "measurement",
        "robot": robot_id,
        "request_id": request_id,
        "sent_at": utc_now(),
        # "plant" is the plant NUMBER, beside "row"; "plant_at" is where it
        # stands. Both were called "plant" -- one dict literal with the same
        # key twice, which Python accepts in silence and resolves last-wins.
        # Every report ever filed therefore carried a coordinate pair where
        # the plant index belonged, next to a row index that was a plain
        # integer. Nothing downstream read it, so nothing ever broke; it was
        # simply wrong in the file, which is the kind of wrong only found by
        # reading the file.
        "station": {
            "label": s.label, "row": s.row, "plant": s.plant, "side": s.side,
            "cross": {"x": s.x, "y": s.y},
            "plant_at": {"x": s.plant_x, "y": s.plant_y},
        },
        "measurement": m.to_dict(),
        "qr_png_b64": encode_b64(m.to_qr_text()),
        "photo": encode_photo(photo_jpeg, *photo_size),
        "parking_error_m": park_error,
    }


def seal_report(report: dict[str, Any], cloud_public_pem,
                robot_private_pem) -> dict[str, Any]:
    """Report -> the envelope that goes on the wire."""
    return {
        "schema": SCHEMA,
        "kind": "sealed",
        "robot": report["robot"],          # routing only; verified after open
        "request_id": report.get("request_id"),
        "sent_at": report["sent_at"],
        "sealed": seal_json(report, cloud_public_pem, robot_private_pem),
    }


def open_envelope(envelope: dict[str, Any], cloud_private_pem,
                  robot_public_pem) -> dict[str, Any]:
    """Verify, decrypt and CHECK. Raises EnvelopeError with a usable reason.

    The checks are the point. Decrypting is not validating: a report that
    opens cleanly can still name a station that does not exist, carry a QR
    that disagrees with its own numbers, or have been parked a metre from
    the cross it claims. Each of those is caught here, once, so no consumer
    downstream has to remember to.
    """
    if "sealed" not in envelope:
        raise EnvelopeError("envelope carries nothing sealed")

    report = unseal_json(envelope["sealed"], cloud_private_pem,
                         robot_public_pem)

    if report.get("schema") != SCHEMA:
        raise EnvelopeError(
            f"report schema {report.get('schema')!r}, expected {SCHEMA!r}")

    # The sender named outside must be the one named under the signature.
    if envelope.get("robot") != report.get("robot"):
        raise EnvelopeError(
            f"sender says {envelope.get('robot')!r} outside and "
            f"{report.get('robot')!r} inside")

    m = Measurement.from_dict(report["measurement"])
    s = station(m.label)                              # raises on a bad label

    # The QR must say the same thing as the numbers it travels with.
    try:
        qr_text = decode_b64(report["qr_png_b64"])
    except (KeyError, QRError) as exc:
        raise EnvelopeError(f"{m.label}: QR image unreadable ({exc})") from exc
    if qr_text != m.to_qr_text():
        raise EnvelopeError(
            f"{m.label}: the QR code and the readings disagree.\n"
            f"    QR   {qr_text}\n"
            f"    JSON {m.to_qr_text()}")

    # Did it actually park on the cross it claims?
    if m.pose is not None:
        near, dist = nearest_station(m.pose[0], m.pose[1])
        if near.label != s.label:
            raise EnvelopeError(
                f"filed as {s.label} but the pose {m.pose[:2]} is closest to "
                f"{near.label} ({dist:.3f} m) -- the robot measured the wrong "
                "plant or the label is wrong")

    return report


def new_request_id() -> str:
    return uuid.uuid4().hex[:12]
