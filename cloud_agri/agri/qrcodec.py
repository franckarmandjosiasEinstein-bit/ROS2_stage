"""qrcodec -- the numeric reading becomes a QR image, and comes back out.

WHY THERE IS A DECODER HERE AT ALL

The brief only asks for the data to be turned INTO a QR code. Writing just
the encoder is the tempting half: it produces a PNG, the PNG looks like a QR
code, and nothing ever checks that it says what it is supposed to say. Two
ways that fails silently are easy to hit -- a payload that overflows the
chosen version and gets truncated, and an error-correction level so low that
the robot's own camera cannot read its own code back.

So the encoder has a decoder beside it, the regression test round-trips
every one of the 48 stations through a real PNG, and the Cloud verifies the
code it receives against the numbers it receives. A QR that does not decode
to the measurement it travels with is a corrupted measurement, and the Cloud
says so instead of filing it.

CHOICES

    ERROR_CORRECTION = M   ~15 % recoverable. L is tempting for density but
                           leaves nothing for a photograph taken at an angle
                           under greenhouse glare.
    box_size = 6           enough pixels per module for cv2 to resolve it
                           after JPEG, without making the PNG large.
    version  = None        let the library pick the smallest that fits, and
                           raise if the payload is too big rather than
                           silently choosing a bigger one than declared.
"""

from __future__ import annotations

import base64
import io

import numpy as np
import qrcode
from PIL import Image

ERROR_CORRECTION = qrcode.constants.ERROR_CORRECT_M
BOX_SIZE = 6
BORDER = 4          # the "quiet zone"; 4 modules is the spec minimum


class QRError(Exception):
    pass


def encode_png(text: str) -> bytes:
    """QR image for `text`, as PNG bytes."""
    if not text:
        raise QRError("refusing to encode an empty payload")
    qr = qrcode.QRCode(version=None, error_correction=ERROR_CORRECTION,
                       box_size=BOX_SIZE, border=BORDER)
    qr.add_data(text)
    try:
        qr.make(fit=True)
    except qrcode.exceptions.DataOverflowError as exc:
        raise QRError(
            f"payload of {len(text)} chars does not fit in a QR code at "
            f"error-correction M") from exc
    buf = io.BytesIO()
    qr.make_image(fill_color="black", back_color="white").save(buf, "PNG")
    return buf.getvalue()


def _decode_zxing(img: Image.Image) -> str | None:
    try:
        import zxingcpp                              # noqa: PLC0415
    except ImportError:                              # pragma: no cover
        return None
    res = zxingcpp.read_barcode(img)
    return res.text if res is not None else None


def _decode_cv2(img: Image.Image) -> str | None:
    try:
        import cv2                                   # noqa: PLC0415
    except ImportError:                              # pragma: no cover
        return None
    text, _, _ = cv2.QRCodeDetector().detectAndDecode(np.array(img))
    return text or None


def decode_png(png: bytes) -> str:
    """Read a QR image back. Raises QRError when nothing decodes.

    TWO DECODERS, AND WHY THAT IS NOT BELT-AND-BRACES

    The first version used OpenCV alone, and the 48-station round-trip test
    caught it failing on exactly two codes -- P1,1R and P2,1L. Not
    mis-reading them: `detect()` could not even find the finder patterns.
    zxing-cpp reads all 48 without complaint, which says the codes are valid
    and OpenCV's detector is the weak link.

    That matters because the Cloud verifies every QR it receives against the
    numbers travelling beside it. A decoder that fails on 4 % of well-formed
    codes would have the Cloud rejecting good measurements and blaming the
    robot. So zxing-cpp is tried first and OpenCV is the fallback, and
    whichever answers first wins.

    Both are imported lazily. The robot only ever ENCODES; making a ROS 2
    node fail to start because a decoder is missing would be a self-inflicted
    outage on the one machine that does not need it.
    """
    img = Image.open(io.BytesIO(png)).convert("L")
    for engine in (_decode_zxing, _decode_cv2):
        text = engine(img)
        if text:
            return text
    raise QRError(
        "no QR code could be read from that image (tried zxing-cpp and "
        "OpenCV; install at least one: pip install zxing-cpp)")


def encode_b64(text: str) -> str:
    """PNG, base64'd, ready to drop into JSON."""
    return base64.b64encode(encode_png(text)).decode()


def decode_b64(b64: str) -> str:
    try:
        png = base64.b64decode(b64, validate=True)
    except Exception as exc:                          # noqa: BLE001
        raise QRError("QR field is not valid base64") from exc
    return decode_png(png)
