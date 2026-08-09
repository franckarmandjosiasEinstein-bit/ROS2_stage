"""protocol -- the topics and message shapes the robot and Cloud share.

TRANSPORT

MQTT, because the Cloud has to be able to reach the robot and not only the
other way round. With HTTP the robot would have to poll for work; with MQTT
it subscribes once and the broker pushes. It is also the protocol the rest of
the agricultural-IoT world speaks, so the same robot could report to a
different back end without changing a line here.

The broker is the only thing both sides need to agree on. Neither has to know
the other's address, and either can restart without the other noticing --
which is the property that makes "the robot is a node" true rather than
decorative.

TOPICS

    agri/v1/request              Cloud -> nodes   signed, not encrypted
    agri/v1/query/<node_id>      Cloud -> node    signed: a question, a grant
    agri/v1/reply/<node_id>      node  -> Cloud   signed: an answer, an offer
    agri/v1/ack/<node_id>        node  -> Cloud   plain: progress, no payload
    agri/v1/report/<node_id>     node  -> Cloud   sealed: the measurement
    agri/v1/status/<node_id>     node  -> Cloud   plain, retained: am I alive

Everything that can CHANGE WHAT THE OTHER SIDE DOES is signed; everything
that only reports is not. An ack and a status are observations -- forging
one lies to a dashboard, which is bad and is not the same as steering a
robot. A query, a reply and a request all decide something, so all three
carry a signature.

Each node publishes on its own subtopic; the Cloud subscribes with an MQTT
wildcard (agri/v1/status/+) to hear every one. Requests stay on a flat topic
because a request addresses the fleet, not a specific node.

NODE KINDS

Every status message carries a `node_kind` field: `"mobile"` for a robot
that drives between stations, `"fixed"` for an ESP microcontroller bolted
to a post. The Cloud uses this to implement a priority rule: when a mobile
node is parked at a station, its reading is preferred over the fixed node's.
Today there is one mobile and zero fixed; the field is carried now so that
adding fixed nodes later does not change the protocol.

WHY THE REQUEST IS SIGNED AND NOT ENCRYPTED

Habit says encrypt everything. Resisting it is what makes the design legible:
a request says "measure P2,5R", which is not a secret, but issuing one drives
a robot around a greenhouse. So the property that matters is authenticity,
not confidentiality. Requests are signed; the robot refuses an unsigned one
and refuses one signed by the wrong key. Reports need both and get both.

WHY STATUS IS RETAINED AND NOTHING ELSE IS

A dashboard opened at any moment should immediately know whether the robot
is there. MQTT's retained flag gives exactly that: the broker keeps the last
status and hands it to every new subscriber. Retaining reports instead would
mean every reconnecting client re-receives the last measurement and files it
again, which is how duplicates appear in a store that has no reason to have
any.
"""

from __future__ import annotations

from typing import Any

from agri.labels import (LabelError, all_labels, expand_target,
                         normalise)
from agri.measurement import utc_now

SCHEMA = "agri-cloud/v1"

# Cloud -> all nodes: flat, because a request addresses the fleet.
TOPIC_REQUEST = "agri/v1/request"

# Node -> Cloud: namespaced by node_id. The Cloud subscribes with +.
TOPIC_STATUS_ALL = "agri/v1/status/+"
TOPIC_ACK_ALL = "agri/v1/ack/+"
TOPIC_REPORT_ALL = "agri/v1/report/+"

KIND_MOBILE = "mobile"
KIND_FIXED = "fixed"

# ---------------------------------------------------------------- modes
#: HOW THE CLOUD DRIVES THE FLEET. Two modes, and the difference is who
#: starts the conversation.
#:
#:   command    the operator asks for stations and the Cloud relays the
#:              order. What Phase C did from the beginning: the robot is
#:              driven, and the human decides when.
#:   collector  the Cloud runs the campaign itself. It polls the node,
#:              waits until it is genuinely stopped, issues the order, and
#:              negotiates the handover of each reading. Nobody types
#:              anything after the first command.
#:
#: The modes share every message below. Collector mode uses more of them,
#: and that is the whole difference -- there is no second protocol, and no
#: message means something different depending on the mode.
MODE_COMMAND = "command"
MODE_COLLECTOR = "collector"
MODES = (MODE_COMMAND, MODE_COLLECTOR)

# ------------------------------------------------------------- handshake
#: THE TWO TOPICS THAT MAKE THE EXCHANGE A CONVERSATION.
#:
#: Everything before these was one-directional: the Cloud published orders
#: and the node published results, and neither ever waited for the other.
#: That is enough to collect data and not enough to show HOW the two
#: cooperate, which is what a demonstration of a connected node is for.
#:
#: query  Cloud -> one node.  A question, or a permission.
#: reply  node  -> Cloud.     An answer, or a request for permission.
#:
#: Namespaced per node like every other node-directed topic, so a second
#: robot needs no protocol change.
TOPIC_QUERY_ALL = "agri/v1/query/+"
TOPIC_REPLY_ALL = "agri/v1/reply/+"

#: The steps of the exchange, spelled out so a log can be read as dialogue
#: rather than decoded. Each is one side saying one thing.
STEP_IDLE_ASK = "idle?"      # Cloud: are you stopped?
STEP_IDLE = "idle"           # node:  yes / no, and here is my speed
STEP_OFFER = "offer"         # node:  parked, reading ready, may I send?
STEP_SEND = "send"           # Cloud: yes, send it
STEP_HOLD = "hold"           # Cloud: not now, keep it
STEP_FILED = "filed"         # Cloud: received, opened, verified, stored


def topic_query(node_id: str) -> str:
    return f"agri/v1/query/{node_id}"


def topic_reply(node_id: str) -> str:
    return f"agri/v1/reply/{node_id}"


def make_query(step: str, node: str, **extra) -> dict:
    """Cloud -> node. The BODY of a query; sign it before it goes out."""
    return {"schema": SCHEMA, "kind": "query", "step": step,
            "node": node, "at": utc_now(), **extra}


def make_reply(step: str, node: str, **extra) -> dict:
    """node -> Cloud. The BODY of a reply; sign it before it goes out."""
    return {"schema": SCHEMA, "kind": "reply", "step": step,
            "node": node, "at": utc_now(), **extra}


# ------------------------------------------------- signing the handshake
# WHY THE HANDSHAKE IS SIGNED TOO, AFTER ALL.
#
# The first version left queries and replies plain, with a reasoning that
# looked sound: a query carries no secret, and a forged `send` only makes
# the robot offer a reading it was going to offer anyway, to a Cloud that
# cannot open it without the right key. Confidentiality was genuinely not
# needed, and it still is not -- these messages stay in the clear.
#
# What that reasoning missed is the OTHER grant. `hold` means "keep it, I
# am not ready", and the robot obeys it by putting the reading in the
# outbox and carrying on. Anyone who could publish to agri/v1/query/<node>
# could answer `hold` to every offer. The robot would measure the whole
# greenhouse, fill its 48-slot outbox, start dropping the oldest readings,
# and report itself perfectly healthy throughout. No error anywhere,
# nothing rejected, no measurement corrupted -- just a campaign that
# quietly produces nothing. That is a denial of service, and it costs one
# MQTT publish.
#
# The same holds in the other direction: a forged `idle` reply tells the
# Cloud that a moving robot is stopped, and collector mode issues an order
# into a robot mid-drive.
#
# So both directions are signed, each with the key that side already holds
# and the other side already trusts. Nothing new to distribute.
def open_handshake(payload: bytes | str | dict, signer_public_pem,
                   kind: str, window_s: float | None = None) -> dict:
    """Verify a query or a reply and return its body, or raise ProtocolError.

    Freshness is checked but NOT uniqueness, and the asymmetry is
    deliberate. A repeated `send` is harmless -- the robot sends a reading
    it already offered. A repeated `hold` is equally harmless: the outbox
    is where the reading already is. These messages are idempotent, so the
    seen-set that requests need would cost memory and buy nothing here.

    A STALE one is a different matter, which is why the window applies: a
    `hold` captured today and replayed during next week's campaign is the
    denial of service above, arriving late.
    """
    import json as _json                                  # noqa: PLC0415
    from datetime import datetime, timezone               # noqa: PLC0415

    from agri.crypto_ecc import CryptoError, verify_json  # noqa: PLC0415
    from agri.replay import (FRESHNESS_S, ReplayError,    # noqa: PLC0415
                             parse_stamp)

    if isinstance(payload, (bytes, str)):
        try:
            signed = _json.loads(payload)
        except Exception as exc:                     # noqa: BLE001
            raise ProtocolError(f"{kind} is not JSON ({exc})") from exc
    else:
        signed = payload

    try:
        body = verify_json(signed, signer_public_pem)
    except CryptoError as exc:
        raise ProtocolError(
            f"{kind} is unsigned or forged ({exc}). Anyone who can reach "
            f"the broker can publish on this topic; only the holder of the "
            f"key can sign.") from exc

    if not isinstance(body, dict):
        raise ProtocolError(f"{kind} body is not an object")
    if body.get("schema") != SCHEMA:
        raise ProtocolError(f"{kind} schema {body.get('schema')!r}")
    if body.get("kind") != kind:
        raise ProtocolError(f"expected a {kind}, got {body.get('kind')!r}")

    window = FRESHNESS_S if window_s is None else window_s
    try:
        age = (datetime.now(timezone.utc)
               - parse_stamp(body.get("at", ""))).total_seconds()
    except ReplayError as exc:
        raise ProtocolError(f"{kind}: {exc}") from exc
    if abs(age) > window:
        raise ProtocolError(
            f"{kind} '{body.get('step')}' is {age:+.0f} s out of date and "
            f"the window is {window:.0f} s -- REFUSED. Either the clocks "
            f"disagree (timedatectl status) or this message was captured "
            f"and replayed.")
    return body


def topic_status(node_id: str) -> str:
    return f"agri/v1/status/{node_id}"


def topic_ack(node_id: str) -> str:
    return f"agri/v1/ack/{node_id}"


def topic_report(node_id: str) -> str:
    return f"agri/v1/report/{node_id}"


def node_id_from_topic(topic: str) -> str | None:
    """Extract the node_id from a namespaced topic, or None."""
    parts = topic.split("/")
    return parts[3] if len(parts) == 4 and parts[:2] == ["agri", "v1"] else None


#: "measure everything", spelled one way so both sides recognise it.
ALL = "ALL"

DEFAULT_BROKER = "localhost"
DEFAULT_PORT = 1883
#: At-least-once. Zero would drop a request when the robot is mid-reconnect;
#: two costs a four-way handshake for no benefit here, because the request id
#: already makes a repeat harmless.
QOS = 1


class ProtocolError(ValueError):
    pass


def mqtt_client(client_id: str):
    """A paho client, built the way THIS paho wants to be built.

    paho-mqtt 2.0 added a mandatory first argument, `callback_api_version`,
    and changed the signature of on_connect along with it. 2.1 still defaults
    to the old behaviour but prints a DeprecationWarning every time, and a
    later release will not default at all. Both sides of this system create a
    client, so the version dance happens once, here, rather than twice and
    differently.

    VERSION1 is chosen deliberately over VERSION2: under V2, on_connect gets
    a ReasonCode object instead of an integer, and the callbacks in server.py
    and robot_node.py test it with `if rc:`. Moving to V2 means changing that
    test in both places at the same moment -- a change worth making on
    purpose, not as a side effect of somebody's pip upgrade.
    """
    import warnings                                    # noqa: PLC0415

    import paho.mqtt.client as mqtt                    # noqa: PLC0415

    api = getattr(mqtt, "CallbackAPIVersion", None)
    if api is None:                                    # paho 1.x
        return mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        return mqtt.Client(api.VERSION1, client_id=client_id,
                           protocol=mqtt.MQTTv311)


# ---------------------------------------------------------------- request
def make_request(request_id: str, targets: str | list[str],
                 mode: str = MODE_COMMAND) -> dict[str, Any]:
    """A work order. `targets` is ALL, one label, or a list of labels.

    `mode` travels INSIDE the signed payload, deliberately. It decides
    whether the robot hands each reading over on request or negotiates the
    handover first, and that is a behavioural instruction like the target
    list -- so it gets the same protection. A mode carried outside the
    signature could be flipped by anyone who can reach the broker.
    """
    return {"schema": SCHEMA, "kind": "request", "request_id": request_id,
            "issued_at": utc_now(), "mode": mode,
            "targets": expand_targets(targets)}


def expand_targets(targets: str | list[str]) -> list[str]:
    """What the Cloud was asked for -> the exact stations to visit.

        ALL         every station, in survey order
        "P2,5"      a PLANT: both of its stations, R then L
        "P2,5R"     one station
        [...]       any mixture of the above

    Resolved on the CLOUD side, before the request is signed, so the robot
    receives an explicit list and never has to interpret a wildcard. A robot
    that expanded ALL itself would silently disagree with the Cloud the day
    the greenhouse gains a row, and the Cloud would wait forever for a
    station the robot does not believe exists.
    """
    if isinstance(targets, str):
        if targets.strip().upper() == ALL:
            return all_labels()
        targets = [targets]
    if not targets:
        raise ProtocolError("a request with no target does nothing")
    out, seen = [], set()
    for t in targets:
        try:
            labels = expand_target(t)
        except LabelError as exc:
            raise ProtocolError(str(exc)) from exc
        for lab in labels:
            if lab not in seen:    # a repeated station is a typo, not a plan
                seen.add(lab)
                out.append(lab)
    return out


def request_mode(payload: dict[str, Any]) -> str:
    """The mode a VERIFIED request asks for, defaulting to command.

    Unknown modes fall back to command rather than raising: a robot that
    refused an order because a newer Cloud named a mode it had not heard of
    would stop working on an upgrade, and command is the mode that needs
    the least from the other side.
    """
    mode = payload.get("mode", MODE_COMMAND)
    return mode if mode in MODES else MODE_COMMAND


def check_request(payload: dict[str, Any]) -> tuple[str, list[str]]:
    """(request_id, targets) from a VERIFIED request payload, or raise."""
    if payload.get("schema") != SCHEMA:
        raise ProtocolError(f"request schema {payload.get('schema')!r}")
    if payload.get("kind") != "request":
        raise ProtocolError(f"not a request: {payload.get('kind')!r}")
    rid = payload.get("request_id")
    if not isinstance(rid, str) or not rid:
        raise ProtocolError("request has no id")
    targets = payload.get("targets")
    if not isinstance(targets, list) or not targets:
        raise ProtocolError("request has no targets")
    return rid, [normalise(t) for t in targets]


# -------------------------------------------------------------------- ack
def make_ack(request_id: str, robot: str, state: str, label: str | None = None,
             done: int = 0, total: int = 0, detail: str = "",
             metres: float | None = None,
             planned_m: float | None = None) -> dict:
    """Progress. Carries no measurement, so it needs no confidentiality.

    `state` is one of accepted / driving / measuring / sent / finished /
    failed. The Cloud shows it live; without it a whole-greenhouse sweep is
    forty minutes of silence and the operator cannot tell a working robot
    from a dead one.

    `metres` is how far the base actually drove, integrated from odometry,
    and `planned_m` is what the chosen order was expected to cost. Both are
    sent only on the closing ack, and only when the driver counts them --
    None means "this robot does not say", which is different from zero and
    must not be exported as zero. They are here rather than in the report
    because they belong to the MISSION: a report is about one station, and
    the distance is the property of the whole trip.
    """
    d = {"schema": SCHEMA, "kind": "ack", "request_id": request_id,
         "robot": robot, "state": state, "label": label,
         "done": done, "total": total, "detail": detail, "at": utc_now()}
    if metres is not None:
        d["metres"] = round(float(metres), 2)
    if planned_m is not None:
        d["planned_m"] = round(float(planned_m), 2)
    return d


# ----------------------------------------------------------------- status
def make_status(robot: str, online: bool, pose=None, note: str = "",
                velocity=None, node_kind: str = KIND_MOBILE) -> dict:
    """Live telemetry: where the node is and how fast it is going.

    Sent on a timer, not only on arrival, so the Cloud can watch the robot
    cross the greenhouse instead of learning about it forty minutes later.
    Retained by the broker, so a dashboard opened at any moment immediately
    knows whether there is a node at all.

    `node_kind` is KIND_MOBILE for a driving robot, KIND_FIXED for a
    stationary sensor (ESP). The Cloud uses it to prioritise mobile readings
    when both a mobile and a fixed node report for the same station.
    """
    d = {"schema": SCHEMA, "kind": "status", "node_kind": node_kind,
         "robot": robot, "online": bool(online), "note": note,
         "at": utc_now()}
    if pose is not None:
        d["pose"] = {"x": round(pose[0], 3), "y": round(pose[1], 3),
                     "yaw_deg": round(pose[2], 1)}
    if velocity is not None:
        d["velocity"] = {"vx": round(velocity[0], 3),
                         "vy": round(velocity[1], 3),
                         "wz": round(velocity[2], 3),
                         "speed": round((velocity[0] ** 2
                                         + velocity[1] ** 2) ** 0.5, 3)}
    return d
