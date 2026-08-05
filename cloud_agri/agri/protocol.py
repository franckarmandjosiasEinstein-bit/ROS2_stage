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

    agri/v1/request        Cloud -> robot   signed, not encrypted
    agri/v1/ack            robot -> Cloud   plain: progress, no payload
    agri/v1/report         robot -> Cloud   sealed: the measurement
    agri/v1/status         robot -> Cloud   plain, retained: am I alive

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

from agri.labels import LabelError, all_labels, normalise
from agri.measurement import utc_now

SCHEMA = "agri-cloud/v1"

TOPIC_REQUEST = "agri/v1/request"
TOPIC_ACK = "agri/v1/ack"
TOPIC_REPORT = "agri/v1/report"
TOPIC_STATUS = "agri/v1/status"

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
def make_request(request_id: str, targets: str | list[str]) -> dict[str, Any]:
    """A work order. `targets` is ALL, one label, or a list of labels."""
    return {"schema": SCHEMA, "kind": "request", "request_id": request_id,
            "issued_at": utc_now(), "targets": expand_targets(targets)}


def expand_targets(targets: str | list[str]) -> list[str]:
    """ALL / "P2,5R" / ["P2,5R", "p1,1l"] -> a canonical list of labels.

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
            lab = normalise(t)
        except LabelError as exc:
            raise ProtocolError(str(exc)) from exc
        if lab not in seen:        # a repeated station is a typo, not a plan
            seen.add(lab)
            out.append(lab)
    return out


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
             done: int = 0, total: int = 0, detail: str = "") -> dict:
    """Progress. Carries no measurement, so it needs no confidentiality.

    `state` is one of accepted / driving / measuring / sent / finished /
    failed. The Cloud shows it live; without it a whole-greenhouse sweep is
    forty minutes of silence and the operator cannot tell a working robot
    from a dead one.
    """
    return {"schema": SCHEMA, "kind": "ack", "request_id": request_id,
            "robot": robot, "state": state, "label": label,
            "done": done, "total": total, "detail": detail, "at": utc_now()}


# ----------------------------------------------------------------- status
def make_status(robot: str, online: bool, pose=None, note: str = "") -> dict:
    d = {"schema": SCHEMA, "kind": "status", "robot": robot,
         "online": bool(online), "note": note, "at": utc_now()}
    if pose is not None:
        d["pose"] = {"x": round(pose[0], 3), "y": round(pose[1], 3),
                     "yaw_deg": round(pose[2], 1)}
    return d
