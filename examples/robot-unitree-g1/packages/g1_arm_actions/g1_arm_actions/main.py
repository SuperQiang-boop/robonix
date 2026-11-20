#!/usr/bin/env python3
"""Robonix skill that invokes the G1 arm action primitive."""

from __future__ import annotations

import logging
import os

import grpc
import g1_arm_pb2
import robonix_contracts_pb2_grpc as contracts_grpc
from g1_arm_actions_mcp import (
    RunG1ArmAction_Request,
    RunG1ArmAction_Response,
)
from robonix_api import ATLAS, Err, Ok, Skill
from robonix_api.atlas_types import Transport


ALLOWED_ACTIONS = {
    # Standard actions from SDK2
    "release_arm",
    "turn_back_wave",
    "blow_kiss_with_both_hands",
    "blow_kiss_with_left_hand",
    "blow_kiss_with_right_hand",
    "both_hands_up",
    "clamp",
    "high_five",
    "hug",
    "make_heart_with_both_hands",
    "make_heart_with_right_hand",
    "refuse",
    "right_hand_up",
    "ultraman_ray",
    "wave_under_head",
    "wave_above_head",
    "shake_hand",
    "box_left_hand_win",
    "box_right_hand_win",
    "box_both_hand_win",
    "right_hand_on_heart",
    "both_hands_up_deviate_right",
    "forward_push",
    # Custom actions
    "Waist_Drum_Dance",
    "Spin_discs",
    "Scratch_head",
    "Throw_money",
}
ARM_ACTION_CONTRACT = "robonix/primitive/arm/action"

logging.basicConfig(
    level=os.environ.get("G1_ARM_ACTIONS_LOG_LEVEL", "INFO").upper(),
    format="[g1_arm_actions] %(levelname)s %(message)s",
)
log = logging.getLogger("g1_arm_actions")

skill = Skill(id="g1_arm_actions", namespace="robonix/skill/g1_arm_actions")

_arm_provider_id = "g1_arm"
_rpc_timeout_s = 15.0


def _response(action: str, accepted: bool, detail: str) -> RunG1ArmAction_Response:
    """Build the MCP response with the normalized action echoed back."""
    return RunG1ArmAction_Response(
        accepted=accepted,
        detail=detail,
        action=action,
    )


def _call_arm_primitive(action: str) -> RunG1ArmAction_Response:
    """Resolve the arm primitive through Atlas and call its gRPC action RPC."""
    try:
        caps = ATLAS.find_capability(
            provider_id=_arm_provider_id,
            contract_id=ARM_ACTION_CONTRACT,
            transport=Transport.GRPC,
        )
    except Exception as exc:  # noqa: BLE001
        return _response(action, False, f"arm primitive discovery failed: {exc}")

    if len(caps) != 1:
        return _response(
            action,
            False,
            f"expected one {_arm_provider_id!r} provider for {ARM_ACTION_CONTRACT}, found {len(caps)}",
        )

    try:
        connection = skill.connect_capability(
            caps[0],
            ARM_ACTION_CONTRACT,
            Transport.GRPC,
        )
    except Exception as exc:  # noqa: BLE001
        return _response(action, False, f"arm primitive connection failed: {exc}")

    endpoint = (connection.endpoint or "").strip()
    if not endpoint:
        connection.close()
        return _response(action, False, "arm primitive endpoint is empty")

    channel = grpc.insecure_channel(endpoint)
    try:
        stub = contracts_grpc.RobonixPrimitiveArmActionStub(channel)
        result = stub.RunArmAction(
            g1_arm_pb2.RunArmAction_Request(action=action),
            timeout=_rpc_timeout_s,
        )
    except grpc.RpcError as exc:
        return _response(action, False, f"arm primitive RPC failed: {exc}")
    finally:
        channel.close()
        connection.close()

    return _response(
        str(result.action or action),
        bool(result.accepted),
        str(result.detail),
    )


@skill.on_init
def init(config: dict):
    """Read skill config; dependency resolution is delayed until activation/call."""
    global _arm_provider_id, _rpc_timeout_s
    _arm_provider_id = str(config.get("arm_provider_id", "g1_arm")).strip()
    _rpc_timeout_s = float(config.get("rpc_timeout_s", 15.0))
    if not _arm_provider_id:
        return Err("arm_provider_id must not be empty")
    return Ok()


@skill.on_activate
def activate():
    """Mark the skill active without acquiring SDK2 resources."""
    return Ok()


@skill.on_deactivate
def deactivate():
    """Release no resources; the skill holds only per-call gRPC channels."""
    return Ok()


@skill.mcp("robonix/skill/g1_arm_actions/run")
def run(req: RunG1ArmAction_Request) -> RunG1ArmAction_Response:
    """Run one allowlisted G1 upper-arm preset action."""
    action = str(req.action or "").strip()
    if action not in ALLOWED_ACTIONS:
        allowed = ", ".join(sorted(ALLOWED_ACTIONS))
        return _response(action, False, f"unsupported action; allowed: {allowed}")
    return _call_arm_primitive(action)


if __name__ == "__main__":
    skill.run()
