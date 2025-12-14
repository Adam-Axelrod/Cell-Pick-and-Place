from __future__ import annotations
import os
import json
from typing import Optional, Dict, Any

# Constants
SINC_RATE_FACTOR = 0.00001953125  # sinc_length -> sampling rate factor: rate = 1 / (factor * sinc_length)

PROTOCOL_TO_FILENAME: Dict[str, str] = {
    "CANopen_over_EtherCAT": "ethercat.json",
    "CANopen_over_EtherCAT_gen0": "ethercat_gen0.json",
    "Bota_Binary_gen0": "bota_binary_gen0.json",
    "Bota_Socket": "bota_socket.json",
    "Bota_Binary": "bota_binary.json",
}

# GEN_A thresholds: (upper_bound_exclusive, mapped_rate)
_GEN_A_THRESHOLDS = [
    (15.0, 10.0),
    (22.5, 20.0),
    (37.5, 25.0),
    (75.0, 50.0),
    (150.0, 100.0),
    (225.0, 200.0),
    (260.0, 250.0),
    (335.0, 270.0),
    (450.0, 400.0),
    (650.0, 500.0),
    (900.0, 800.0),
    (1300.0, 1000.0),
    (1865.0, 1600.0),
    (2650.0, 2133.0),
    (3520.0, 3200.0),
    (float("inf"), 3840.0),
]

RATE_TO_SUBMODE: Dict[float, int] = {
    10.0: 0,
    20.0: 1,
    25.0: 2,
    50.0: 3,
    100.0: 4,
    200.0: 5,
    250.0: 6,
    270.0: 7,
    400.0: 8,
    500.0: 9,
    800.0: 10,
    1000.0: 11,
    1600.0: 12,
    2133.0: 13,
    3200.0: 14,
    3840.0: 15,
}


def _resolve_config_path(base_dir: str, protocol: str) -> str:
    """Resolve the JSON config file path for a protocol.

    Raises ValueError for unknown protocols.
    """
    filename = PROTOCOL_TO_FILENAME.get(protocol)
    if not filename:
        raise ValueError(f"Unknown protocol: {protocol}. Valid: {list(PROTOCOL_TO_FILENAME.keys())}")
    return os.path.join(base_dir, "bft_config", filename)


def _read_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _write_json(path: str, obj: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=4)


def sinc_length_from_rate(rate: float) -> int:
    """Convert a desired sampling rate to an integer sinc_length.

    Formula inverted from rate = 1 / (SINC_RATE_FACTOR * sinc_length)
    """
    if rate <= 0:
        raise ValueError("rate must be > 0")
    return int(round(1.0 / (SINC_RATE_FACTOR * float(rate))))


def get_update_rate_gen_a(requested_rate: float) -> float:
    """Map a requested (continuous) rate to the discrete GEN_A supported rate.

    This is data-driven using _GEN_A_THRESHOLDS; it mirrors the original behavior.
    """
    if requested_rate is None:
        raise ValueError("requested_rate must be provided")
    r = float(requested_rate)
    for upper, mapped in _GEN_A_THRESHOLDS:
        if r < upper:
            return mapped
    return _GEN_A_THRESHOLDS[-1][1]


def get_app_submode_from_rate(discrete_rate: float) -> int:
    try:
        return RATE_TO_SUBMODE[float(discrete_rate)]
    except KeyError:
        raise ValueError(f"Invalid update rate: {discrete_rate}. Allowed rates: {sorted(RATE_TO_SUBMODE.keys())}")


def write_config(protocol: str,
                 com_port: Optional[str] = None,
                 update_rate: Optional[float] = None,
                 network_interface: Optional[str] = None) -> str:
    """Update and write the driver JSON config for a given protocol.

    Returns the path to the written config file.
    """
    # Resolve config path
    package_path = os.path.dirname(os.path.dirname(__file__))
    config_path = _resolve_config_path(package_path, protocol)

    cfg = _read_json(config_path)

    # Ensure nested keys exist
    driver_cfg = cfg.setdefault("driver_config", {})
    comm_params = driver_cfg.setdefault("communication_interface_params", {})
    sensor_params = driver_cfg.setdefault("sensor_operation_params", {})

    # Communication interface selection
    if protocol in ("Bota_Binary_gen0", "Bota_Binary"):
        if not com_port:
            raise ValueError(f"com_port is required for protocol {protocol}")
        comm_params["com_port"] = com_port
    else:
        if not network_interface:
            raise ValueError(f"network_interface is required for protocol {protocol}")
        comm_params["network_interface"] = network_interface

    # Update rate handling
    if update_rate is not None:
        r = float(update_rate)
        if protocol in ("CANopen_over_EtherCAT_gen0", "Bota_Binary_gen0"):
            sensor_params["sinc_length"] = sinc_length_from_rate(r)
        else:
            discrete = get_update_rate_gen_a(r)
            sensor_params["app_submode"] = get_app_submode_from_rate(discrete)

    _write_json(config_path, cfg)
    return config_path