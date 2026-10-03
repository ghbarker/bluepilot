"""Exercise the portal's state gates without starting its HTTP server or touching sysfs."""

import ast
import logging
import re
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from urllib.parse import urlparse

import pytest

from bluepilot.backend.network import utils
from bluepilot.backend.params.params_watcher import ParamsWatcher
from bluepilot.backend.utils import power
from bluepilot.backend.utils.params_fallback import is_explicitly_offroad


ROOT = Path(__file__).resolve().parents[3]
PARAM_KEYS = set(re.findall(r'^\s*\{"([^"]+)"', (ROOT / "openpilot/common/params_keys.h").read_text(), re.MULTILINE))


class StrictParams:
    """Use the branch's declared keys with typed and legacy fallback values."""

    def __init__(self, offroad, error=None):
        self.values = {"IsOffroad": offroad}
        self.error = error

    def get(self, key):
        assert key in PARAM_KEYS, f"Undeclared Params key: {key}"
        if self.error:
            raise self.error
        return self.values.get(key)

    def get_bool(self, key):
        return is_explicitly_offroad(self.get(key))


def source_function(relative_path, name, **namespace):
    # These entry points live in modules with process/logging initialization.
    # Execute their unchanged AST bodies with dependencies supplied by the test.
    path = ROOT / relative_path
    tree = ast.parse(path.read_text())
    functions = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == name]
    assert len(functions) == 1
    namespace.setdefault("logger", logging.getLogger(__name__))
    namespace.setdefault("is_explicitly_offroad", is_explicitly_offroad)
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), "exec"), namespace)
    return namespace[name]


@pytest.mark.parametrize("offroad,expected", [
    (True, False), (False, True), (1, True), (0, True),
    (b"1", False), ("1", False), (b"0", True), ("0", True),
    (None, True), (b"", True), ("", True), (b"true", True), (b"1\n", True),
])
def test_only_explicit_offroad_unlocks_portal(monkeypatch, offroad, expected):
    monkeypatch.setattr(utils, "params", StrictParams(offroad))
    assert utils.is_onroad() is expected


def test_params_read_error_keeps_onroad_restrictions(monkeypatch, caplog):
    monkeypatch.setattr(utils, "params", StrictParams(None, OSError("unreadable")))
    assert utils.is_onroad() is True
    assert "assuming onroad" in caplog.text


def test_native_params_roundtrip(monkeypatch, tmp_path):
    # Linux CI builds libparams. Windows source-only runs lack that library.
    if sys.platform == "win32":
        pytest.skip("native Params library requires Linux or macOS")
    from openpilot.common.params import Params

    params = Params(str(tmp_path))
    monkeypatch.setattr(utils, "params", params)
    idle = source_function("bluepilot/backend/routes/preprocessor.py", "is_device_idle", params=params)
    assert params.get("IsOffroad") is None
    assert utils.is_onroad() is True
    assert idle() is False
    for offroad in (True, False, True):
        params.put_bool("IsOffroad", offroad, block=True)
        assert params.get("IsOffroad") is offroad
        assert utils.is_onroad() is not offroad
        assert idle() is offroad
    params.remove("IsOffroad")
    assert utils.is_onroad() is True
    assert idle() is False


@pytest.mark.parametrize("offroad,expected", [(True, True), (False, False), (b"1", True), (b"0", False)])
def test_watcher_publishes_offroad_boolean(offroad, expected):
    watcher = ParamsWatcher(StrictParams(offroad))
    assert watcher._read_param_value("IsOffroad") is expected


@pytest.mark.parametrize("offroad,expected", [(True, True), (False, False), (b"1", True), (b"0", False), (None, False), (b"", False)])
def test_preprocessor_requires_explicit_offroad(offroad, expected):
    idle = source_function("bluepilot/backend/routes/preprocessor.py", "is_device_idle", params=StrictParams(offroad))
    assert idle() is expected


def test_preprocessor_read_error_is_busy():
    idle = source_function("bluepilot/backend/routes/preprocessor.py", "is_device_idle",
                           params=StrictParams(None, OSError("unreadable")))
    assert idle() is False


@pytest.mark.parametrize("awake_key", ["IsDriverViewEnabled", "IsEngaged"])
def test_preprocessor_does_not_run_when_awake(awake_key):
    params = StrictParams(b"1")
    params.values[awake_key] = b"1"
    idle = source_function("bluepilot/backend/routes/preprocessor.py", "is_device_idle", params=params)
    assert idle() is False


@pytest.mark.parametrize("offroad", [b"0", None])
@pytest.mark.parametrize("method,path", [
    ("do_POST", "/api/route-export/example/front/cancel"),
    ("do_DELETE", "/api/delete/example"),
])
def test_route_mutations_remain_blocked_without_offroad(monkeypatch, offroad, method, path):
    monkeypatch.setattr(utils, "params", StrictParams(offroad))
    handler = SimpleNamespace(path=path, _enforce_rate_limit=lambda *a, **kw: True, send_json_response=Mock())
    operation = source_function("bluepilot/backend/bp_portal.py", method,
                                urlparse=urlparse, should_server_run=lambda: True, is_onroad=utils.is_onroad)
    operation(handler)
    response, status = handler.send_json_response.call_args.args
    assert status == 503
    assert response["reason"] == "safety"


def test_params_post_retains_existing_onroad_policy(monkeypatch):
    monkeypatch.setattr(utils, "params", StrictParams(b"0"))
    handler = SimpleNamespace(path="/api/params/set", headers={},
                              _enforce_rate_limit=lambda *a, **kw: True, send_json_response=Mock())
    operation = source_function("bluepilot/backend/bp_portal.py", "do_POST",
                                urlparse=urlparse, should_server_run=lambda: True, is_onroad=utils.is_onroad)
    operation(handler)
    response, status = handler.send_json_response.call_args.args
    # Reaching request validation proves the existing params exception remains.
    assert status == 400
    assert response["error"] == "Missing request body"


def test_status_monitor_supplies_callback_and_still_broadcasts(monkeypatch):
    monkeypatch.setattr(power, "last_activity_time", None)
    onroad = Mock(return_value=True)
    broadcast = Mock()
    monitor = source_function("bluepilot/backend/bp_portal.py", "monitor_status",
                              check_and_restore_power_save=power.check_and_restore_power_save,
                              is_onroad=onroad, last_onroad_status=[False],
                              broadcast_websocket_event=broadcast,
                              WebSocketEvent=SimpleNamespace(STATUS_CHANGED="status_changed"))
    monitor()
    broadcast.assert_called_once_with("status_changed", {"status": "onroad", "onroad": True})


@pytest.fixture
def restored(monkeypatch):
    restore = Mock()
    monkeypatch.setattr(power, "restore_power_save", restore)
    monkeypatch.setattr(power.time, "time", lambda: 1000)
    monkeypatch.setattr(power, "last_activity_time", 1000 - power.IDLE_TIMEOUT_SECONDS - 1)
    return restore


def test_power_save_restores_after_timeout_offroad(restored):
    onroad = Mock(return_value=False)
    power.check_and_restore_power_save(onroad)
    restored.assert_called_once_with()
    assert onroad.call_count == 2
    assert power.last_activity_time is None


def test_power_save_does_not_disable_cores_onroad(restored):
    power.check_and_restore_power_save(lambda: True)
    restored.assert_not_called()
    assert power.last_activity_time is None


def test_power_save_rechecks_onroad_after_idle_check(restored):
    onroad = Mock(side_effect=[False, True])
    power.check_and_restore_power_save(onroad)
    restored.assert_not_called()
    assert onroad.call_count == 2
    assert power.last_activity_time is None


def test_power_save_waits_for_idle_timeout(monkeypatch, restored):
    monkeypatch.setattr(power, "last_activity_time", 1000 - power.IDLE_TIMEOUT_SECONDS)
    power.check_and_restore_power_save(lambda: False)
    restored.assert_not_called()
    assert power.last_activity_time is not None


def test_power_save_does_nothing_without_remux_activity(monkeypatch, restored):
    monkeypatch.setattr(power, "last_activity_time", None)
    onroad = Mock()
    power.check_and_restore_power_save(onroad)
    restored.assert_not_called()
    onroad.assert_not_called()
