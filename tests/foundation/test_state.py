"""aloxcam.state.ServerState: histories, settings replacement, locking (01 §8, §10.1, design §3.1.4)."""

from __future__ import annotations

import threading

import pytest

from aloxcam.errors import NO_ERROR, ErrorCluster
from aloxcam.settings import HIGH_MAG, LOW_MAG, AloxSettings, ServerOptions
from aloxcam.state import ServerState
from aloxcam.testing import make_state


def test_from_settings() -> None:
    alox = AloxSettings()
    state = ServerState.from_settings(alox)
    assert state.high_mag is alox.high_mag and state.low_mag is alox.low_mag
    assert state.light is alox.light and state.z is alox.z
    assert state.inited == {HIGH_MAG: False, LOW_MAG: False}
    assert state.acq_state == {HIGH_MAG: "Start", LOW_MAG: "Start"}
    assert state.error_out == NO_ERROR and state.server_state == "Listen"


def test_record_command_newest_first_bounded_500() -> None:
    state = make_state()
    for i in range(600):
        state.record_command(f"c{i}")
    assert len(state.commands_history) == 500
    assert state.commands_history[0] == "c599" and state.commands_history[-1] == "c100"
    assert state.last_command == "c599"


def test_histories_bounded_by_options() -> None:
    state = make_state(options=ServerOptions(commands_history=3, error_history=2, temperature_history=4))
    for i in range(10):
        state.set_error_out(ErrorCluster(True, i, "x"))
        state.update_camera(HIGH_MAG, temperature=float(i))
        state.record_camera_iteration()
        state.record_command(str(i))
    assert list(state.commands_history) == ["9", "8", "7"]
    assert [err.code for _, err in state.error_history] == [8, 9]
    assert len(state.temperature_history) == 4
    t, hm, lm = state.temperature_history[-1]
    assert (hm, lm) == (9.0, 0.0) and t > 0


def test_camera_settings_and_replace() -> None:
    state = make_state()
    old = state.camera_settings(LOW_MAG)
    new = state.replace_camera_settings(LOW_MAG, horizontal_flip=True)
    assert new is not old and new.horizontal_flip and not old.horizontal_flip
    assert state.camera_settings(LOW_MAG) is new and state.low_mag is new
    assert state.camera_settings(HIGH_MAG) is state.high_mag
    with pytest.raises(KeyError):
        state.camera_settings("Mid Mag.")
    with pytest.raises(KeyError):
        state.replace_camera_settings("Mid Mag.", flat_field=True)
    old_z = state.z
    new_z = state.replace_z_settings(max_pos=24.0)
    assert new_z.max_pos == 24.0 and old_z.max_pos == 20.0 and state.z is new_z


def test_update_and_update_camera() -> None:
    state = make_state()
    state.update(connected=True, last_command="Exit", z_pos=1.5)
    assert (state.connected, state.last_command, state.z_pos) == (True, "Exit", 1.5)
    with pytest.raises(AttributeError):
        state.update(connected=False, no_such_field=1)
    assert state.connected                      # nothing written on failure
    with pytest.raises(AttributeError):
        state.update(lock=None)
    with pytest.raises(AttributeError):
        state.update(commands_history=None)
    state.update_camera(LOW_MAG, inited=True, acq_state="Stopped", temperature=40.5, bus_type="Simulator")
    assert state.inited[LOW_MAG] and state.acq_state[LOW_MAG] == "Stopped"
    assert state.temperature[LOW_MAG] == 40.5 and state.bus_type[LOW_MAG] == "Simulator"
    assert not state.inited[HIGH_MAG]
    with pytest.raises(KeyError):
        state.update_camera("Mid Mag.", inited=True)


def test_snapshot_copies() -> None:
    state = make_state()
    state.record_command("Get Heartbeat")
    snap = state.snapshot()
    assert "lock" not in snap
    assert snap["commands_history"] == ["Get Heartbeat"] and isinstance(snap["commands_history"], list)
    snap["inited"][HIGH_MAG] = True
    snap["commands_history"].append("x")
    assert state.inited[HIGH_MAG] is False
    assert list(state.commands_history) == ["Get Heartbeat"]


def test_concurrent_writers_never_lose_updates() -> None:
    state = make_state(options=ServerOptions(commands_history=100_000))
    n_threads, n_each = 8, 500

    def writer(k: int) -> None:
        for i in range(n_each):
            state.record_command(f"{k}:{i}")
            with state.lock:                     # read-modify-write under the shared RLock
                state.update(z_pos=state.z_pos + 1.0)
            state.replace_z_settings(max_pos=float(i))

    threads = [threading.Thread(target=writer, args=(k,)) for k in range(n_threads)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert len(state.commands_history) == n_threads * n_each
    assert len(set(state.commands_history)) == n_threads * n_each
    assert state.z_pos == n_threads * n_each
