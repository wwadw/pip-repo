from types import SimpleNamespace

import rerun_projection.cli as cli_module


def test_exit_handler_disconnects_recording_and_exits():
    disconnected = []
    exits = []
    runtime = SimpleNamespace(recording=SimpleNamespace(disconnect=lambda: disconnected.append(True)))

    handler = cli_module.build_exit_handler(runtime, exit_fn=lambda code: exits.append(code))

    handler(None, None)

    assert disconnected == [True]
    assert exits == [0]
