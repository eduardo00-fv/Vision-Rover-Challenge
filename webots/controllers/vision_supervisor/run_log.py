"""Registro NDJSON de una corrida Webots.

La traza se mantiene deliberadamente simple y de biblioteca estándar para que
pueda leerse en una máquina de CI aunque Webots no esté instalado.
"""

import json
import time
from pathlib import Path


class RunLog:
    def __init__(self, directory, scenario, config):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        started_at_ms = int(time.time() * 1000)
        self.path = directory / "{}_{}.ndjson".format(scenario["id"], started_at_ms)
        self._file = self.path.open("x", encoding="utf-8")
        self.write("metadata", scenario=scenario, contract_config=config, started_at_ms=started_at_ms)

    def write(self, kind, **data):
        line = {"type": kind, **data}
        self._file.write(json.dumps(line, separators=(",", ":"), ensure_ascii=False) + "\n")
        self._file.flush()

    def close(self, **summary):
        if not self._file.closed:
            self.write("summary", **summary)
            self._file.close()
