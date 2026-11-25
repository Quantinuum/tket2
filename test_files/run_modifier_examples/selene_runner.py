"""Compile and execute modifier examples with the local QIS compiler."""

from pathlib import Path

import selene_sim
from hugr import Hugr
from selene_hugr_qis_compiler import EmulatorState
from selene_quest_plugin import QuestPlugin, SeleneQuestState


def run_hugr(
    hugr: Hugr, build_dir: Path, *, n_qubits: int
) -> dict[str, SeleneQuestState]:
    """Run a HUGR and collect its tagged state results."""
    bitcode = EmulatorState.from_python(hugr).compile_to_bitcode()
    runner = selene_sim.build(bitcode, build_dir=build_dir)
    results = runner.run(QuestPlugin(), n_qubits=n_qubits, random_seed=0, timeout=30.0)
    return QuestPlugin.extract_states_dict(results)
