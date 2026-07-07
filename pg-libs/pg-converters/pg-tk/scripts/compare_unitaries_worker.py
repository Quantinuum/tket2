"""Long-lived worker for comparing TKET circuit unitaries.

Reads newline-delimited JSON requests from stdin. Each request has the form
``{"first": <str>, "second": <str>}``, where each value is a serialized TKET
circuit as produced by ``pg_to_tk_json(...).to_string()`` in Rust.

Writes one newline-delimited JSON response per request:
- ``{"result": true|false}`` on success.
- ``{"error": "<full traceback>"}`` if a request fails.
"""

import json
import sys
import traceback

from pytket.circuit import Circuit
from pytket.utils import compare_unitaries


def compare_request(data: dict) -> dict:
    first = data["first"]
    second = data["second"]
    result = compare_unitaries(
        Circuit.from_dict(json.loads(first)).get_unitary(),
        Circuit.from_dict(json.loads(second)).get_unitary(),
    )
    return {"result": bool(result)}


def main() -> None:
    for line in sys.stdin:
        if not line.strip():
            continue
        try:
            response = compare_request(json.loads(line))
        except Exception:
            response = {"error": traceback.format_exc()}
        print(json.dumps(response), flush=True)


if __name__ == "__main__":
    main()
