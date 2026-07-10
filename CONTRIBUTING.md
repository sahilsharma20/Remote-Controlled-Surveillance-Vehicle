# Contributing

Contributions that improve code quality, documentation, hardware safety or reproducibility are welcome.

## Development setup

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
python -m pip install -e ".[dev]"
ruff check src tests main.py
pytest -q
```

## Pull-request expectations

- Keep perception, geometry and hardware-I/O concerns separated.
- Add tests for pure logic changes.
- Avoid hard-coded local serial ports.
- Preserve bounded actuator commands and cleanup behaviour.
- Document hardware assumptions and limitations honestly.
- Do not add functionality intended to harm people or animals.
