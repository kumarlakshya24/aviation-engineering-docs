.PHONY: install test lint visuals

install:
	uv sync

test:
	uv run pytest -q

lint:
	uv run ruff check .

visuals:
	uv run scripts/make_visuals.py
