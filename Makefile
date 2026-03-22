.PHONY: test test-parallel lint

test:
	python3 -m pytest -q

test-parallel:
	python3 -m pytest -q -n auto

lint:
	python3 -m ruff check .
