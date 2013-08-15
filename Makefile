all:
	python setup.py sdist
	pip install -r requirements.txt

install: all
