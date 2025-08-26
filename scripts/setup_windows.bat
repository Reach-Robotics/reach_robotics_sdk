@echo off

:: Create a Python virtual environment
@REM python -m venv venv

:: Activate the virtual environment
@REM call venv\Scripts\activate

:: Install dependencies
pip install -r requirements.txt
pip install -r .docsrc/sphinx_requirements.txt