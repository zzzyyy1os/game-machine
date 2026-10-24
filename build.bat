@echo off
set MSYSTEM=
set MINGW_PREFIX=
set IDF_PATH=D:\esp\v5.5.4\esp-idf
set IDF_TOOLS_PATH=C:\Espressif\tools
set IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v5.5.4\venv
set PATH=C:\Espressif\tools\python\v5.5.4\venv\Scripts;C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin;C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\ccache\4.12.1;C:\Espressif\tools\python;%PATH%
cd /d E:\esp\game machine
python D:\esp\v5.5.4\esp-idf\tools\idf.py %*
