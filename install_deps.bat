@echo off
cd /d "%~dp0vlm"
python -m venv .venv
.venv\Scripts\python -m pip install --upgrade pip
.venv\Scripts\pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
.venv\Scripts\pip install ultralytics roslibpy opencv-python requests numpy git+https://github.com/ultralytics/CLIP.git
echo.
echo Done. Dependencies installed in vlm\.venv
pause
