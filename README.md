# CSS-CAV
Install PyEnv
Open Powershell to run as admin
Run: Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope LocalMachine
open new PowerShell
Run: Invoke-WebRequest -UseBasicParsing -Uri "https://raw.githubusercontent.com/pyenv-win/pyenv-win/master/pyenv-win/install-pyenv-win.ps1" -OutFile "./install-pyenv-win.ps1"; &"./install-pyenv-win.ps1"
next you install Python
Go to another PowerShell
-go to the git repo
-Run: pyenv install 3.8.10
-Run: pyenv global 3.8.10
-Run: pyenv local 3.8.10
confirm if running correctly with: python --version (should be 3.8.10)
Install Carla
-run: pip install carla == 0.9.13
find where you have the CarlaUE4.exe file located by copying the path
restructure file such as test.py depending on the file cath for Carla. for example: cwd = 'C:/Users/yperez20/Carla0.9'
exe_path = f'{cwd}/CarlaUE4.exe'
port = 2000
subprocess.Popen([exe_path, '-quality-level=Low', f'-carla-port={port}'], cwd=cwd)
time.sleep(10)  # Wait for the simulator to start