# Setting up the environment from scratch — full guide

This file describes **everything** you need to install on a clean Windows machine to run the IsaacPilot project: WSL2 (Linux inside Windows), VS Code, the Python environment for Isaac Sim, a Jupyter kernel for the notebooks, and ArduPilot SITL.

The order in this file = the installation order. Every command is explained: what it does, and what happens if you change a parameter.

> **Paths note:** throughout this guide `C:\path\to\IsaacPilot` stands for the folder where you cloned this repository. Substitute your own path.

---

## 0. Get the project

Clone (or download) this repository to a folder on Windows — for example `C:\IsaacPilot`:

```powershell
git clone https://github.com/tomtto/IsaacPilot.git
```

No git on Windows? Install it from **git-scm.com**, or download the repo as a ZIP from the GitHub page (green **Code** button → **Download ZIP**) and extract it.

This folder is the `C:\path\to\IsaacPilot` referred to throughout this guide.

---

## 1. WSL2 (Linux inside Windows)

### Why you need it
ArduPilot SITL (the flight-controller simulator) is officially built and runs only on Linux. Isaac Sim, on the other hand, is installed on Windows (it needs an NVIDIA GPU with drivers directly in the host system). WSL2 is a way to run a real Linux kernel (Ubuntu) inside Windows without a second computer and without a virtual machine in the usual sense (VirtualBox/VMware) — Microsoft built lightweight virtualization right into Windows.

### Requirements
- Windows 10 version 2004+ (build 19041+) or Windows 11
- Virtualization (Intel VT-x / AMD-V) must be enabled in the BIOS/UEFI — on most modern PCs it is on by default
- Check your Windows version: `Win+R` → `winver`

### Installation
Open **PowerShell as administrator** (right-click Start → "Terminal (Admin)" or "Windows PowerShell (Admin)") and run:

```powershell
wsl --install
```

**What this command does:**
- Enables the required Windows components (Virtual Machine Platform, Windows Subsystem for Linux) — these are Windows features that were disabled until now
- Downloads and installs **Ubuntu** (the default distribution) — specifically the latest LTS version available in the Microsoft Store
- Installs WSL version 2 (unlike WSL1, WSL2 is a real Linux kernel rather than a system-call translator; SITL and networking only work on WSL2)

If you want a specific Ubuntu version (for example, if the project was tested on 22.04), specify it explicitly:
```powershell
wsl --install -d Ubuntu-22.04
```
List all available distributions: `wsl --list --online`.

### Reboot
After `wsl --install`, **you must reboot the computer** — the command enables Windows system components that only take effect after a restart.

### First launch
After the reboot, Ubuntu will start on its own (or find it in Start → Ubuntu). The first launch will ask you to:
1. Choose a **Linux username** (it can differ from your Windows username, Latin letters, no spaces)
2. Choose a **password** — it is typed blind (characters are not shown, not even as dots), which is normal for Linux terminals

This user becomes the administrator (sudo) inside Ubuntu.

### Verify the install
In regular PowerShell (not in Ubuntu):
```powershell
wsl --list --verbose
```
It should show something like:
```
  NAME      STATE           VERSION
* Ubuntu    Running         2
```
The **VERSION column must be 2**, not 1 — otherwise SITL and networking will not work. If it says 1: `wsl --set-version Ubuntu 2`.

### Update the system
Inside Ubuntu (open it from Start, or type `wsl` in PowerShell):
```bash
sudo apt update && sudo apt upgrade -y
```
`apt update` refreshes the list of available packages (not the packages themselves); `apt upgrade -y` installs the latest versions of already-installed packages, and `-y` means "answer yes to all prompts automatically". This is the standard first step on any fresh Ubuntu.

---

## 2. Visual Studio Code

### Download
Official site: **code.visualstudio.com** → the "Download for Windows" button. It downloads a `.exe` installer (the System Installer, installed machine-wide — recommended, as opposed to the User Installer).

### Installation
Run the downloaded `.exe`. On the "Select Additional Tasks" screen I recommend checking:
- **Add "Open with Code" action to Windows Explorer file context menu** — right-click a folder → "Open with Code", handy
- **Add to PATH** (usually already checked) — without it the `code` command won't work in a terminal

The remaining steps — "Next" with defaults.

### Required extensions
Open VS Code → the squares icon on the left (Extensions, `Ctrl+Shift+X`) → search for and install (the Install button):

| Extension | ID | Why |
|---|---|---|
| **Python** | `ms-python.python` | Syntax highlighting, autocompletion, running .py files |
| **Jupyter** | `ms-toolsai.jupyter` | Opening and running `.ipynb` notebooks directly in VS Code (our `drone_control_ardupilot.ipynb` and `flight_missions.ipynb`) |
| **WSL** | `ms-vscode-remote.remote-wsl` | Lets you open VS Code "inside" Ubuntu (a `/home/...` or `/mnt/c/...` folder from Linux's point of view) — useful when editing ArduPilot files |

After installing, restart VS Code (it will offer a "Reload" button).

---

## 3. Python environment for Isaac Sim (`isaac_env`)

Isaac Sim 6.0 is not installed as a standalone program with an installer, but as a **regular Python package via pip**, inside an isolated virtual environment (venv). This lets you keep several versions of Isaac Sim / Python side by side without conflicts.

### 3.1 Enable Windows long paths (LongPaths)
Isaac Sim unpacks thousands of files with very long paths (deeply nested extension folders) — Windows' standard 260-character path limit can't handle this, and installation fails with cryptic errors. This must be enabled **once per system, BEFORE installing Isaac Sim**.

PowerShell **as administrator**:
```powershell
Set-ItemProperty -Path 'HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem' -Name 'LongPathsEnabled' -Value 1
```
**What it does:** sets the Windows registry value `LongPathsEnabled` to 1 (enabled). The registry is Windows' system settings database; `HKLM:\...` means "applies to the whole machine", not a single user. A reboot afterward is not required, but signing out and back in is safer.

### 3.2 Install Python 3.12
Isaac Sim 6.0.1.0 targets Python 3.12 (the project used 3.12.7). Download from **python.org/downloads** → the Windows section → "Windows installer (64-bit)" for a 3.12.x version.

During installation, **be sure to check "Add python.exe to PATH"** on the installer's first screen — without it the `python` command won't be found in a terminal.

Check after installation (open a fresh PowerShell — existing windows won't pick up the new PATH):
```powershell
python --version
```
It should print `Python 3.12.7` (or a close 3.12.x).

### 3.3 Create the virtual environment
In PowerShell, go to the project folder and create a venv named `isaac_env`:
```powershell
cd C:\path\to\IsaacPilot
python -m venv isaac_env
```
**What it does:** `python -m venv <name>` creates an `isaac_env` folder — inside it a copy of the Python interpreter and empty space for libraries, isolated from the system Python. This keeps Isaac Sim's packages from mixing with other Python projects on the machine.

Activate the environment (do this in every new terminal before working on the project):
```powershell
.\isaac_env\Scripts\activate
```
After activation, `(isaac_env)` appears at the start of the terminal line — a sign that all subsequent `pip install` and `python` calls go into this isolated environment rather than the system Python.

### 3.4 Install Isaac Sim
Inside the activated `isaac_env`:
```powershell
pip install isaacsim[all,extscache]==6.0.1.0 --extra-index-url https://pypi.nvidia.com
```
**What it does:**
- `isaacsim[all,extscache]` — installs the core Isaac Sim package plus **all** optional extensions (`all`) and the extension cache (`extscache`, so they aren't pulled over the network on first launch)
- `==6.0.1.0` — pins the exact version. **Don't remove it** — on other versions the `omni.physx`/`isaacsim.code_editor` API may differ, and the entire bridge code was written and tested against 6.0.1.0
- `--extra-index-url` — Isaac Sim packages live not on the regular PyPI but on a separate NVIDIA server; this option tells pip to also look there

The install is heavy (several gigabytes) and can take 10-20+ minutes depending on your connection.

### 3.5 First launch of Isaac Sim
From the activated environment:
```powershell
isaacsim
```
The first launch takes longer than usual — Isaac Sim compiles shaders and unpacks the extension cache. Then the main window opens (Viewport, Stage, Property, etc.).

### 3.6 Enable the required extension
Inside Isaac Sim: menu **Window → Extensions** → search for `isaacsim.code_editor.python_server` → enable it (toggle). This opens TCP port **8226**, through which the notebook (`bridge_up()`) sends commands straight into the running Isaac Sim.

**Important:** keep the `isaacsim.code_editor.jupyter` extension **disabled** — in version 6.0.1.0 it conflicts and breaks the session. We use only `python_server`.

---

## 4. Jupyter kernel for the notebooks

The project's notebooks (`drone_control_ardupilot.ipynb`, `flight_missions.ipynb`) must run with exactly the Python interpreter where Isaac Sim and all the bridge libraries are installed — that is, inside `isaac_env`. Jupyter calls such an interpreter a **kernel**. For VS Code to see `isaac_env` as a separate kernel in the list, you need to **register** it.

### 4.1 Install ipykernel and pymavlink into the environment
In the activated `isaac_env` (the terminal must show `(isaac_env)` at the start of the line):
```powershell
pip install ipykernel pymavlink
```
- `ipykernel` — turns a Python interpreter into a Jupyter kernel (a separate process that receives code from a notebook cell and returns the result).
- `pymavlink` — the MAVLink protocol library. `flight_missions.ipynb` uses it to send flight commands (arm / takeoff / goto / land) to the drone over UDP 14550. Required to fly the drone; the bridge itself (`drone_control_ardupilot.ipynb`) does not need it. It is a standalone package — `pip install isaacsim` does **not** bring it in.

### 4.2 Register the kernel
```powershell
python -m ipykernel install --user --name isaac_env --display-name "Isaac Env (isaac_env)"
```
**Breakdown of the parameters:**
- `--user` — registers the kernel only for the current Windows user (no admin rights needed), writing the config to `%APPDATA%\jupyter\kernels\isaac_env\`
- `--name isaac_env` — the internal technical name of the kernel (used by the system; no need to change it)
- `--display-name "Isaac Env (isaac_env)"` — what you **see in the dropdown** when picking a kernel in VS Code. You can change it to any label you like, e.g. `"Isaac Sim 6.0.1"` — it doesn't affect anything, it's just a human-facing label

### 4.3 Select the kernel in VS Code
1. Open `notebooks\drone_control_ardupilot.ipynb` in VS Code
2. In the top-right corner of the notebook window — the button with the current kernel (usually shows "Select Kernel" after the first open)
3. Click it → **Jupyter Kernel...** (not "Python Environments", but specifically the section with registered Jupyter kernels) → pick **"Isaac Env (isaac_env)"** from the list

If `isaac_env` doesn't appear in the list — fully restart VS Code (kernel registration is read at startup), or re-check step 4.2 (a common cause: the command was run without activating the venv, and `ipykernel` was installed into the system Python instead of `isaac_env`).

### 4.4 Check
Run the first cell of any notebook (`Shift+Enter`). If everything is correct, the code runs without `ModuleNotFoundError`. If you get a missing-module error (`isaacsim`, `pymavlink`, etc.) — you either picked the wrong kernel, or the package wasn't installed into `isaac_env` (see below).

---

## 5. ArduPilot SITL (inside WSL/Ubuntu)

SITL (Software In The Loop) is the ArduCopter autopilot program itself, compiled so that its physics is supplied not by a real drone but by our bridge from Isaac Sim.

Open Ubuntu (via Start, or type `wsl` in PowerShell) and run in order:

### 5.1 Install build dependencies
```bash
sudo apt update
sudo apt install -y git python3-pip python3-venv build-essential ccache
```
These packages are needed to compile ArduPilot from source: `git` — to fetch the code, `build-essential` — the C/C++ compiler and linker, `ccache` — a compilation cache (speeds up repeated rebuilds), `python3-pip`/`python3-venv` — for ArduPilot's own Python dependencies (mavproxy, etc.).

### 5.2 Clone ArduPilot
```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```
**`--recurse-submodules`** — mandatory: the ArduPilot repository references several nested git repositories (libraries such as mavlink); without this flag they stay empty folders and the build fails.

### 5.3 Run ArduPilot's prerequisites script
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
ArduPilot's official script; it installs all the autopilot-build-specific dependencies (the correct versions of the mavlink/pymavlink/pyserial Python libraries, etc.). `-y` — don't ask for confirmation at each step. Afterward the script will ask you to reopen the terminal (`exec bash` or reopen Ubuntu) — important to do, as it appends environment variables (`PATH` for `ccache`, etc.) to `~/.bashrc`.

### 5.4 Build SITL for a quadcopter
```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```
`waf` is ArduPilot's own build system (like `make`/`cmake`). `configure --board sitl` says we are building not for a real board (Matek/Pixhawk) but for the virtual "SITL" board. `./waf copter` compiles specifically the ArduCopter firmware (quad/multicopter; for a plane it would be `./waf plane`). The result appears in `~/ardupilot/build/sitl/bin/arducopter` — that's the file `launch_sitl()` from the notebook runs.

The first build can take 5-15 minutes.

### 5.5 (Optional) venv for ArduPilot Python tools
The project notes mention activating a `venv-ardupilot` in the launch script — if you want to reproduce it exactly:
```bash
python3 -m venv ~/venv-ardupilot
source ~/venv-ardupilot/bin/activate
pip install pymavlink mavproxy
```
This is a separate **Linux** environment (don't confuse it with the Windows `isaac_env`!) — needed if you launch SITL via `sim_vehicle.py` (a wrapper that brings up the familiar ArduCopter console window with MavProxy for diagnostics), as described in `README.md`.

---

## 6. Testing the whole chain

### 6.1 Firewall rules (once, PowerShell as administrator)
```powershell
New-NetFirewallRule -DisplayName "ArduPilot JSON UDP 9002" -Direction Inbound -Protocol UDP -LocalPort 9002 -Action Allow
New-NetFirewallRule -DisplayName "ArduPilot MAVLink UDP 14550" -Direction Inbound -Protocol UDP -LocalPort 14550 -Action Allow
```
WSL2 lives on its own virtual network — traffic from SITL (inside Ubuntu) to Isaac Sim (on the Windows host) goes through a virtual network adapter, and Windows Defender Firewall blocks inbound UDP packets on non-standard ports by default. These two rules allow port **9002** (physics/JSON) and **14550** (MAVLink commands) — without them Isaac Sim will never receive data from SITL, even if everything else is configured correctly.

### 6.2 Quick smoke test
1. Open Isaac Sim (`isaacsim` in the activated `isaac_env`), open/create a scene containing a drone named `cf2x`, and enable `isaacsim.code_editor.python_server`

   > **Adding the drone to a scene:** the repo ships the drone model at `assets/cf2x/cf2x.usd` (the scenes themselves are not included — they are large and gitignored). On any stage — an empty one or any environment — add the drone: **File → Import** (or drag `assets/cf2x/cf2x.usd` into the Stage / Viewport). Make sure the resulting prim is named **`cf2x`** in the Stage tree (rename it if the import used a different name) — the bridge finds the drone by this name. Save the stage (**File → Save As…**) to reuse it next time.

   > **Using your own drone:** the bridge locates the drone by the prim name `cf2x`. To use your own drone, name its root prim `cf2x` in the scene (the path can be anything). Note: the physics is tuned for the provided Iris-class quad (1.5 kg, servo remap, Fix-12) — a different airframe needs re-tuning of mass, motor layout and servo mapping (see the sign-test tool and TROUBLESHOOTING.md).

2. In VS Code open `notebooks/drone_control_ardupilot.ipynb`, select the `isaac_env` kernel, and run the cells in order:
   ```python
   from isaac_bridge import *
   await bridge_up()
   launch_sitl()
   await diag()
   ```
3. `diag()` should show a growing packet counter and `dt≈4.17ms` — if so, the whole WSL↔Windows↔Isaac Sim chain is working

If something is off — see the problems table in the main `README.md` (the common-errors section) and the detailed log in `TROUBLESHOOTING.md`.

---

## Summary map: what goes where

| Component | Where | Why |
|---|---|---|
| WSL2 + Ubuntu | Windows feature + Linux system | Running ArduPilot SITL |
| VS Code | Windows, regular program | Code editor + running the notebooks |
| Python 3.12 | Windows | Interpreter for `isaac_env` |
| `isaac_env` (venv) | `C:\path\to\IsaacPilot\isaac_env` | Isolated environment with Isaac Sim and the bridge dependencies |
| Isaac Sim 6.0.1.0 | Inside `isaac_env` (pip package) | Physical drone simulator |
| `ipykernel` in `isaac_env` | Inside `isaac_env` | Lets VS Code run the notebooks with this exact interpreter |
| ArduPilot SITL | Inside Ubuntu (`~/ardupilot`) | Autopilot, the same code that will fly on a real Matek H743 |
