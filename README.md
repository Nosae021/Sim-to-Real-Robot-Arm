# Sim-to-Real-Robot-Arm
## Environment Setup Guide for Simulation

### Step 1. Create a virtual environment
```powershell
python -m venv .venv
```

```powershell
.\.venv\Scripts\Activate.ps1
```

#### Notice: If security error occurred:
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process
```

### Step 2. Install Packages
```powershell
python -m pip install --upgrade pip
```

```powershell
pip install pybullet gymnasium stable-baselines3 numpy matplotlib tensorboard
```

To generate requirements.txt:

```powershell
pip freeze > requirements.txt
```
