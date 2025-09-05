# ROS 2 Workspace (ros2_ws) 
This repository contains a ready-to-use **ROS 2 Humble workspace** inside a **VS Code Dev Container**.  
It ensures that everyone has the same development environment without worrying about operating system differences.

---

## Prerequisites

### 1. Install Visual Studio Code
from here: https://code.visualstudio.com/download

### 2. Install Docker Desktop
from here: https://docs.docker.com/desktop/  

- To ensure the installation , open PowerShell and `docker --version` should print the docker version 
- During installation, enable **WSL2 backend**.  
- After installation, open Docker Desktop → **Settings → Resources → WSL Integration** → enable your Ubuntu distribution.

### 3. Install VS Code Extensions
- [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)  
- [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)  

---

## 📂 Getting Started

### 1. Clone this Repository
```bash
git clone <your-repo-link>
cd <repo-name>
```

### 2. Open in VS Code
- Open the repo folder in **VS Code**.  
- Press **Ctrl+Shift+P** → search for **Dev Containers: Rebuild and Reopen in Container**.  
- Wait while Docker builds the image (first build may take ~10–15 minutes).  

- Once you're connected, notice the green remote indicator on the left of the Status bar to show you are connected to your dev container: 

![Screenshot to make sure you're working inside the dev container](path/to/image.png)

---


## Using the Workspace

### 1. Open the Workspace
By default, your ROS 2 workspace is located at:
```bash
~/ros2_ws
```

### 2. Build the Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

💡 **Tip:** Add this line to your `~/.bashrc` so ROS 2 sources automatically in every terminal:
```bash
source ~/ros2_ws/install/setup.bash
```

---

##  To Ensure ROS2 Installation: 

### Run this Command:
```bash
echo $ROS_DISTRO
```
---

