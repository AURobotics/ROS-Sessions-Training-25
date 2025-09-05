# ROS 2 Workspace (ros2_ws)
This repository contains a ready-to-use **ROS 2 Humble workspace** inside a **VS Code Dev Container**.  
It ensures that everyone has the same development environment without worrying about operating system differences.

---

## Prerequisites

### 1. Visual Studio Code

### 2. Docker

#### For Windows:
- To ensure the installation, open PowerShell and `docker --version` should print the docker version.
- During installation, enable **WSL2 backend**.  
- After installation, open Docker Desktop → **Settings → Resources → WSL Integration** → enable your Ubuntu distribution.

#### For Linux:
- Install Docker Engine following official instructions: [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

---

### Docker Permission Notes

Sometimes running Docker commands may fail with a **permission denied** error:  

```
docker: permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock
```

This happens when your user doesn’t have access to the Docker daemon.

#### Linux Solution
1. Add your user to the `docker` group:
```bash
sudo usermod -aG docker $USER
```
2. Apply the new group membership:
```bash
newgrp docker
```
3. Test Docker:
```bash
docker run hello-world
```
💡 You may need to **log out and log back in** (or restart VS Code) for the changes to take effect.
if the issue still presists on VS Code:
1. Open your terminal
2. Navigate to your cloned repo directory
3. ```bash
   newgrp docker
   docker run hello-world
   code .
   ```

#### Windows Solution (WSL2 / Docker Desktop)
- Ensure your WSL2 Ubuntu distribution is enabled in Docker Desktop:
  1. Open Docker Desktop → **Settings → Resources → WSL Integration**.
  2. Enable your Ubuntu distribution.
- Restart your VS Code or open a **new terminal** inside the WSL2 Ubuntu session.
- Test Docker:
```bash
docker run hello-world
```

✅ After following these steps, your user should be able to run Docker commands without `sudo` (Linux) or permission issues (Windows WSL2).

---

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

- Once you're connected, notice the green remote indicator on the left of the Status bar to show you are inside the dev container:

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

## To Ensure ROS2 Installation
Run this command:
```bash
echo $ROS_DISTRO
```
