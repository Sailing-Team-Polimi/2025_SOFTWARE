# 🐳 ROS 2 Jazzy Docker Environment

This repository provides a fully containerized **ROS 2 Jazzy** development environment using **Docker**, **Docker Compose**, and a custom **entrypoint** script. It allows you to start developing ROS 2 packages easily on any machine, without worrying about local setup or dependencies.

## 📦 Overview

The container setup includes:

- A `Dockerfile` that builds a ROS 2 Jazzy base image.
- A `compose.yaml` file to run the container with optional shared volumes.
- An `entrypoint.sh` script that initializes the environment and passes commands correctly.

This is ideal for robotics development, CI setups, or cross-platform compatibility when using ROS 2.

For an overview of what docker is, how it works and why we use it, see the [Why Docker README](why_docker.md).

---

## 💻 Required Software for Development
To work efficiently with this ROS 2 Jazzy Docker-based framework, we recommend following the instructions in [Software Necessary README](software_necessary.md).

---

## 🚀 Getting Started

### 1. Customize and Run the Docker Compose File

To allow the container to access the `ros2_ws/` folder from this repository, it must be mounted as a shared volume.

Follow these steps:

1. Copy the entire `orca_container` folder and paste it inside the `Docker` folder using a new name of your choice (this avoids modifying the original example).
2. Open the `compose.yaml` file inside your copied folder and replace `_path_to_repo_` with the **full absolute path** to your local copy of the repository.
3. Open a terminal in the folder where the `compose.yaml` file is located and run:

    ```bash
    docker compose up -d
    ```

This command will build the ROS 2 container with all dependencies and mount the ros2_ws/ folder into the container.

> ⚠️ The first build might take several minutes, as it requires downloading and setting up the entire image. After that, the container can be stopped, restarted, or rebuilt quickly since the image will be cached on your machine.

### 2. Attach a VSCode window to the running container

Using the extensions listed in the [Software Necessary README](software_necessary.md) , you can connect to the container directly from VS Code:

If you see the Docker extension panel (usually on the sidebar), go to the Containers section, right-click on the newly created container, and select Attach Visual Studio Code.

Alternatively, press F1, search for Dev Containers: Attach to Running Container, and select your container from the dropdown list.

You should now have a VS Code window open inside the container environment.

### 3. Build the ROS 2 Workspace

With the container running and VS Code attached:

1. In the VS Code window, navigate to the ros2_ws/ folder inside the container.
2. Open a terminal (it should open in the workspace folder by default).
3. Run:
    ```bash
    colcon build
    ```
    This will build all the ROS 2 packages in the repository.
4. You can now start testing the software by running or launching nodes as needed.

### 4. Check the Graphical Interface (X Server)

The final step is to verify that the container can display graphical applications (e.g., rqt, rviz2) on your host computer.

To test this:

1. Make sure your host system is running an X Server (as provided in the [README](software_necessary.md))
2. In the container terminal, run a graphical command such as: `rqt`, `rqt_graph` or `rviz2`

