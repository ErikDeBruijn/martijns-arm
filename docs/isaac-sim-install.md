# Isaac Sim 4.5+ installatie en gebruik (na VM/LXC creatie)

**Doel:** RL-training voor het 3-DOF robotarm-project, met ROS2 Jazzy bridge.
**Voorwaarde:** VM of LXC `isaac-sim` is aangemaakt en `nvidia-smi` werkt erin (zie `isaac-sim-vm-plan.md`).

## Versie keuze

NVIDIA heeft Omniverse Launcher gedeprecateerd. Voor Isaac Sim 4.5 en nieuwer (5.x) is de standalone download de juiste route:

- **Isaac Sim 5.x** (laatste): https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html
- **NGC catalog**: https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/containers/sim
- Container variant: `nvcr.io/nvidia/isaac-sim:<version>` — handig voor headless / RL-training.

Voor RL is de **container variant** vaak de pijnloze keuze (geen lokale dependency-hell). De workstation-install is beter voor interactieve scene-bouw.

### Licentie
Isaac Sim is gratis voor individueel gebruik en non-commercial onderzoek. Erik valt onder die voorwaarden voor dit hobby-project. Een NVIDIA developer-account (gratis) is wel nodig voor de NGC download.

## Optie 1: Container install (aanbevolen voor RL)

Binnen de VM/CT (na `nvidia-smi` werkend):

```bash
# Docker installeren
curl -fsSL https://get.docker.com | sh
systemctl enable --now docker

# NVIDIA Container Toolkit
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
apt update && apt install -y nvidia-container-toolkit
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker

# NGC login (vereist NGC API key — maak op https://ngc.nvidia.com/setup/api-key)
docker login nvcr.io
# Username: $oauthtoken
# Password: <NGC API key>

# Pull (controleer laatste tag op NGC catalog)
docker pull nvcr.io/nvidia/isaac-sim:5.0.0

# Headless RL run
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all \
  -e "ACCEPT_EULA=Y" --rm --network=host \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  nvcr.io/nvidia/isaac-sim:5.0.0
```

LXC-specifiek: in unprivileged LXC heeft Docker extra config nodig (`features: nesting=1` is al gezet, maar kijk uit voor cgroupv2 issues).

## Optie 2: Workstation install (interactief)

```bash
# Vereisten
apt install -y libglu1-mesa libxcursor1 libxinerama1 libxrandr2 libxi6 \
  libgl1 libglfw3 libxkbcommon0 libxcb-cursor0 mesa-vulkan-drivers vulkan-tools

# Download via NVIDIA developer portal (account vereist):
# https://developer.nvidia.com/isaac-sim
# of via NGC CLI:
ngc registry resource download-version "nvidia/isaac-sim/isaac-sim-standalone:5.0.0"

# Unzip naar ~/isaacsim
cd ~/isaacsim
./isaac-sim.sh --no-window  # headless test
```

## Headless gebruik vanaf Erik's Mac

Isaac Sim ondersteunt **WebRTC streaming** via de `omni.services.streamclient.webrtc` extension. Aanpak:

1. Start Isaac Sim met streaming flag:
   ```bash
   ./isaac-sim.streaming.sh   # workstation
   # of in container: ./runheadless.native.sh
   ```
2. Open op Mac: `https://<isaac-sim-vm-ip>:8011/streaming/client/`
   (over Tailscale: vervang IP door Tailscale-hostnaam)
3. Voor lage latency en stabiliteit: gebruik de **Omniverse Streaming Client** desktop app (download via NVIDIA) i.p.v. de browser.

Alternatief: VNC of NoMachine voor desktop-style remote — minder geschikt voor de Vulkan/RTX viewer, beter werken met WebRTC.

## ROS2 Jazzy bridge

Isaac Sim ondersteunt ROS2 Humble out of the box; Jazzy support is in 4.5+ stabieler maar kan extra config vereisen.

```bash
# Install ROS2 Jazzy in dezelfde VM/CT (Ubuntu 24.04 = native Jazzy support)
locale  # check UTF-8
apt install -y software-properties-common
add-apt-repository universe
apt update && apt install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list
apt update && apt install -y ros-jazzy-desktop python3-colcon-common-extensions
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

Binnen Isaac Sim: enable extension `omni.isaac.ros2_bridge` (of `omni.isaac.ros2_bridge-jazzy` indien aparte build). Set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` of `rmw_cyclonedds_cpp` matching met de host-kant.

Verifieer met:
```bash
# In één terminal (Isaac Sim draait):
ros2 topic list
# Verwacht: /clock, /tf, /joint_states (afhankelijk van scene)
```

## URDF/USD voor de 3-DOF arm

Erik's bestaande `ros2_ws` (zie `/Users/erik/Dev/robotics/martijns-arm/ros2_ws`) bevat URDFs. Voor Isaac Sim:

1. Converteer URDF → USD via Isaac Sim's URDF importer (Tools menu of script_editor):
   ```python
   from omni.importer.urdf import _urdf
   urdf_interface = _urdf.acquire_urdf_interface()
   # zie docs.isaacsim.omniverse.nvidia.com voor laatste API
   ```
2. Sla USD op in `~/isaac-assets/martijns-arm.usd`.
3. Bouw RL-environment met **Isaac Lab** (vroeger Orbit) — opvolger van OmniIsaacGymEnvs:
   ```bash
   git clone https://github.com/isaac-sim/IsaacLab.git ~/IsaacLab
   cd ~/IsaacLab
   ./isaaclab.sh --install
   ```

## RL workflow

1. Definieer task in `IsaacLab/source/extensions/omni.isaac.lab_tasks/...` met de arm-USD.
2. Train headless met PPO/SAC:
   ```bash
   ./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py \
     --task Isaac-MartijnsArm-v0 --headless --num_envs 4096
   ```
3. RTX PRO 6000 (96 GB VRAM) kan duizenden parallelle envs aan — start met 4096, schaal op.

## TODO (voor Erik)
- [ ] NGC API key aanmaken op https://ngc.nvidia.com/setup/api-key (gratis account)
- [ ] Tailscale auth key in 1Password zetten of nieuwe genereren
- [ ] Beslissen: Optie A/B/C uit `isaac-sim-vm-plan.md`
- [ ] Na install: URDF van de arm omzetten naar USD
- [ ] Isaac Lab task definiëren voor de specifieke 3-DOF reach/grasp opdracht
