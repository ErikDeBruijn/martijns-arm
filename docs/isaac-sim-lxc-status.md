# Isaac Sim LXC — status

**Datum aanleg:** 2026-05-06
**Status:** Container draait, GPU werkt, Isaac Sim zelf nog NIET geinstalleerd (wacht op Erik's NGC account).

## Container

| Veld | Waarde |
|---|---|
| **CTID** | 140 |
| **Hostname** | isaac-sim |
| **Host** | pve03 (10.1.1.3) |
| **OS** | Ubuntu 24.04 (standard template) |
| **Cores** | 16 |
| **RAM** | 64 GB |
| **Swap** | 4 GB |
| **Rootfs** | 200 GB op `local-lvm` |
| **Network** | DHCP op vmbr0, MAC `bc:24:11:a1:d9:20`, kreeg `10.1.1.130` |
| **Privileges** | Unprivileged, `nesting=1, keyctl=1` |
| **Onboot** | Uit (handmatig starten) |
| **Root password** | In 1Password — item "isaac-sim CT 140 (pve03)", account `my.1password.com`, vault Private |

### GPU passthrough (cgroup-stijl, gelijk aan ollama/comfyui/splat-worker)

```
dev0: /dev/nvidia0,gid=0
dev1: /dev/nvidia1,gid=0
dev2: /dev/nvidiactl,gid=0
dev3: /dev/nvidia-uvm,gid=0
dev4: /dev/nvidia-uvm-tools,gid=0
dev5: /dev/nvidia-modeset,gid=0
mp0: /opt/nvidia-container-libs,mp=/opt/nvidia-libs,ro=1
lxc.mount.entry: /usr/lib/x86_64-linux-gnu/libcuda.so.590.48.01 ... bind,optional,create=file
lxc.mount.entry: /usr/lib/x86_64-linux-gnu/libcuda.so.1 ... bind,optional,create=file
```

`/opt/nvidia-libs/nvidia-smi` is gesymlinked naar `/usr/local/bin/nvidia-smi` en de libs staan in `/etc/ld.so.conf.d/nvidia.conf` zodat alles "uit de doos" werkt zonder `LD_LIBRARY_PATH` gefoezel.

## SSH-toegang

### Aanbevolen (werkt nu, geen Tailscale nodig)

```bash
ssh -i ~/.ssh/pve03_key root@10.1.1.3 'pct enter 140'
```

Of in twee stappen:

```bash
ssh -i ~/.ssh/pve03_key root@10.1.1.3
pct enter 140
```

### Direct via IP — WERKT NU NIET BETROUWBAAR

`ssh -J root@10.1.1.3 root@10.1.1.130` levert nu CT 123 (dllm-experiment) op, niet CT 140. Reden: CT 123 squat op ~150 IP-adressen in `10.1.1.0/24` als secundaire dynamische leases, waaronder `.130`. ARP wint vaak voor CT 123. Dit is een bestaand probleem met CT 123 dat losstaat van Isaac Sim. Workaround: gebruik `pct enter` of wacht op Tailscale.

### Na Tailscale up (toekomstig)

```bash
ssh root@isaac-sim          # via MagicDNS
```

## Geinstalleerd in de container

- `vim`, `git`, `curl`, `wget`, `jq`, `unzip`, `rsync`, `htop`, `tmux`
- `python3-pip`, `python3-venv`, `python3-dev` (Python 3.12.3)
- `build-essential`, `pkg-config`
- `tailscaled` (draait als systemd service, **nog niet geauthenticeerd**)
- Locale `en_US.UTF-8` + `nl_NL.UTF-8`

## nvidia-smi output (vanuit CT 140)

```
Wed May  6 15:12:24 2026
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 590.48.01              Driver Version: 590.48.01      CUDA Version: 13.1     |
+-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA RTX PRO 6000 Blac...    Off |   00000000:02:00.0 Off |                  Off |
| 30%   35C    P8             10W /  300W |    8352MiB /  97887MiB |      0%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+
|   1  NVIDIA RTX PRO 6000 Blac...    Off |   00000000:04:00.0 Off |                  Off |
| 30%   35C    P8             13W /  300W |       4MiB /  97887MiB |      0%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+
```

Beide GPUs zichtbaar. GPU0 is gedeeld met ollama (8.3 GB in gebruik). GPU1 is praktisch leeg (4 MB). Voor RL-training kan Isaac Sim eventueel CUDA_VISIBLE_DEVICES=1 zetten om alleen GPU1 te gebruiken en zo niet met ollama te botsen.

## TODO voor Erik

Een leesbare versie hiervan staat ook in de CT zelf op `/root/TODO-erik.txt`.

1. **Tailscale auth**
   - Genereer key op https://login.tailscale.com/admin/settings/keys
   - In CT: `tailscale up --auth-key=tskey-auth-XXXXXXX --hostname=isaac-sim --ssh`

2. **NGC account + Isaac Sim install**
   - NGC account: https://ngc.nvidia.com/ (gratis voor developers)
   - API key onder Setup > API Key
   - In CT (de pip-route is de schoonste sinds Isaac Sim 4.5):
     ```bash
     pip install --upgrade pip
     pip install isaacsim --extra-index-url https://pypi.nvidia.com
     isaacsim --help    # accepteert EULA + downloadt assets
     ```
   - Voor headless workflow met Martijn's arm:
     ```bash
     isaacsim --headless --enable omni.kit.streaming.webrtc
     ```

3. **(Niet urgent) CT 123 IP-squatting opruimen**
   - CT 123 (dllm-experiment) heeft 150+ secundaire IPs op vmbr0. Dat blokkeert directe SSH naar CT 140 op 10.1.1.130. Niet kritisch zodra Tailscale werkt.

## Niet aangeraakt (per de constraints)

- Geen reboot van pve03
- Geen wijzigingen aan CT 116/118/123/125/128/131/132 of VM 130
- Geen vfio-pci binding, geen aanraking aan `gpu-switch` script
- Geen kernel module install in CT (zou conflicteren met host driver)
