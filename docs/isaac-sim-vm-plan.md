# Isaac Sim op pve03 — Plan en risico-analyse

**Datum:** 2026-05-06
**Status:** PLAN — niet uitgevoerd. Erik moet eerst kiezen tussen Optie A, B of C hieronder.

## TL;DR — Beslissing nodig

De huidige pve03-architectuur deelt beide GPUs via LXC cgroup-passthrough met meerdere actieve workloads (ollama, comfyui, splat-worker, dllm-experiment, dendrobase). Een **VM** met GPU-passthrough vereist VFIO en daarmee het lossen van de `nvidia` host-modules — dat zet de hele lokale AI-stack tijdelijk uit. Bovendien staat in het bestaande `gpu-switch` script gedocumenteerd dat **GPU 04:00 een Blackwell FLR reset-bug heeft met vfio-pci** en dus niet betrouwbaar door te geven is aan een VM.

Drie opties:

| Optie | Voor | Tegen |
|---|---|---|
| **A. LXC container i.p.v. VM** (aanbevolen) | Geen GPU-rebind, geen interruptie van bestaande workloads, GPU shared zoals ollama/comfyui. Snelste pad. | Isaac Sim in LXC is minder gangbaar. Mogelijk extra werk voor Vulkan/X drivers. ROS2 Jazzy wel goed te draaien. |
| **B. VM met GPU 02:00 passthrough** | Volledige isolatie, "schone" Isaac Sim install zoals NVIDIA documenteert. | Vereist `gpu-switch --force vastai` (stopt CT 116 ollama!) of een nieuwe "isaac" mode in het script. Botst met vastai-host VM 130 die dezelfde GPU 02:00 al heeft toegewezen. |
| **C. VM met GPU 04:00 passthrough** | Zou ollama/CT 116 met rust laten | **FLR reset bug** — gedocumenteerd dat dit niet werkt. Zou wel een poging waard kunnen zijn, maar risicovol. |

**Aanbeveling:** Optie A (LXC). Zie aparte sectie "Plan voor Optie A" onderaan.

Als Erik toch persé een VM wil → Optie B met expliciet protocol.

---

## Verkenning — wat ik heb gevonden op pve03

### Hardware en host
- Proxmox VE op kernel `6.17.13-6-pve`
- 2× NVIDIA RTX PRO 6000 Blackwell Max-Q, driver `590.48.01`, CUDA 13.1
  - GPU0: PCI `0000:02:00.0` (+ audio `02:00.1`) — IOMMU group 15
  - GPU1: PCI `0000:04:00.0` (+ audio `04:00.1`) — IOMMU group 17
- Beide IOMMU-groepen zijn schoon (alleen GPU + audio function), passthrough is technisch mogelijk.
- Host kernel cmdline: `intel_iommu=on iommu=pt` — IOMMU is ingeschakeld.
- VFIO-pci modprobe-config bestaat (`/etc/modprobe.d/vfio-pci.conf`), maar `lsmod` toont vfio modules NIET geladen — alleen `nvidia*`.

### Bestaande GPU-gebruikers (LXC, cgroup device passthrough — niet vfio)
- **CT 116 ollama** (running, 252GB RAM, 1TB rootfs) — llama.cpp + whisper-server actief op GPU0 (4 GB).
- **CT 125 comfyui** (running, 32GB RAM) — Flux.1-dev + chatterbox-tts actief op GPU0 (~4 GB).
- **CT 132 splat-worker** (running)
- **CT 123 dllm-experiment** (running)
- **CT 131 dendrobase** (running)

Allemaal mounten ze `/dev/nvidia0`, `/dev/nvidia1`, `/dev/nvidia-uvm` etc. Dat werkt prima zolang de host-`nvidia` driver de GPUs beheert.

### Bestaande VM met GPU-conflict
- **VM 130 vastai-host** (stopped, 200GB RAM, 1064GB disks) heeft `hostpci0: 0000:02:00,pcie=1,x-vga=0`. Niet actief, maar als iemand `gpu-switch --force vastai` draait wordt deze VM gestart en pakt GPU 02:00.

### Het `gpu-switch` script
Op `/usr/local/bin/gpu-switch`, **disabled by default** (vereist `--force`). Beheert het wisselen tussen LOCAL (CT 116 + nvidia) en VASTAI (VM 130 + vfio-pci) modes.

Citaat uit het script:
```
# GPU 02:00 works with vfio-pci passthrough (for VM 130)
# GPU 04:00 has Blackwell FLR reset bug with vfio-pci, stays on nvidia (for CT 116)
```
Dit is essentiële info — alleen GPU 02:00 is "VM-compatible".

### Storage
- `local-lvm` (lvmthin): 7.6 TB totaal, 4.4 TB vrij. Geen probleem voor 200 GB Isaac Sim VM-disk.
- `local` (root fs): **94 GB totaal, 4.6 GB vrij — 95% vol**. Wees voorzichtig met ISO-uploads of grote downloads naar `/var/lib/vz/`. Ubuntu 24.04 cloudimg staat er al (`ubuntu-24.04-cloudimg-amd64.img`, 597 MB).

### Eerstvolgende vrije VMID/CTID
`pvesh get /cluster/nextid` → **119**. Voor leesbaarheid stel ik VMID/CTID **140** voor (binnen het door Erik genoemde 140-149 bereik).

### Tailscale
Host pve03 draait al Tailscale (`100.65.148.27`). Op de Mac waar dit script draait is `op` niet ingelogd, dus geen Tailscale auth-key te halen → **TODO voor Erik**.

---

## Plan voor Optie A (aanbevolen) — LXC container `isaac-sim`

### Resources
- **CTID:** 140
- **Hostname:** isaac-sim
- **Template:** `local:vztmpl/ubuntu-24.04-standard_24.04-2_amd64.tar.zst` (al aanwezig)
- **Cores:** 16
- **RAM:** 64 GB
- **Swap:** 4 GB
- **Rootfs:** 200 GB op `local-lvm`
- **Network:** `vmbr0` met DHCP
- **Features:** `nesting=1, keyctl=1` (nodig voor sommige Python/CUDA workloads)
- **Unprivileged:** ja (zelfde patroon als CT 116)

### GPU-toegang via cgroup (zoals comfyui/ollama)
```
dev0: /dev/nvidia0,gid=0
dev1: /dev/nvidia1,gid=0
dev2: /dev/nvidiactl,gid=0
dev3: /dev/nvidia-uvm,gid=0
dev4: /dev/nvidia-uvm-tools,gid=0
dev5: /dev/nvidia-modeset,gid=0
```

Optioneel maar handig: bind-mount `libcuda.so` van host (zoals CT 116 doet) zodat Python/CUDA binnen de container kan linken zonder volledige driver-install:
```
lxc.mount.entry: /usr/lib/x86_64-linux-gnu/libcuda.so.590.48.01 usr/lib/x86_64-linux-gnu/libcuda.so.590.48.01 none bind,optional,create=file
lxc.mount.entry: /usr/lib/x86_64-linux-gnu/libcuda.so.1 usr/lib/x86_64-linux-gnu/libcuda.so.1 none bind,optional,create=file
```

Of: installeer `nvidia-utils-590` (en alleen userspace, **GEEN kernel module**) inside de CT — dat is de schone manier.

### Stappen
1. SSH naar pve03 als root.
2. Maak CT aan:
   ```bash
   pct create 140 local:vztmpl/ubuntu-24.04-standard_24.04-2_amd64.tar.zst \
     --hostname isaac-sim \
     --cores 16 --memory 65536 --swap 4096 \
     --rootfs local-lvm:200 \
     --net0 name=eth0,bridge=vmbr0,ip=dhcp,firewall=1 \
     --features nesting=1,keyctl=1 \
     --unprivileged 1 \
     --onboot 0 \
     --ostype ubuntu \
     --ssh-public-keys /root/.ssh/authorized_keys \
     --start 0
   ```
3. Voeg GPU device-lines toe aan `/etc/pve/lxc/140.conf` (kopieer van CT 132 splat-worker).
4. Voeg ook `lxc.cap.drop:` aanpassingen toe als nodig — controleer comfyui-conf.
5. `pct start 140` en `pct enter 140`.
6. Binnen de CT:
   ```bash
   apt update && apt install -y vim git curl python3-pip build-essential ca-certificates wget
   # Userspace-only NVIDIA driver:
   apt install -y nvidia-utils-590-server  # of equivalent — match versie host
   nvidia-smi  # moet beide GPUs tonen
   # Tailscale:
   curl -fsSL https://tailscale.com/install.sh | sh
   tailscale up --auth-key=<TODO ERIK>
   ```

### Risico's Optie A
- **Geen impact op bestaande containers** zolang we niet rebooten en niet aan VFIO komen.
- LXC unprivileged + GPU + Vulkan kan rendering-problemen geven voor de Isaac Sim viewer. Workaround: headless draaien, web-streaming via WebRTC (zie `isaac-sim-install.md`).
- Isaac Sim kan in `/tmp` of `/dev/shm` schrijven die in unprivileged LXC beperkt is. Mogelijk moet `lxc.mount.entry` toegevoegd voor extra paths.

---

## Plan voor Optie B (VM met GPU 02:00 passthrough)

### Voorwaarden
- CT 116 (ollama) **moet uit** voor GPU-rebind. Erik moet expliciet OK geven.
- VM 130 (vastai-host) mag niet starten zolang isaac-sim VM draait — beide claimen GPU 02:00.
- Het bestaande `gpu-switch` script moet uitgebreid of vervangen door een "isaac" mode.

### Resources VM 140
- Ubuntu 24.04 cloud-image (al op `local`).
- 24 vCPU, host CPU type, 64 GB RAM, 200 GB op `local-lvm`.
- `hostpci0: 0000:02:00,pcie=1,x-vga=0` (zelfde syntax als VM 130).
- Cloud-init met SSH-key Erik + Tailscale auth-key.

### Stappen (op hoofdlijnen)
1. CT 116 stoppen: `pct shutdown 116`.
2. Comfyui (CT 125) en andere CTs die GPU 02:00 gebruiken stoppen of beperken.
3. Host nvidia-modules lossen + GPU 02:00 binden aan `vfio-pci` (kopieer logica uit gpu-switch script).
4. VM 140 maken op basis van `local:iso/ubuntu-24.04-cloudimg-amd64.img`:
   ```bash
   qm create 140 --name isaac-sim --memory 65536 --cores 24 --cpu host \
     --machine q35 --bios ovmf --numa 1 --balloon 0 \
     --net0 virtio,bridge=vmbr0 \
     --scsihw virtio-scsi-single \
     --serial0 socket --vga serial0 \
     --ostype l26 --agent enabled=1
   qm importdisk 140 /var/lib/vz/template/iso/ubuntu-24.04-cloudimg-amd64.img local-lvm
   qm set 140 --scsi0 local-lvm:vm-140-disk-0,discard=on,iothread=1,size=200G,ssd=1
   qm set 140 --ide2 local-lvm:cloudinit
   qm set 140 --boot order=scsi0
   qm set 140 --efidisk0 local-lvm:0,efitype=4m,pre-enrolled-keys=0,size=4M
   qm set 140 --hostpci0 0000:02:00,pcie=1,x-vga=0
   qm set 140 --ciuser erik --sshkeys ~/.ssh/authorized_keys
   qm set 140 --ipconfig0 ip=dhcp
   ```
5. `qm start 140` en wachten tot SSH werkt.
6. Binnen VM:
   - `apt install -y nvidia-driver-590-server` (matching host) — **echte kernel-driver in VM**, anders dan LXC.
   - Reboot VM, verifieer `nvidia-smi`.
   - Tailscale up met auth-key.

### Risico's Optie B
- **Onderbreking ollama/llama.cpp** (port 8080, OpenAI-compatible) — Erik's centrale LLM-server.
- Comfyui kan blijven draaien als die alleen GPU 04:00 gebruikt — maar in de huidige `nvidia-smi` output gebruikt comfyui GPU0 (02:00). Moet aangepast worden via `CUDA_VISIBLE_DEVICES=1` of CT-conf.
- Als de VM crasht of GPU 02:00 vasthoudt na shutdown — host reboot nodig. Dat valt buiten de constraints van deze taak.
- Geen weg terug zonder service-onderbreking.

---

## Plan voor Optie C (VM met GPU 04:00 passthrough)

Zou theoretisch ollama (gebruikt vooral GPU0=02:00) ongemoeid laten. **MAAR** het script vermeldt expliciet de Blackwell FLR-bug op GPU 04:00. Niet aanbevolen tenzij Erik bewust wil testen of die bug nog actueel is met de huidige driver/kernel-combinatie.

---

## Concrete vraag aan Erik

1. **Optie A, B of C?** Aanbeveling: A.
2. Als A: bevestig dat ik de LXC container 140 mag aanmaken zoals beschreven.
3. Tailscale auth-key delen (1Password reference of nieuwe key vanuit https://login.tailscale.com/admin/settings/keys).
4. Gewenste VRAM-budget: moet Isaac Sim alleen draaien (volledige 96 GB beschikbaar) of fair-share met ollama/comfyui? Voor RL-training met grote scenes is exclusief gebruik gewenst.
