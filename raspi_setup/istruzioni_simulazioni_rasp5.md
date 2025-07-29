# 📦 Sistema di bordo - PoliSail

Questo repository contiene tutto il necessario per la gestione dei **Raspberry Pi** usati nel sistema di bordo del progetto *PoliSail*.

## 📍 Panoramica dispositivi

Nel sistema sono presenti **due Raspberry Pi**:

- 🛥️ **Orca** → si trova a bordo **della barca**
- 🚤 **Persefone** → si trova sul **gommone** ed è **collegato direttamente allo schermo Raymarine**

Entrambi i dispositivi hanno installati:

- ✅ Ambiente ROS 2 (Jazzy)
- ✅ Server Web con dashboard
- 🔁 Lo stesso codice, per permettere lo switch in caso di guasti

Tuttavia, **solo Persefone** è configurato per essere riconosciuto da **Raymarine** tramite il nome di rete `polisail.local`.

---

## 👥 Accesso ai Raspberry

| Nome      | Hostname    | Utente     | Password     | SSH                        | Server Web                             |
|-----------|-------------|------------|--------------|----------------------------|----------------------------------------|
| persefone | `polisail`  | `persefone`| `admin`| `ssh persefone@polisail.local` | [http://polisail.local:8080](http://polisail.local:8080) |
| orca      | `orca`      | `orca`     | `admin`| `ssh orca@orca.local`         | [http://orca.local:8080](http://orca.local:8080)         |

---

## 📁 Struttura delle cartelle

Ecco come sono organizzate le directory principali:
```text
/home/
├── Desktop/
│ └── PoliSail/
│    └── gui/
│       └── PoliMiSailGui/ ← WebApp (FE + BE)
│
├── Main-Computer/ ← Repository ROS (col workspace già incluso)
│ ├── install/
│ ├── build/
│ ├── src/
│ ├── rosbags/
│ │ └── rosbag2_2025_04_09-20_15_45/ ← Bag della simulazione
│ └── ...
│
└── Picture
│
└── ......
```

🔄 Inoltre, il file `~/.bashrc` è già configurato per:
- Entrare direttamente in `~/Main-Computer` all'apertura del terminale
- Eseguire il `source` dell’ambiente ROS e del workspace

---

## 🧪 Simulazione della bag

Per simulare i dati registrati, è necessario aprire **5 terminali distinti**. In ciascuno:

1. Collegarsi in SSH con il comando relativo (vedi tabella sopra)
2. Entrare nel workspace:
```bash
   cd ~/Main-Computer
   colcon build
```

3. Eseguire uno di questi comandi:
## 🧵 Terminale 1 – Riproduzione della bag

```bash
   cd ~/Main-Computer/rosbags/rosbag2_2025_04_09-20_15_45
    ros2 bag play .
```

## 🧵 ⚙️ Terminale 2 – IMU Parser

```bash
   ros2 launch can_bus pot_imu_parsers.launch.py
```

## 🧠 Terminale 3 – Sensor Fusion

```bash
   ros2 launch state_estimation sensor_fusion.launch.py
```

## 📤 Terminale 4 – Invio al monitor

```bash
   ros2 run poli_sail_gui send_to_monitor
```

## 🧵 🌐 Terminale 5 – WebSocket per connessione WebApp

```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## 🌍 WebApp (FE + BE)
Per i dettagli su installazione e deploy della Web Application, consulta il README nella repository della WebApp.
