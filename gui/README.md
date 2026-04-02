# AUV Web GUI

Web-based GUI for controlling and monitoring the AUV over SSH. Runs on your PC and connects to the Jetson remotely.

## Setup

```bash
# Install dependencies (one-time)
cd gui/backend  && npm install
cd ../frontend  && npm install
```

## Configuration

Edit `backend/.env` to match your Jetson setup:

| Variable | Description | Default |
|---|---|---|
| `JETSON_HOST` | Jetson IP address | `192.168.1.162` |
| `JETSON_USER` | SSH username | `timi` |
| `JETSON_PORT` | SSH port | `22` |
| `JETSON_AUV_DIR` | Path to AUV folder on Jetson | `/home/timi/AUV` |
| `JETSON_PASSWORD` | SSH password (optional if using SSH keys) | — |
| `JETSON_PRIVATE_KEY` | Path to SSH private key (optional) | — |
| `BACKEND_PORT` | Backend server port | `8000` |
| `FRONTEND_ORIGIN` | Frontend URL (for CORS) | `http://localhost:5173` |

> **Note:** You can also enter the host, user, and password directly in the GUI's Connection tab at runtime — those values override the `.env` defaults.

## Run

Open two terminals:

```bash
# Terminal 1 — backend
cd gui/backend && npm run dev

# Terminal 2 — frontend
cd gui/frontend && npm run dev
```

Then open http://localhost:5173 in your browser.

## Features

- **Connection** — Connect to the Jetson via SSH (host/user/password)
- **Sensors** — Activate/deactivate sensors, list USB devices, ROS topics/nodes, Echo, Plot, Camera, Modem, DVL GUI
- **Rosbridge** — Toggle rosbridge on/off from the top bar
- **Terminals** — 4 interactive SSH terminals (2× Jetson host, 2× Docker container) with resizable split panes
