## AUV Web GUI (PC browser, Jetson execution over SSH)

This `gui/` folder is a self-contained web GUI with:

- `backend/`: runs on **your PC** and executes commands on the **Jetson** via SSH.
- `frontend/`: runs in your **browser** and talks to the backend (REST + WebSocket).

Your Jetson is accessed as `timi@192.168.1.162`.

---

## How it works

### Buttons (host-side, outside Docker)
The backend SSHes to the Jetson and runs these in the Jetson AUV folder:

- `./activate_sensors.sh`
- `./deactivate_sensors.sh`

### ROS2 topics (inside Docker container)
Your ROS2 nodes run inside the `auv` container (see your scripts). So the backend uses:

- list topics: `docker exec auv bash -ic 'source ~/.bashrc; ros2 topic list'`
- echo topic: `docker exec auv bash -ic 'source ~/.bashrc; ros2 topic echo <topic>'`

Echo output is streamed back to the browser via WebSocket.

---

## Run it on your PC

### 0) Prereqs (PC)
- Node.js 18+ (recommended 20+)
- Ability to SSH to Jetson:

```bash
ssh timi@192.168.1.162
```

### 1) Start backend

```bash
cd gui/backend
npm install
cp .env.example .env
npm run dev
```

Backend listens on `http://localhost:8000`.

### 2) Start frontend

In another terminal:

```bash
cd gui/frontend
npm install
npm run dev
```

Open the URL printed by Vite (usually `http://localhost:5173`).

---

## Configuration

Edit `gui/backend/.env`:
- `JETSON_HOST` (default `192.168.1.162`)
- `JETSON_USER` (default `timi`)
- `JETSON_AUV_DIR` (default `/home/timi/AUV`)
- `JETSON_PORT` (default `22`)

Notes:
- For best UX, set up SSH keys so the backend can connect without password prompts.
- If `activate_sensors.sh` needs `sudo`, you’ll need passwordless sudo for those specific commands or move them into a service.

