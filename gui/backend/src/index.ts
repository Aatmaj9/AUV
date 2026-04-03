import http from "node:http";
import net from "node:net";
import express from "express";
import cors from "cors";
import { WebSocketServer } from "ws";
import { loadConfig } from "./config.js";
import { z } from "zod";
import { bashLcInDir, buildSshConfig, execOnce, execStream } from "./ssh.js";
import { Client } from "ssh2";
import type { ConnectConfig } from "ssh2";
import type { ClientChannel } from "ssh2";

const cfg = loadConfig();
const defaultSshConfig = buildSshConfig(cfg);
let activeTarget: {
  ssh: ConnectConfig;
  auvDir: string;
  label: string;
  password?: string;
  containerName: string | null;
} | null = null;


const app = express();
app.use(express.json());
app.use(
  cors({
    origin: cfg.FRONTEND_ORIGIN,
    credentials: false,
  })
);

function asyncHandler(
  fn: (req: express.Request, res: express.Response) => Promise<void>
): express.RequestHandler {
  return (req, res, next) => {
    fn(req, res).catch(next);
  };
}

const ConnectBody = z.object({
  host: z.string().min(1),
  user: z.string().min(1),
  port: z.number().int().positive().optional(),
  password: z.string().min(1).optional(),
  auvDir: z.string().min(1).optional(),
});

function getTarget() {
  return (
    activeTarget ?? {
      ssh: defaultSshConfig,
      auvDir: cfg.JETSON_AUV_DIR,
      label: `${cfg.JETSON_USER}@${cfg.JETSON_HOST}`,
      password: cfg.JETSON_PASSWORD,
      containerName: cfg.DOCKER_CONTAINER as string | null,
    }
  );
}

function composeFile(t: ReturnType<typeof getTarget>): string {
  return `${t.auvDir}/.devcontainer/docker-compose.yml`;
}

async function detectContainerName(t: ReturnType<typeof getTarget>): Promise<string | null> {
  const cmd = `bash -lc "grep 'container_name:' ${shSingleQuote(composeFile(t))} 2>/dev/null | head -1 | sed 's/.*container_name: *//'"`;
  const r = await execOnce(t.ssh, cmd);
  const name = r.stdout.trim();
  return name || null;
}

async function tcpConnectMs(host: string, port: number, timeoutMs: number): Promise<number | null> {
  const start = Date.now();
  return await new Promise((resolve) => {
    const sock = new net.Socket();
    const done = (ms: number | null) => {
      try {
        sock.destroy();
      } catch {}
      resolve(ms);
    };
    sock.setTimeout(timeoutMs);
    sock.once("connect", () => done(Date.now() - start));
    sock.once("timeout", () => done(null));
    sock.once("error", () => done(null));
    sock.connect(port, host);
  });
}

function shSingleQuote(s: string): string {
  // Safe single-quote for bash: ' -> '"'"'
  return `'${s.replaceAll("'", `'\"'\"'`)}'`;
}

function yamlSingleQuotedString(s: string): string {
  // YAML single-quoted escaping: ' -> ''
  return `'${s.replaceAll("'", "''")}'`;
}

app.get(
  "/api/health",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    res.json({ ok: true, connected: !!activeTarget, target: t.label, auvDir: t.auvDir });
  })
);

app.post(
  "/api/connect",
  asyncHandler(async (req, res) => {
    const body = ConnectBody.parse(req.body ?? {});
    const ssh: ConnectConfig = {
      host: body.host,
      port: body.port ?? 22,
      username: body.user,
      password: body.password,
      readyTimeout: 8000,
      keepaliveInterval: 5000,
      keepaliveCountMax: 3,
    };
    const auvDir = body.auvDir ?? "/home/timi/AUV";

    // Verify SSH works (no side effects).
    const r = await execOnce(ssh, "bash -lc 'echo connected'");
    if (r.code !== 0) {
      res.status(500).json(r);
      return;
    }

    activeTarget = {
      ssh,
      auvDir,
      label: `${body.user}@${body.host}`,
      password: body.password,
      containerName: null,
    };

    // Auto-detect container name from docker-compose.yml
    const detected = await detectContainerName(activeTarget).catch(() => null);
    activeTarget.containerName = detected;

    res.json({ ok: true, code: 0, target: activeTarget.label, auvDir: activeTarget.auvDir, containerName: detected });
  })
);

app.get(
  "/api/docker/status",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    if (!t.containerName) {
      res.json({ running: false, containerName: null, error: "container name not detected" });
      return;
    }
    const cmd = `docker inspect --format '{{.State.Running}}' ${shSingleQuote(t.containerName)} 2>/dev/null || echo false`;
    const r = await execOnce(t.ssh, cmd);
    const running = r.stdout.trim() === "true";
    res.json({ running, containerName: t.containerName });
  })
);

app.post(
  "/api/docker/start",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const cf = composeFile(t);
    const cmd = bashLcInDir(t.auvDir, `docker compose -f ${shSingleQuote(cf)} up -d`);
    const r = await execOnce(t.ssh, cmd);
    if (r.code !== 0) {
      res.status(500).json(r);
      return;
    }
    // Re-detect container name in case it wasn't known before
    if (!t.containerName && activeTarget) {
      const detected = await detectContainerName(t).catch(() => null);
      activeTarget.containerName = detected;
    }
    res.json({ code: 0, stdout: r.stdout, containerName: t.containerName });
  })
);

app.post(
  "/api/docker/stop",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const cf = composeFile(t);
    const cmd = bashLcInDir(t.auvDir, `docker compose -f ${shSingleQuote(cf)} down`);
    const r = await execOnce(t.ssh, cmd);
    res.json(r);
  })
);

app.get(
  "/api/docker/list",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const fmt = '{"name":"{{.Names}}","image":"{{.Image}}","status":"{{.Status}}","id":"{{.ID}}"}';
    const cmd = `docker ps --format '${fmt}'`;
    const r = await execOnce(t.ssh, cmd);
    if (r.code !== 0) {
      res.status(500).json(r);
      return;
    }
    const containers = r.stdout
      .trim()
      .split("\n")
      .filter(Boolean)
      .map((line) => {
        try { return JSON.parse(line); }
        catch { return null; }
      })
      .filter(Boolean);
    res.json({ containers });
  })
);

const KillBody = z.object({ name: z.string().min(1) });

app.post(
  "/api/docker/kill",
  asyncHandler(async (req, res) => {
    const t = getTarget();
    const body = KillBody.parse(req.body ?? {});
    const name = body.name;
    const cmd = `docker kill ${shSingleQuote(name)} 2>/dev/null; docker rm -f ${shSingleQuote(name)} 2>/dev/null; echo done`;
    const r = await execOnce(t.ssh, cmd);
    res.json({ code: 0, name, stdout: r.stdout });
  })
);

app.get(
  "/api/status",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const host = t.ssh.host as string | undefined;
    const port = (t.ssh.port as number | undefined) ?? 22;
    if (!host) {
      res.json({ ok: false, reason: "no-host" });
      return;
    }

    const [sshTcpMs, rosbridgeTcpMs] = await Promise.all([
      tcpConnectMs(host, port, 1200),
      tcpConnectMs(host, 9090, 700),
    ]);

    res.json({
      ok: sshTcpMs !== null,
      target: t.label,
      host,
      sshPort: port,
      sshTcpMs,
      rosbridgeTcpMs,
    });
  })
);

// rosbridge lifecycle controlled by HTTP (toggle start/stop).
// DVL GUI tunnel (local port-forward): localhost:8080 -> (via Jetson SSH) -> 192.168.194.95:80
let dvlTunnel:
  | {
      server: net.Server;
      client: Client;
      localPort: number;
      dstHost: string;
      dstPort: number;
    }
  | null = null;

app.post(
  "/api/rosbridge/start",
  asyncHandler(async (_req, res) => {
    const t = getTarget();

    // Kill any existing rosbridge to avoid "Address already in use"
    const killCmd = dockerRos2(`pkill -f "rosbridge_server|rosbridge_websocket" >/dev/null 2>&1 || true`, t.containerName);
    await execOnce(t.ssh, bashLcInDir(t.auvDir, killCmd)).catch(() => null);

    // Start rosbridge detached so it survives SSH disconnects
    const startCmd = dockerRos2Detached(
      `cd /workspaces/mavlab && bash ./rosbridge.sh`,
      t.containerName
    );
    const r = await execOnce(t.ssh, bashLcInDir(t.auvDir, startCmd));
    if (r.code !== 0) {
      res.status(500).json(r);
      return;
    }

    res.json({ code: 0, stdout: "rosbridge started (detached)" });
  })
);

app.post(
  "/api/dvl/tunnel/start",
  asyncHandler(async (_req, res) => {
    const t = getTarget();

    if (dvlTunnel) {
      res.json({ code: 0, stdout: `tunnel already running on http://localhost:${dvlTunnel.localPort}` });
      return;
    }

    const localPort = 8080;
    const dstHost = "192.168.194.95";
    const dstPort = 80;

    const client = new Client();
    const server = net.createServer((sock) => {
      client.forwardOut(
        // source address/port are informational for the SSH server
        sock.localAddress ?? "127.0.0.1",
        sock.localPort ?? 0,
        dstHost,
        dstPort,
        (err, stream) => {
          if (err) {
            try {
              sock.destroy();
            } catch {}
            return;
          }
          sock.pipe(stream);
          stream.pipe(sock);
          sock.on("close", () => {
            try {
              stream.end();
            } catch {}
          });
          stream.on("close", () => {
            try {
              sock.end();
            } catch {}
          });
        }
      );
    });

    const cleanup = () => {
      if (!dvlTunnel) return;
      const cur = dvlTunnel;
      dvlTunnel = null;
      try {
        cur.server.close();
      } catch {}
      try {
        cur.client.end();
      } catch {}
    };

    client.on("error", cleanup);
    client.on("close", cleanup);
    server.on("error", (e) => {
      cleanup();
      throw e;
    });

    await new Promise<void>((resolve, reject) => {
      client.once("ready", resolve);
      client.once("error", reject);
      client.connect(t.ssh);
    });

    await new Promise<void>((resolve, reject) => {
      server.listen(localPort, "127.0.0.1", () => resolve());
      server.once("error", reject);
    });

    dvlTunnel = { server, client, localPort, dstHost, dstPort };
    res.json({ code: 0, stdout: `tunnel started: http://localhost:${localPort}` });
  })
);

app.post(
  "/api/rosbridge/stop",
  asyncHandler(async (_req, res) => {
    const t = getTarget();

    const killCmd = dockerRos2(`pkill -f "rosbridge_server|rosbridge_websocket" >/dev/null 2>&1 || true`, t.containerName);
    await execOnce(t.ssh, bashLcInDir(t.auvDir, killCmd)).catch(() => null);

    res.json({ code: 0, stdout: "rosbridge stopped" });
  })
);

function withSudoPasswordIfAny(
  t: ReturnType<typeof getTarget>,
  cmdInAuvDir: string
): string {
  // If a password was provided, run the whole command under sudo (non-interactive).
  // This prevents scripts that contain `sudo ...` from trying to prompt for a TTY mid-run.
  const pw = t.password;
  if (!pw) return cmdInAuvDir;
  const pwEsc = String(pw).replaceAll("\\", "\\\\").replaceAll('"', '\\"');
  const wrapped = cmdInAuvDir.replaceAll('"', '\\"');
  // -p '' suppresses the "[sudo] password for ..." prompt noise on stderr.
  return `printf "%s\\n" "${pwEsc}" | sudo -S -p '' bash -lc "${wrapped}"`;
}

function dockerRos2(cmd: string, container: string | null = "auv"): string {
  const c = container ?? "auv";
  const prefix =
    "source /opt/ros/humble/setup.bash" +
    " && if [ -f /home/mavlab/ros2_ws/install/setup.bash ]; then source /home/mavlab/ros2_ws/install/setup.bash; fi" +
    " && if [ -f /workspaces/mavlab/code_ws/install/setup.bash ]; then source /workspaces/mavlab/code_ws/install/setup.bash; fi";
  const inner = `${prefix} && ${cmd}`;
  return `docker exec -u mavlab ${c} bash -lc "${inner.replaceAll('"', '\\"')}"`;
}

function dockerRos2Interactive(cmd: string, container: string | null = "auv"): string {
  const c = container ?? "auv";
  const prefix =
    "source /opt/ros/humble/setup.bash" +
    " && if [ -f /home/mavlab/ros2_ws/install/setup.bash ]; then source /home/mavlab/ros2_ws/install/setup.bash; fi" +
    " && if [ -f /workspaces/mavlab/code_ws/install/setup.bash ]; then source /workspaces/mavlab/code_ws/install/setup.bash; fi";
  const inner = `${prefix} && ${cmd}`;
  return `docker exec -it -u mavlab ${c} bash -lc "${inner.replaceAll('"', '\\"')}"`;
}

app.post(
  "/api/sensors/activate",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const cmd = bashLcInDir(t.auvDir, withSudoPasswordIfAny(t, "./activate_sensors.sh"));
    const r = await execOnce(t.ssh, cmd);
    res.json(r);
  })
);

app.post(
  "/api/sensors/deactivate",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const cmd = bashLcInDir(t.auvDir, withSudoPasswordIfAny(t, "./deactivate_sensors.sh"));
    const r = await execOnce(t.ssh, cmd);
    res.json(r);
  })
);

app.get(
  "/api/devices/usb",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const cmd = bashLcInDir(t.auvDir, "./usb_devices.sh");
    const r = await execOnce(t.ssh, cmd);
    if (r.code !== 0) {
      res.status(500).json(r);
      return;
    }
    const devices = r.stdout
      .split(/\r?\n/g)
      .map((s) => s.trim())
      .filter(Boolean);
    res.json({ code: r.code, devices, stderr: r.stderr });
  })
);

app.post(
  "/api/devices/udev",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const inner = `cd ${shSingleQuote(t.auvDir)} && bash ./udev.sh`;
    let cmd: string;
    if (t.password) {
      const pwEsc = String(t.password).replaceAll("\\", "\\\\").replaceAll("'", "'\"'\"'");
      cmd = `printf '%s\\n' '${pwEsc}' | sudo -S -p '' bash -lc ${shSingleQuote(inner)}`;
    } else {
      cmd = `bash -lc ${shSingleQuote(inner)}`;
    }
    const r = await execOnce(t.ssh, cmd);
    res.json(r);
  })
);

app.get(
  "/api/devices/udev-map",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const rulesFile = `${t.auvDir}/99-usb-serial.rules`;
    const script = [
      `symlinks=$(grep -oP "SYMLINK\\+=\\"\\K[^\\"]+" ${shSingleQuote(rulesFile)} 2>/dev/null)`,
      `for dev in $symlinks; do`,
      `  if [ -e "/dev/$dev" ]; then`,
      `    real=$(readlink -f "/dev/$dev")`,
      `    echo "$dev|$real|active"`,
      `  else`,
      `    echo "$dev||not_found"`,
      `  fi`,
      `done`,
    ].join("\n");
    const cmd = `bash -lc '${script.replaceAll("'", "'\"'\"'")}'`;
    const r = await execOnce(t.ssh, cmd);
    const mappings = r.stdout
      .trim()
      .split("\n")
      .filter(Boolean)
      .map((line) => {
        const [symlink, realDevice, status] = line.split("|");
        return { symlink: `/dev/${symlink}`, realDevice: realDevice || null, status };
      });
    res.json({ mappings });
  })
);

app.get(
  "/api/ros/topics",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const cmd = bashLcInDir(
      t.auvDir,
      dockerRos2("ros2 topic list", t.containerName)
    );
    const r = await execOnce(t.ssh, cmd);
    if (r.code !== 0) {
      res.status(500).json(r);
      return;
    }
    const topics = r.stdout
      .split(/\r?\n/g)
      .map((s) => s.trim())
      .filter(Boolean);
    res.json({ code: r.code, topics, stderr: r.stderr });
  })
);

app.get(
  "/api/ros/nodes",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const cmd = bashLcInDir(t.auvDir, dockerRos2("ros2 node list", t.containerName));
    const r = await execOnce(t.ssh, cmd);
    if (r.code !== 0) {
      res.status(500).json(r);
      return;
    }
    const nodes = r.stdout
      .split(/\r?\n/g)
      .map((s) => s.trim())
      .filter(Boolean);
    res.json({ code: r.code, nodes, stderr: r.stderr });
  })
);

// rosbridge is controlled via WebSocket at `/ws/rosbridge` (start on connection, stop on close).

const ModemSendBody = z.object({
  data: z.string().min(1).max(8),
  topic: z.string().min(1).optional()
});

app.post(
  "/api/modem/send",
  asyncHandler(async (req, res) => {
    const t = getTarget();
    const body = ModemSendBody.parse(req.body ?? {});
    const topic = body.topic ?? "/auv/modem/send_command";

    const dataYaml = yamlSingleQuotedString(body.data);
    const msg = `{data: ${dataYaml}}`;

    const cmd = bashLcInDir(
      t.auvDir,
      dockerRos2(
        `ros2 topic pub --once ${shSingleQuote(topic)} std_msgs/msg/String ${shSingleQuote(msg)}`,
        t.containerName
      )
    );
    const r = await execOnce(t.ssh, cmd);
    res.json(r);
  })
);

// ── Rosbag recording ────────────────────────────────────────────────
let rosbagRecording = false;
let rosbagActiveName: string | null = null;

async function nextRosbagRunName(t: ReturnType<typeof getTarget>): Promise<string> {
  const c = t.containerName ?? "auv";
  // Simple command: list rosbags dir, extract run numbers, find max
  const cmd = `docker exec -u mavlab ${c} ls -1 /workspaces/mavlab/rosbags 2>/dev/null || true`;
  const r = await execOnce(t.ssh, cmd).catch(() => ({ stdout: "" as string }));
  const stdout = String((r as any).stdout ?? "");
  let max = 0;
  for (const line of stdout.split(/\r?\n/)) {
    const m = line.trim().match(/^run(\d+)$/);
    if (m) {
      const n = Number(m[1]);
      if (n > max) max = n;
    }
  }
  return `run${max + 1}`;
}

function dockerRos2Detached(cmd: string, container: string | null = "auv"): string {
  const c = container ?? "auv";
  const prefix =
    "source /opt/ros/humble/setup.bash" +
    " && if [ -f /home/mavlab/ros2_ws/install/setup.bash ]; then source /home/mavlab/ros2_ws/install/setup.bash; fi" +
    " && if [ -f /workspaces/mavlab/code_ws/install/setup.bash ]; then source /workspaces/mavlab/code_ws/install/setup.bash; fi";
  const inner = `${prefix} && ${cmd}`;
  return `docker exec -d -u mavlab ${c} bash -lc "${inner.replaceAll('"', '\\"')}"`;
}

const RosbagStartBody = z.object({
  topics: z.array(z.string().min(1)).min(1),
});

app.post(
  "/api/rosbag/start",
  asyncHandler(async (req, res) => {
    const t = getTarget();
    const body = RosbagStartBody.parse(req.body ?? {});

    if (rosbagRecording) {
      res.status(409).json({ code: 1, stderr: "rosbag recording already running — stop it first" });
      return;
    }

    const mkdirCmd = bashLcInDir(
      t.auvDir,
      dockerRos2("mkdir -p /workspaces/mavlab/rosbags", t.containerName)
    );
    await execOnce(t.ssh, mkdirCmd).catch(() => null);

    const topicList = body.topics.map((tp) => shSingleQuote(tp)).join(" ");
    const bagName = await nextRosbagRunName(t);
    const recordCmd = dockerRos2Detached(
      `ros2 bag record ${topicList} -o /workspaces/mavlab/rosbags/${bagName}`,
      t.containerName
    );
    const remoteCmd = bashLcInDir(t.auvDir, recordCmd);

    const r = await execOnce(t.ssh, remoteCmd);
    if (r.code !== 0) {
      res.status(500).json(r);
      return;
    }

    rosbagRecording = true;
    rosbagActiveName = bagName;
    res.json({ code: 0, stdout: `recording started → rosbags/${bagName}`, bagName });
  })
);

app.post(
  "/api/rosbag/stop",
  asyncHandler(async (_req, res) => {
    const t = getTarget();

    // Send SIGINT for a clean rosbag shutdown inside the container
    const killCmd = dockerRos2(
      `pkill -INT -f "ros2.*bag.*record" >/dev/null 2>&1 || true`,
      t.containerName
    );
    await execOnce(t.ssh, bashLcInDir(t.auvDir, killCmd)).catch(() => null);

    rosbagRecording = false;
    const stopped = rosbagActiveName;
    rosbagActiveName = null;

    res.json({ code: 0, stdout: "recording stopped", bagName: stopped });
  })
);

app.get(
  "/api/rosbag/status",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const cmd = dockerRos2(
      `(pgrep -af "ros2.*bag.*record" 2>/dev/null | grep -vE "pgrep|bash -lc|docker exec" | grep -q . && echo running || echo stopped)`,
      t.containerName
    );
    const r = await execOnce(t.ssh, bashLcInDir(t.auvDir, cmd)).catch(() => ({ stdout: "stopped" }));
    const running = (r.stdout ?? "").trim() === "running";
    rosbagRecording = running;
    if (!running) rosbagActiveName = null;
    res.json({ recording: running, bagName: rosbagActiveName });
  })
);

// ── Previous rosbag runs ─────────────────────────────────────────────
app.get(
  "/api/rosbag/runs",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const c = t.containerName ?? "auv";
    const cmd = `docker exec -u mavlab ${c} ls -1 /workspaces/mavlab/rosbags 2>/dev/null || true`;
    const r = await execOnce(t.ssh, cmd).catch(() => ({ stdout: "" as string }));
    const stdout = String((r as any).stdout ?? "");
    const runs: string[] = [];
    for (const line of stdout.split(/\r?\n/)) {
      const name = line.trim();
      if (name && /^run\d+$/.test(name)) runs.push(name);
    }
    runs.sort((a, b) => {
      const na = Number(a.replace("run", ""));
      const nb = Number(b.replace("run", ""));
      return nb - na;
    });
    res.json({ runs });
  })
);

app.get(
  "/api/rosbag/runs/:name/metadata",
  asyncHandler(async (req, res) => {
    const t = getTarget();
    const c = t.containerName ?? "auv";
    const runName = req.params.name.replace(/[^a-zA-Z0-9_-]/g, "");
    const cmd = `docker exec -u mavlab ${c} cat /workspaces/mavlab/rosbags/${runName}/metadata.yaml 2>&1`;
    const r = await execOnce(t.ssh, cmd);
    if (r.code !== 0) {
      res.status(404).json({ code: r.code, stderr: r.stderr || r.stdout });
      return;
    }
    res.json({ metadata: r.stdout });
  })
);

app.delete(
  "/api/rosbag/runs/:name",
  asyncHandler(async (req, res) => {
    const t = getTarget();
    const c = t.containerName ?? "auv";
    const runName = req.params.name.replace(/[^a-zA-Z0-9_-]/g, "");
    if (!runName) {
      res.status(400).json({ code: 1, stderr: "invalid run name" });
      return;
    }
    const cmd = `docker exec -u mavlab ${c} rm -rf /workspaces/mavlab/rosbags/${runName}`;
    const r = await execOnce(t.ssh, cmd);
    if (r.code !== 0) {
      res.status(500).json(r);
      return;
    }
    res.json({ code: 0, stdout: `deleted ${runName}` });
  })
);

app.use(
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  (err: unknown, _req: express.Request, res: express.Response, _next: express.NextFunction) => {
    const message = err instanceof Error ? err.message : String(err);
    res.status(500).json({ code: 255, stdout: "", stderr: message });
  }
);

const server = http.createServer(app);

// All WebSocket servers use noServer mode and a single upgrade dispatcher
// (ws library requires this when sharing one HTTP server across multiple paths).
const wss = new WebSocketServer({ noServer: true, perMessageDeflate: false });
wss.on("connection", (ws, req) => {
  // eslint-disable-next-line no-console
  console.log(`[ws/echo] connected from ${req.socket.remoteAddress ?? "?"}`);
  const pingId = setInterval(() => {
    try {
      ws.ping();
    } catch {}
  }, 15000);
  ws.on("error", (e) => {
    // eslint-disable-next-line no-console
    console.log(`[ws/echo] ws error: ${String(e)}`);
  });
  ws.on("close", (code, reason) => {
    clearInterval(pingId);
    // eslint-disable-next-line no-console
    console.log(`[ws/echo] ws closed code=${code} reason=${reason.toString()}`);
  });
  const t = getTarget();
  const url = new URL(req.url ?? "", `http://127.0.0.1:${cfg.BACKEND_PORT}`);
  const topic = (url.searchParams.get("topic") ?? "").trim();

  if (!topic) {
    ws.send(JSON.stringify({ type: "error", message: "Missing topic" }));
    ws.close();
    return;
  }

  const topicQ = shSingleQuote(topic);
  const remoteCmd = bashLcInDir(
    t.auvDir,
    dockerRos2(`ros2 topic echo ${topicQ}`, t.containerName)
  );

  ws.send(JSON.stringify({ type: "status", message: `echo started: ${topic}` }));

  const handle = execStream(
    t.ssh,
    remoteCmd,
    (chunk) => {
      if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "out", chunk }));
    },
    (chunk) => {
      // eslint-disable-next-line no-console
      console.log(`[ws/echo] ssh stderr: ${chunk.trim()}`);
      if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "err", chunk }));
    },
    (code) => {
      // eslint-disable-next-line no-console
      console.log(`[ws/echo] ssh done exit=${code ?? 0}`);
      if (ws.readyState === ws.OPEN)
        ws.send(JSON.stringify({ type: "status", message: `echo stopped (exit=${code ?? 0})` }));
      try {
        ws.close();
      } catch {}
    },
    // Use a PTY for echo to behave like an interactive SSH terminal.
    { pty: true, retries: 3, retryDelayMs: 400 }
  );

  ws.on("close", () => handle.stop());
  ws.on("error", () => handle.stop());
});

const mrosWss = new WebSocketServer({ noServer: true, perMessageDeflate: false });
mrosWss.on("connection", (ws) => {
  const t = getTarget();
  const remoteCmd = bashLcInDir(t.auvDir, "./mros.sh");

  ws.send(JSON.stringify({ type: "status", message: "micro-ros agent starting..." }));

  const handle = execStream(
    t.ssh,
    remoteCmd,
    (chunk) => {
      if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "out", chunk }));
    },
    (chunk) => {
      if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "err", chunk }));
    },
    (code) => {
      if (ws.readyState === ws.OPEN)
        ws.send(JSON.stringify({ type: "status", message: `micro-ros agent stopped (exit=${code ?? 0})` }));
      try {
        ws.close();
      } catch {}
    },
    { pty: true, sendSigintOnStop: true, retries: 2, retryDelayMs: 500 }
  );

  ws.on("close", () => handle.stop());
  ws.on("error", () => handle.stop());
});

const termWss = new WebSocketServer({ noServer: true, perMessageDeflate: false });

server.on("upgrade", (request, socket, head) => {
  const { pathname } = new URL(request.url ?? "", `http://127.0.0.1:${cfg.BACKEND_PORT}`);
  if (pathname === "/ws/echo") {
    wss.handleUpgrade(request, socket, head, (ws) => wss.emit("connection", ws, request));
  } else if (pathname === "/ws/mros") {
    mrosWss.handleUpgrade(request, socket, head, (ws) => mrosWss.emit("connection", ws, request));
  } else if (pathname === "/ws/term") {
    termWss.handleUpgrade(request, socket, head, (ws) => termWss.emit("connection", ws, request));
  } else {
    socket.destroy();
  }
});
termWss.on("connection", (ws, req) => {
  // eslint-disable-next-line no-console
  console.log(`[ws/term] new connection from ${req.socket.remoteAddress ?? "?"}`);
  const t = getTarget();
  const url = new URL(req.url ?? "", `http://127.0.0.1:${cfg.BACKEND_PORT}`);
  const mode = (url.searchParams.get("mode") ?? "jetson").trim();
  const isDocker = mode === "docker";

  const conn = new Client();
  let ch: ClientChannel | null = null;
  let rows = 30;
  let cols = 120;
  let closed = false;

  const sendStatus = (message: string) => {
    try {
      if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "status", message }));
    } catch {}
  };

  const cleanup = () => {
    if (closed) return;
    closed = true;
    try {
      ch?.close();
    } catch {}
    ch = null;
    try {
      conn.end();
    } catch {}
    try {
      ws.close();
    } catch {}
  };

  ws.on("close", cleanup);
  ws.on("error", cleanup);

  ws.on("message", (data) => {
    try {
      const msg = JSON.parse(String(data)) as any;
      if (msg?.type === "input" && typeof msg.data === "string") {
        ch?.write(msg.data);
      }
      if (
        msg?.type === "resize" &&
        typeof msg.cols === "number" &&
        typeof msg.rows === "number" &&
        Number.isFinite(msg.cols) &&
        Number.isFinite(msg.rows)
      ) {
        cols = Math.max(20, Math.min(400, Math.floor(msg.cols)));
        rows = Math.max(5, Math.min(200, Math.floor(msg.rows)));
        try {
          ch?.setWindow(rows, cols, 0, 0);
        } catch {}
      }
    } catch {
      // ignore malformed messages
    }
  });

  conn.on("ready", () => {
    sendStatus(isDocker ? "docker terminal connecting..." : "jetson terminal connecting...");

    const pty: any = { term: "xterm-256color", cols, rows };
    if (isDocker) {
      const cname = t.containerName ?? "auv";
      const cmd = `docker exec -it -u mavlab ${cname} bash`;
      conn.exec(cmd, { pty }, (err: Error | undefined, stream: ClientChannel) => {
        if (err) {
          sendStatus(`error: ${String(err)}`);
          cleanup();
          return;
        }
        ch = stream;
        sendStatus("connected");
        stream.on("data", (d: Buffer) => {
          try {
            if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "output", data: d.toString("utf8") }));
          } catch {}
        });
        stream.stderr.on("data", (d: Buffer) => {
          try {
            if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "output", data: d.toString("utf8") }));
          } catch {}
        });
        stream.on("close", cleanup);
      });
    } else {
      conn.shell(pty, (err: Error | undefined, stream: ClientChannel) => {
        if (err) {
          sendStatus(`error: ${String(err)}`);
          cleanup();
          return;
        }
        ch = stream;
        sendStatus("connected");
        stream.on("data", (d: Buffer) => {
          try {
            if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "output", data: d.toString("utf8") }));
          } catch {}
        });
        stream.stderr.on("data", (d: Buffer) => {
          try {
            if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "output", data: d.toString("utf8") }));
          } catch {}
        });
        stream.on("close", cleanup);
      });
    }
  });

  conn.on("error", (e) => {
    sendStatus(`ssh error: ${String(e)}`);
    cleanup();
  });

  conn.on("close", cleanup);
  conn.connect(t.ssh);
});

// rosbridge lifecycle is controlled by HTTP endpoints (`/api/rosbridge/start` and `/api/rosbridge/stop`).

server.listen(cfg.BACKEND_PORT, () => {
  // eslint-disable-next-line no-console
  console.log(`backend listening on http://localhost:${cfg.BACKEND_PORT}`);
});

