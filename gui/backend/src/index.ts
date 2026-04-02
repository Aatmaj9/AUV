import http from "node:http";
import net from "node:net";
import express from "express";
import cors from "cors";
import { WebSocketServer } from "ws";
import { loadConfig } from "./config.js";
import { z } from "zod";
import { bashLcInDir, buildSshConfig, execOnce, execStream } from "./ssh.js";
import type { ConnectConfig } from "ssh2";

const cfg = loadConfig();
const defaultSshConfig = buildSshConfig(cfg);
let activeTarget: {
  ssh: ConnectConfig;
  auvDir: string;
  label: string;
  password?: string;
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
    }
  );
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
    };

    res.json({ ok: true, code: 0, target: activeTarget.label, auvDir: activeTarget.auvDir });
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

function dockerRos2(cmd: string): string {
  // Run as mavlab, and explicitly source ROS + overlays (do not rely on bashrc/login shell behavior).
  // - /opt/ros/humble: base ROS2
  // - /home/mavlab/ros2_ws/install: built into the image (packages/)
  // - /workspaces/mavlab/code_ws/install: mounted workspace (optional)
  const prefix =
    "source /opt/ros/humble/setup.bash" +
    " && if [ -f /home/mavlab/ros2_ws/install/setup.bash ]; then source /home/mavlab/ros2_ws/install/setup.bash; fi" +
    " && if [ -f /workspaces/mavlab/code_ws/install/setup.bash ]; then source /workspaces/mavlab/code_ws/install/setup.bash; fi";
  const inner = `${prefix} && ${cmd}`;
  return `docker exec -u mavlab auv bash -lc "${inner.replaceAll('"', '\\"')}"`;
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

app.get(
  "/api/ros/topics",
  asyncHandler(async (_req, res) => {
    const t = getTarget();
    const cmd = bashLcInDir(
      t.auvDir,
      dockerRos2("ros2 topic list")
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
    const cmd = bashLcInDir(t.auvDir, dockerRos2("ros2 node list"));
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
        `ros2 topic pub --once ${shSingleQuote(topic)} std_msgs/msg/String ${shSingleQuote(msg)}`
      )
    );
    const r = await execOnce(t.ssh, cmd);
    res.json(r);
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

// WebSocket: ws://localhost:8000/ws/echo?topic=/foo
const wss = new WebSocketServer({ server, path: "/ws/echo", perMessageDeflate: false });
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
    dockerRos2(`ros2 topic echo ${topicQ}`)
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

// WebSocket: ws://localhost:8000/ws/mros  (streams ./mros.sh output)
const mrosWss = new WebSocketServer({ server, path: "/ws/mros", perMessageDeflate: false });
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

// WebSocket: ws://localhost:8000/ws/rosbridge  (control rosbridge lifecycle)
let rosbridgeStreamHandle: { stop: () => void } | null = null;
let rosbridgeActiveWs: any = null;

const rosbridgeCtrlWss = new WebSocketServer({ server, path: "/ws/rosbridge", perMessageDeflate: false });
rosbridgeCtrlWss.on("connection", (ws) => {
  const t = getTarget();

  // Ensure only one rosbridge instance runs at a time.
  if (rosbridgeStreamHandle) {
    try {
      rosbridgeStreamHandle.stop();
    } catch {}
    rosbridgeStreamHandle = null;
  }
  if (rosbridgeActiveWs && rosbridgeActiveWs.readyState === ws.OPEN) {
    try {
      rosbridgeActiveWs.close();
    } catch {}
  }
  rosbridgeActiveWs = ws;

  const remoteCmd = bashLcInDir(
    t.auvDir,
    dockerRos2(
      // Start in the foreground (no nohup) so Ctrl+C can stop it.
      // Also kill any stale rosbridge before launching.
      `pkill -f "rosbridge_server|rosbridge_websocket" >/dev/null 2>&1 || true; ` +
        `cd /workspaces/mavlab && bash ./rosbridge.sh`
    )
  );

  ws.send(JSON.stringify({ type: "status", message: "rosbridge starting..." }));

  const handle = execStream(
    t.ssh,
    remoteCmd,
    // We intentionally keep the UI clean; the toggle only needs lifecycle.
    () => {},
    (chunk) => {
      // eslint-disable-next-line no-console
      console.log(`[ws/rosbridge] ssh stderr: ${chunk.trim()}`);
    },
    (code) => {
      if (ws.readyState === ws.OPEN) {
        ws.send(JSON.stringify({ type: "status", message: `rosbridge stopped (exit=${code ?? 0})` }));
        try {
          ws.close();
        } catch {}
      }
      rosbridgeStreamHandle = null;
      rosbridgeActiveWs = null;
    },
    { pty: true, sendSigintOnStop: true, retries: 0 }
  );

  rosbridgeStreamHandle = handle;

  ws.on("close", () => {
    try {
      handle.stop();
    } catch {}
    if (rosbridgeStreamHandle === handle) rosbridgeStreamHandle = null;
    if (rosbridgeActiveWs === ws) rosbridgeActiveWs = null;
  });
  ws.on("error", () => {
    try {
      handle.stop();
    } catch {}
    if (rosbridgeStreamHandle === handle) rosbridgeStreamHandle = null;
    if (rosbridgeActiveWs === ws) rosbridgeActiveWs = null;
  });
});

server.listen(cfg.BACKEND_PORT, () => {
  // eslint-disable-next-line no-console
  console.log(`backend listening on http://localhost:${cfg.BACKEND_PORT}`);
});

