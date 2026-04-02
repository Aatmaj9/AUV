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

// rosbridge lifecycle controlled by HTTP (toggle start/stop).
let rosbridgeHandle: { stop: () => void } | null = null;

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

    if (rosbridgeHandle) {
      try {
        rosbridgeHandle.stop();
      } catch {}
      rosbridgeHandle = null;
    }

    // Safety: if rosbridge is already running from a previous attempt, stop it first.
    // This keeps the toggle simple and prevents "Address already in use" conflicts.
    const killCmd = dockerRos2(`pkill -f "rosbridge_server|rosbridge_websocket" >/dev/null 2>&1 || true`);
    void execOnce(t.ssh, bashLcInDir(t.auvDir, killCmd)).catch(() => null);

    // Run EXACTLY like your manual terminal test:
    // `cd /workspaces/mavlab && bash ./rosbridge.sh`
    const remoteCmd = dockerRos2Interactive(`cd /workspaces/mavlab && bash ./rosbridge.sh`);

    rosbridgeHandle = execStream(
      t.ssh,
      remoteCmd,
      () => {},
      (chunk) => {
        // eslint-disable-next-line no-console
        console.log(`[rosbridge start stderr] ${chunk.trim()}`);
      },
      () => {
        rosbridgeHandle = null;
      },
      { pty: true, sendSigintOnStop: true, retries: 0 }
    );

    res.json({ code: 0, stdout: "rosbridge start requested" });
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

    if (rosbridgeHandle) {
      try {
        rosbridgeHandle.stop();
      } catch {}
      rosbridgeHandle = null;
    }

    // Safety: ensure port 9090 is closed even if Ctrl+C doesn't fully stop all children.
    const killCmd = dockerRos2(`pkill -f "rosbridge_server|rosbridge_websocket" >/dev/null 2>&1 || true`);
    void execOnce(t.ssh, bashLcInDir(t.auvDir, killCmd)).catch(() => null);

    res.json({ code: 0, stdout: "rosbridge stop requested" });
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

function dockerRos2Interactive(cmd: string): string {
  // Same as dockerRos2, but allocates a TTY (-t). This makes Ctrl+C propagate more reliably
  // to foreground processes started via execStream.
  const prefix =
    "source /opt/ros/humble/setup.bash" +
    " && if [ -f /home/mavlab/ros2_ws/install/setup.bash ]; then source /home/mavlab/ros2_ws/install/setup.bash; fi" +
    " && if [ -f /workspaces/mavlab/code_ws/install/setup.bash ]; then source /workspaces/mavlab/code_ws/install/setup.bash; fi";
  const inner = `${prefix} && ${cmd}`;
  return `docker exec -it -u mavlab auv bash -lc "${inner.replaceAll('"', '\\"')}"`;
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

// ── Rosbag recording ────────────────────────────────────────────────
let rosbagHandle: { stop: () => void } | null = null;
let rosbagRunIndex = 1;

const RosbagStartBody = z.object({
  topics: z.array(z.string().min(1)).min(1),
});

app.post(
  "/api/rosbag/start",
  asyncHandler(async (req, res) => {
    const t = getTarget();
    const body = RosbagStartBody.parse(req.body ?? {});

    if (rosbagHandle) {
      res.status(409).json({ code: 1, stderr: "rosbag recording already running — stop it first" });
      return;
    }

    const mkdirCmd = bashLcInDir(
      t.auvDir,
      dockerRos2("mkdir -p /workspaces/mavlab/rosbags")
    );
    await execOnce(t.ssh, mkdirCmd).catch(() => null);

    const topicList = body.topics.map((tp) => shSingleQuote(tp)).join(" ");
    const bagName = `run${rosbagRunIndex}`;
    const recordCmd = dockerRos2Interactive(
      `ros2 bag record ${topicList} -o /workspaces/mavlab/rosbags/${bagName}`
    );
    const remoteCmd = bashLcInDir(t.auvDir, recordCmd);

    rosbagHandle = execStream(
      t.ssh,
      remoteCmd,
      () => {},
      (chunk) => {
        // eslint-disable-next-line no-console
        console.log(`[rosbag stderr] ${chunk.trim()}`);
      },
      () => {
        rosbagHandle = null;
      },
      { pty: true, sendSigintOnStop: true, retries: 0 }
    );

    res.json({ code: 0, stdout: `recording started → rosbags/${bagName}`, bagName });
  })
);

app.post(
  "/api/rosbag/stop",
  asyncHandler(async (_req, res) => {
    if (!rosbagHandle) {
      res.json({ code: 0, stdout: "no recording running" });
      return;
    }

    try {
      rosbagHandle.stop();
    } catch {}
    rosbagHandle = null;
    rosbagRunIndex += 1;

    res.json({ code: 0, stdout: "recording stopped" });
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
      // Open a shell inside the auv container.
      const cmd = `docker exec -it -u mavlab auv bash`;
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

