import fs from "node:fs";
import path from "node:path";
import { Client, type ConnectConfig, type ClientChannel } from "ssh2";
import type { AppConfig } from "./config.js";

export type ExecResult = { code: number; stdout: string; stderr: string };

function readPrivateKeyIfSet(p?: string): string | undefined {
  if (!p) return undefined;
  const resolved = p.startsWith("~")
    ? path.join(process.env.HOME ?? "", p.slice(1))
    : p;
  return fs.readFileSync(resolved, "utf8");
}

export function buildSshConfig(cfg: AppConfig): ConnectConfig {
  const privateKey = readPrivateKeyIfSet(cfg.JETSON_PRIVATE_KEY);
  const agent = cfg.SSH_AUTH_SOCK ?? process.env.SSH_AUTH_SOCK;
  return {
    host: cfg.JETSON_HOST,
    port: cfg.JETSON_PORT,
    username: cfg.JETSON_USER,
    privateKey,
    password: cfg.JETSON_PASSWORD,
    agent,
    // Prefer keys/agent; keep it permissive to get you running quickly.
    readyTimeout: 8000,
    keepaliveInterval: 5000,
    keepaliveCountMax: 3
  };
}

export function bashLcInDir(dir: string, cmd: string): string {
  // Use double-quoted bash -lc payload so nested single quotes are safe.
  const qd = (s: string) =>
    `"${s
      .replaceAll("\\", "\\\\")
      .replaceAll('"', '\\"')
      .replaceAll("$", "\\$")
      .replaceAll("`", "\\`")}"`;
  const payload = `cd ${qd(dir)} && ${cmd}`;
  return `bash -lc ${qd(payload)}`;
}

export function execOnce(sshConfig: ConnectConfig, command: string): Promise<ExecResult> {
  return new Promise((resolve, reject) => {
    const conn = new Client();
    let stdout = "";
    let stderr = "";

    conn
      .on("ready", () => {
        conn.exec(command, { pty: false }, (err: Error | undefined, stream: ClientChannel) => {
          if (err) {
            conn.end();
            reject(err);
            return;
          }
          stream
            .on("close", (code: number | null) => {
              conn.end();
              resolve({ code: code ?? 0, stdout, stderr });
            })
            .on("data", (d: Buffer) => {
              stdout += d.toString("utf8");
            });
          stream.stderr.on("data", (d: Buffer) => {
            stderr += d.toString("utf8");
          });
        });
      })
      .on("error", (e: Error) => reject(e))
      .connect(sshConfig);
  });
}

export function execStream(
  sshConfig: ConnectConfig,
  command: string,
  onData: (chunk: string) => void,
  onErr: (chunk: string) => void,
  onClose: (code: number | null) => void,
  opts?: {
    pty?: boolean;
    sendSigintOnStop?: boolean;
    retries?: number;
    retryDelayMs?: number;
  }
): { stop: () => void } {
  let conn: Client | null = null;
  let streamRef: ClientChannel | null = null;
  let closed = false;
  let cancelled = false;
  let attemptsLeft = opts?.retries ?? 0;

  const stop = () => {
    if (closed) return;
    closed = true;
    cancelled = true;
    try {
      if (streamRef && opts?.sendSigintOnStop) {
        // Best-effort Ctrl+C for PTY-backed sessions (like `docker run -it ...`).
        try {
          streamRef.write("\x03");
        } catch {}
      }
    } catch {}
    // Give the remote process a moment to exit cleanly.
    setTimeout(() => {
      try {
        if (streamRef) streamRef.close();
      } catch {}
      try {
        conn?.end();
      } catch {}
    }, opts?.sendSigintOnStop ? 200 : 0);
  };

  const connectOnce = () => {
    if (cancelled) return;
    conn = new Client();

    const maybeRetry = (why: string) => {
      if (cancelled) return false;
      if (attemptsLeft <= 0) return false;
      attemptsLeft -= 1;
      const delay = opts?.retryDelayMs ?? 350;
      onErr(`[SSH] ${why}, retrying in ${delay}ms (${attemptsLeft} left)\n`);
      try {
        conn?.end();
      } catch {}
      setTimeout(connectOnce, delay);
      return true;
    };

    conn.on("ready", () => {
        conn?.exec(
          command,
          { pty: opts?.pty ?? false },
          (err: Error | undefined, stream: ClientChannel) => {
            if (err) {
              onErr(`[SSH exec error] ${String(err)}\n`);
              stop();
              onClose(255);
              return;
            }
            streamRef = stream;
            stream
              .on("close", (code: number | null) => {
                stop();
                onClose(code);
              })
              .on("data", (d: Buffer) => onData(d.toString("utf8")));
            stream.stderr.on("data", (d: Buffer) => onErr(d.toString("utf8")));
          }
        );
      });

    conn.on("end", () => {
      // 'end' can happen for transport drop without a helpful error.
      const retried = maybeRetry("connection ended");
      if (retried) return;
      onErr("[SSH error] connection ended unexpectedly\n");
      stop();
      onClose(255);
    });

    // @types/ssh2 doesn't type Client 'close' cleanly; use any.
    (conn as any).on("close", (hadError: boolean) => {
      if (!cancelled && hadError) {
        const retried = maybeRetry("connection closed (hadError=true)");
        if (retried) return;
        onErr("[SSH error] connection closed unexpectedly\n");
        stop();
        onClose(255);
      }
    });

    conn.on("error", (e: any) => {
        const msg = String(e?.message ?? e);
        const code = String(e?.code ?? "");
        const isReset = code === "ECONNRESET" || msg.includes("ECONNRESET");

        if (!cancelled && isReset) {
          const retried = maybeRetry("ECONNRESET");
          if (retried) return;
        }

        onErr(`[SSH error] ${String(e)}\n`);
        stop();
        onClose(255);
      });

    conn.connect(sshConfig);
  };

  connectOnce();

  return { stop };
}

