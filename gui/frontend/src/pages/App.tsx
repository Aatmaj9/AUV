import { useEffect, useMemo, useRef, useState } from "react";
import {
  AppBar,
  Autocomplete,
  Box,
  Button,
  Divider,
  Paper,
  Tab,
  Tabs,
  TextField,
  Toolbar,
  Typography,
  Chip,
  Alert,
  Tooltip,
  Switch,
  Link,
} from "@mui/material";
// roslib has no bundled TS types in this setup
import ROSLIB from "roslib";
import { TerminalSplitPane } from "../components/TerminalSplitPane";

type TopicsResponse =
  | { code: number; topics: string[]; stderr?: string }
  | { code: number; stdout: string; stderr: string };

type NodesResponse =
  | { code: number; nodes: string[]; stderr?: string }
  | { code: number; stdout: string; stderr: string };

type UsbDevicesResponse =
  | { code: number; devices: string[]; stderr?: string }
  | { code: number; stdout: string; stderr: string };

function useBackendBaseWsUrl(): string {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  // Firefox can fail ws://localhost:8000 while HTTP works; use IPv4 loopback for WS only.
  const host =
    window.location.hostname === "localhost" ? "127.0.0.1" : window.location.hostname;
  return `${proto}://${host}:8000`;
}

function useBackendBaseHttpUrl(): string {
  const proto = window.location.protocol === "https:" ? "https" : "http";
  return `${proto}://${window.location.hostname}:8000`;
}

export default function App() {
  const [topTab, setTopTab] = useState(0);
  const [leftTab, setLeftTab] = useState(0);
  const [rightTab, setRightTab] = useState(0);
  const [termsMounted, setTermsMounted] = useState(false);

  useEffect(() => {
    if (topTab === 2 && !termsMounted) setTermsMounted(true);
  }, [topTab, termsMounted]);

  const [busy, setBusy] = useState<string | null>(null);
  const [connected, setConnected] = useState(false);
  const [connectedLabel, setConnectedLabel] = useState<string>("not connected");
  const [status, setStatus] = useState<{
    ok: boolean;
    sshTcpMs: number | null;
    rosbridgeTcpMs: number | null;
  } | null>(null);
  const [sensorState, setSensorState] = useState<"unknown" | "activated" | "deactivated">(
    "unknown"
  );

  const [jetsonUser, setJetsonUser] = useState("timi");
  const [jetsonHost, setJetsonHost] = useState("192.168.1.162");
  const [jetsonPort, setJetsonPort] = useState("22");
  const [jetsonPassword, setJetsonPassword] = useState("");
  const [jetsonAuvDir, setJetsonAuvDir] = useState("/home/timi/AUV");

  const [topics, setTopics] = useState<string[]>([]);
  const [nodes, setNodes] = useState<string[]>([]);
  const [usbDevices, setUsbDevices] = useState<string[]>([]);
  const [mrosText, setMrosText] = useState<string>("");
  const mrosWsRef = useRef<WebSocket | null>(null);
  const [topicFilter, setTopicFilter] = useState("");
  const [echoTopic, setEchoTopic] = useState<string>("");
  const [plotTopic, setPlotTopic] = useState<string>("");
  const [plotHz, setPlotHz] = useState<number[]>([]);
  const [plotTs, setPlotTs] = useState<number[]>([]);

  const [rosbridgeOn, setRosbridgeOn] = useState<boolean>(false);
  // rosbridge toggle is controlled via HTTP endpoints.

  const [camTopic, setCamTopic] = useState<string>("");
  const [camImgUrl, setCamImgUrl] = useState<string>("");
  const [camErr, setCamErr] = useState<string>("");

  const [modemText, setModemText] = useState<string>("");
  const [modemTopics, setModemTopics] = useState<string[]>([]);
  const [modemTab, setModemTab] = useState(0);
  const [modemLogs, setModemLogs] = useState<Record<string, string>>({});
  const [dvlUrl, setDvlUrl] = useState<string>("");
  const modemSubRef = useRef<Record<string, any>>({});

  const [echoText, setEchoText] = useState<string>("");
  const [logText, setLogText] = useState<string>("");
  const [bottomTab, setBottomTab] = useState(0);
  const [logFrac, setLogFrac] = useState(0.25);
  const rightSplitRef = useRef<HTMLDivElement | null>(null);
  const draggingRef = useRef(false);

  const wsBase = useBackendBaseWsUrl();
  const httpBase = useBackendBaseHttpUrl();
  const wsRef = useRef<WebSocket | null>(null);
  const plotWsRef = useRef<WebSocket | null>(null);
  const rosRef = useRef<any | null>(null);
  const camSubRef = useRef<any | null>(null);
  const teleRosRef = useRef<any | null>(null);
  const echoSubRef = useRef<any | null>(null);
  const plotSubRef = useRef<any | null>(null);
  const rosbridgeReachableRef = useRef<boolean | null>(null);

  const filteredTopics = useMemo(() => {
    const q = topicFilter.trim();
    if (!q) return topics;
    return topics.filter((t) => t.includes(q));
  }, [topics, topicFilter]);

  function appendEcho(s: string) {
    setEchoText((prev) => {
      const next = prev + s;
      // keep last ~200k chars
      return next.length > 200_000 ? next.slice(next.length - 200_000) : next;
    });
  }

  function appendLog(s: string) {
    setLogText((prev) => {
      const next = prev + s;
      return next.length > 200_000 ? next.slice(next.length - 200_000) : next;
    });
  }

  function appendMros(s: string) {
    setMrosText((prev) => {
      const next = prev + s;
      return next.length > 200_000 ? next.slice(next.length - 200_000) : next;
    });
  }

  function stopMros() {
    const ws = mrosWsRef.current;
    mrosWsRef.current = null;
    if (ws) ws.close();
  }

  function startMros() {
    stopMros();
    setMrosText("");
    const ws = new WebSocket(`${wsBase}/ws/mros`);
    mrosWsRef.current = ws;
    ws.onmessage = (ev) => {
      try {
        const msg = JSON.parse(ev.data as string) as { type: string; chunk?: string; message?: string };
        if (msg.type === "status" && msg.message) appendMros(`[status] ${msg.message}\n`);
        if (msg.type === "out" && msg.chunk) appendMros(msg.chunk);
        if (msg.type === "err" && msg.chunk) appendMros(`[stderr] ${msg.chunk}`);
      } catch {
        appendMros(String(ev.data));
      }
    };
    ws.onclose = () => appendMros("\n[status] connection closed\n");
    ws.onerror = () => appendMros("\n[status] websocket error\n");
  }

  function getTeleRos(): any | null {
    const host = jetsonHost.trim();
    if (!host) {
      appendLog("[rosbridge] missing jetson host\n");
      return null;
    }
    if (teleRosRef.current) return teleRosRef.current;

    const ros = new ROSLIB.Ros({ url: `ws://${host}:9090` });
    teleRosRef.current = ros;
    ros.on("connection", () => {
      // Connection success will naturally allow echo/plot/cameras to work.
    });
    ros.on("error", () => {
      appendLog("[rosbridge] connection error (is rosbridge running? port 9090 reachable?)\n");
    });
    ros.on("close", () => {
      // allow reconnect on next attempt
      teleRosRef.current = null;
      appendLog("[rosbridge] connection closed\n");
    });
    return ros;
  }

  async function getTopicTypeViaRosapi(ros: any, topic: string): Promise<string> {
    return await new Promise((resolve, reject) => {
      try {
        const svc = new ROSLIB.Service({
          ros,
          name: "/rosapi/topic_type",
          serviceType: "rosapi_msgs/srv/TopicType"
        });
        const req = new ROSLIB.ServiceRequest({ topic });
        svc.callService(req, (resp: any) => {
          const t = String(resp?.type ?? "");
          if (!t) reject(new Error("topic type not found"));
          else resolve(t);
        });
      } catch (e) {
        reject(e);
      }
    });
  }

  function onStartDrag() {
    draggingRef.current = true;
  }

  useEffect(() => {
    const onMove = (e: MouseEvent) => {
      if (!draggingRef.current) return;
      const el = rightSplitRef.current;
      if (!el) return;
      const r = el.getBoundingClientRect();
      const y = e.clientY - r.top;
      const frac = 1 - y / r.height;
      const clamped = Math.max(0.25, Math.min(0.75, frac));
      setLogFrac(clamped);
    };
    const onUp = () => {
      draggingRef.current = false;
    };
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
    return () => {
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
    };
  }, []);

  async function connect() {
    setBusy("/api/connect");
    try {
      const r = await fetch(`${httpBase}/api/connect`, {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({
          host: jetsonHost.trim(),
          user: jetsonUser.trim(),
          port: Number(jetsonPort || "22"),
          password: jetsonPassword,
          auvDir: jetsonAuvDir.trim()
        })
      });
      const j = (await r.json()) as { ok?: boolean; stderr?: string; target?: string };
      if (!r.ok || !j.ok) {
        setConnected(false);
        setConnectedLabel("not connected");
        appendLog(`[connect] failed\n${j.stderr ?? "unknown error"}\n\n`);
        return;
      }
      setConnected(true);
      setConnectedLabel(j.target ?? "connected");
      appendLog(`[connect] ok: ${j.target ?? ""}\n\n`);
    } catch (e) {
      setConnected(false);
      setConnectedLabel("not connected");
      appendLog(`[connect] error: ${String(e)}\n\n`);
    } finally {
      setBusy(null);
    }
  }

  async function ensureBackendConnected(): Promise<boolean> {
    try {
      const r = await fetch(`${httpBase}/api/health`);
      const j = (await r.json().catch(() => null)) as null | { connected?: boolean; target?: string };
      const ok = !!j?.connected;
      if (ok) return true;
    } catch {
      // fall through to reconnect attempt
    }

    // Backend lost state (restart) or never connected; re-run connect using current form values.
    await connect();
    try {
      const r2 = await fetch(`${httpBase}/api/health`);
      const j2 = (await r2.json().catch(() => null)) as null | { connected?: boolean };
      return !!j2?.connected;
    } catch {
      return false;
    }
  }

  async function postJson(path: string) {
    setBusy(path);
    try {
      const r = await fetch(`${httpBase}${path}`, { method: "POST" });
      const j = (await r.json()) as { code: number; stdout?: string; stderr?: string };
      appendLog(`[${path}] exit=${j.code}\n`);
      if (j.stdout) appendLog(j.stdout);
      if (j.stderr) appendLog(`\n[stderr]\n${j.stderr}\n`);
      appendLog("\n");

      if (path === "/api/sensors/activate" || path === "/api/sensors/deactivate") {
        void refreshTopics();
        void refreshNodes();
        if (j.code === 0) {
          setSensorState(path === "/api/sensors/activate" ? "activated" : "deactivated");
        }
      }
    } catch (e) {
      appendEcho(`[${path}] error: ${String(e)}\n\n`);
    } finally {
      setBusy(null);
    }
  }

  async function refreshTopics() {
    setBusy("/api/ros/topics");
    try {
      const r = await fetch(`${httpBase}/api/ros/topics`);
      const j = (await r.json()) as TopicsResponse;
      if ("topics" in j) {
        setTopics(j.topics);
      } else {
        appendLog(`[topics] failed (exit=${j.code})\n${j.stderr}\n\n`);
      }
    } catch (e) {
      appendLog(`[topics] error: ${String(e)}\n\n`);
    } finally {
      setBusy(null);
    }
  }

  async function refreshNodes() {
    setBusy("/api/ros/nodes");
    try {
      const r = await fetch(`${httpBase}/api/ros/nodes`);
      const j = (await r.json()) as NodesResponse;
      if ("nodes" in j) {
        setNodes(j.nodes);
      } else {
        appendLog(`[nodes] failed (exit=${j.code})\n${j.stderr}\n\n`);
      }
    } catch (e) {
      appendLog(`[nodes] error: ${String(e)}\n\n`);
    } finally {
      setBusy(null);
    }
  }

  async function refreshUsbDevices() {
    setBusy("/api/devices/usb");
    try {
      const r = await fetch(`${httpBase}/api/devices/usb`);
      const j = (await r.json()) as UsbDevicesResponse;
      if ("devices" in j) {
        setUsbDevices(j.devices);
      } else {
        appendLog(`[devices] failed (exit=${j.code})\n${j.stderr}\n\n`);
      }
    } catch (e) {
      appendLog(`[devices] error: ${String(e)}\n\n`);
    } finally {
      setBusy(null);
    }
  }

  function stopEcho() {
    const ws = wsRef.current;
    wsRef.current = null;
    if (ws) ws.close();
    try {
      echoSubRef.current?.unsubscribe();
    } catch {}
    echoSubRef.current = null;
  }

  async function startEcho() {
    const topic = echoTopic.trim();
    if (!topic) {
      appendEcho("[echo] pick a topic first\n\n");
      return;
    }

    stopEcho();
    setEchoText("");
    appendEcho(`[echo] subscribing via rosbridge: ${topic}\n`);

    const ros = getTeleRos();
    if (!ros) return;

    try {
      const messageType = await getTopicTypeViaRosapi(ros, topic);
      appendEcho(`[echo] type: ${messageType}\n`);
      const sub = new ROSLIB.Topic({ ros, name: topic, messageType });
      echoSubRef.current = sub;
      sub.subscribe((msg: any) => {
        try {
          appendEcho(JSON.stringify(msg) + "\n");
        } catch {
          appendEcho(String(msg) + "\n");
        }
      });
    } catch (e) {
      appendEcho(`[echo] error: ${String(e)}\n`);
    }
  }

  function stopPlot() {
    const ws = plotWsRef.current;
    plotWsRef.current = null;
    if (ws) ws.close();
    try {
      plotSubRef.current?.unsubscribe();
    } catch {}
    plotSubRef.current = null;
  }

  async function startPlot() {
    const topic = plotTopic.trim();
    if (!topic) {
      appendLog("[plot] pick a topic first\n\n");
      return;
    }
    stopPlot();
    setPlotHz([]);
    setPlotTs([]);

    const ros = getTeleRos();
    if (!ros) return;

    let count = 0;
    const t0 = Date.now();
    const timer = window.setInterval(() => {
      const now = Date.now();
      const dt = (now - t0) / 1000;
      if (dt <= 0) return;
      const hz = count; // per 1s bucket
      count = 0;
      setPlotTs((p) => {
        const next = [...p, now];
        return next.length > 120 ? next.slice(-120) : next;
      });
      setPlotHz((p) => {
        const next = [...p, hz];
        return next.length > 120 ? next.slice(-120) : next;
      });
    }, 1000);

    try {
      const messageType = await getTopicTypeViaRosapi(ros, topic);
      const sub = new ROSLIB.Topic({ ros, name: topic, messageType });
      plotSubRef.current = sub;
      sub.subscribe(() => {
        count += 1;
      });
    } catch (e) {
      window.clearInterval(timer);
      appendLog(`[plot] error: ${String(e)}\n`);
    }
  }

  function startRosbridgeCtrl() {
    void (async () => {
      const ok = await ensureBackendConnected();
      if (!ok) {
        setRosbridgeOn(false);
        return;
      }
      try {
        setBusy("/api/rosbridge/start");
        await fetch(`${httpBase}/api/rosbridge/start`, { method: "POST" });
      } catch {}
      finally {
        setBusy(null);
      }
    })();
  }

  function stopRosbridgeCtrl() {
    void (async () => {
      try {
        setBusy("/api/rosbridge/stop");
        await fetch(`${httpBase}/api/rosbridge/stop`, { method: "POST" });
      } catch {}
      finally {
        setBusy(null);
      }
    })();
  }

  function stopCamera() {
    camSubRef.current?.unsubscribe();
    camSubRef.current = null;
    rosRef.current?.close();
    rosRef.current = null;
    if (camImgUrl) URL.revokeObjectURL(camImgUrl);
    setCamImgUrl("");
  }

  function startCamera() {
    const topic = camTopic.trim();
    if (!topic) {
      setCamErr("Pick a camera topic first.");
      return;
    }
    setCamErr("");
    stopCamera();

    const ros = new ROSLIB.Ros({
      url: `ws://${jetsonHost.trim()}:9090`
    });
    rosRef.current = ros;

    ros.on("connection", () => {
      // If we can connect, the camera subscription will start receiving frames.
    });
    ros.on("error", () => {
      setCamErr("Rosbridge connection failed. Start rosbridge and ensure port 9090 reachable.");
    });
    ros.on("close", () => {
      // ignore
    });

    const sub = new ROSLIB.Topic({
      ros,
      name: topic,
      messageType: "sensor_msgs/msg/CompressedImage"
    });
    camSubRef.current = sub;
    sub.subscribe((msg: any) => {
      try {
        const b64 = msg.data as string;
        const mime = msg.format && String(msg.format).includes("png") ? "image/png" : "image/jpeg";
        const byteChars = atob(b64);
        const bytes = new Uint8Array(byteChars.length);
        for (let i = 0; i < byteChars.length; i++) bytes[i] = byteChars.charCodeAt(i);
        const blob = new Blob([bytes], { type: mime });
        const url = URL.createObjectURL(blob);
        setCamImgUrl((prev) => {
          if (prev) URL.revokeObjectURL(prev);
          return url;
        });
      } catch (e) {
        setCamErr(`Decode error: ${String(e)}`);
      }
    });
  }

  function stopModemEchos() {
    const m = modemSubRef.current;
    for (const k of Object.keys(m)) {
      try {
        m[k].unsubscribe();
      } catch {}
    }
    modemSubRef.current = {};
  }

  async function ensureModemEchos(nextTopics: string[]) {
    const desired = nextTopics.slice(0, 4);
    setModemTopics(desired);
    setModemTab((t) => (t >= desired.length ? 0 : t));

    // close removed
    for (const existing of Object.keys(modemSubRef.current)) {
      if (!desired.includes(existing)) {
        try {
          modemSubRef.current[existing].unsubscribe();
        } catch {}
        delete modemSubRef.current[existing];
      }
    }

    const ros = getTeleRos();
    if (!ros) return;

    for (const topic of desired) {
      if (modemSubRef.current[topic]) continue;
      setModemLogs((prev) => ({ ...prev, [topic]: prev[topic] ?? "" }));

      try {
        const messageType = await getTopicTypeViaRosapi(ros, topic);
        const sub = new ROSLIB.Topic({ ros, name: topic, messageType });
        modemSubRef.current[topic] = sub;
        sub.subscribe((msg: any) => {
          setModemLogs((prev) => {
            const cur = prev[topic] ?? "";
            const line = (() => {
              try {
                return JSON.stringify(msg);
              } catch {
                return String(msg);
              }
            })();
            const next = (cur + line + "\n").slice(-80_000);
            return { ...prev, [topic]: next };
          });
        });
      } catch (e) {
        appendLog(`[modem] subscribe error for ${topic}: ${String(e)}\n`);
      }
    }
  }

  async function sendModem() {
    const data = modemText.trim();
    if (!data) {
      appendLog("[modem] enter up to 8 characters\n\n");
      return;
    }
    setBusy("/api/modem/send");
    try {
      const r = await fetch(`${httpBase}/api/modem/send`, {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({ data })
      });
      const j = (await r.json()) as { code: number; stderr?: string; stdout?: string };
      appendLog(`[/api/modem/send] exit=${j.code}\n`);
      if (j.stdout) appendLog(j.stdout);
      if (j.stderr) appendLog(`\n[stderr]\n${j.stderr}\n`);
      appendLog("\n");

      // refresh & keep modem echos running
      await refreshTopics();
      const available = topics.filter((t) => t.includes("/modem/"));
      void ensureModemEchos(available);
    } catch (e) {
      appendLog(`[/api/modem/send] error: ${String(e)}\n\n`);
    } finally {
      setBusy(null);
    }
  }

  async function startDvlTunnel() {
    setBusy("/api/dvl/tunnel/start");
    try {
      const r = await fetch(`${httpBase}/api/dvl/tunnel/start`, { method: "POST" });
      const j = (await r.json()) as { code: number; stdout?: string; stderr?: string };
      if (j.code !== 0) {
        appendLog(`[/api/dvl/tunnel/start] exit=${j.code}\n${j.stderr ?? ""}\n\n`);
        return;
      }
      const url = "http://localhost:8080";
      setDvlUrl(url);
      appendLog("[dvl] tunnel ready: http://localhost:8080\n\n");
    } catch (e) {
      appendLog(`[/api/dvl/tunnel/start] error: ${String(e)}\n\n`);
    } finally {
      setBusy(null);
    }
  }

  useEffect(() => {
    const id = window.setInterval(async () => {
      try {
        const r = await fetch(`${httpBase}/api/status`);
        const j = (await r.json()) as {
          ok?: boolean;
          sshTcpMs?: number | null;
          rosbridgeTcpMs?: number | null;
        };
        if (typeof j.ok === "boolean") {
          setStatus({
            ok: j.ok,
            sshTcpMs: j.sshTcpMs ?? null,
            rosbridgeTcpMs: j.rosbridgeTcpMs ?? null
          });
        }
      } catch {
        setStatus(null);
      }
    }, 2000);
    return () => {
      window.clearInterval(id);
      stopEcho();
      stopPlot();
      stopCamera();
      stopModemEchos();
      stopMros();
      stopRosbridgeCtrl();
      setRosbridgeOn(false);
      try {
        teleRosRef.current?.close();
      } catch {}
      teleRosRef.current = null;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => {
    const reachable = status?.rosbridgeTcpMs !== null && status?.rosbridgeTcpMs !== undefined;
    const prev = rosbridgeReachableRef.current;
    if (prev === null) {
      rosbridgeReachableRef.current = reachable;
      return;
    }
    if (!prev && reachable) {
      appendLog("[rosbridge] connected\n\n");
    } else if (prev && !reachable) {
      appendLog("[rosbridge] disconnected\n\n");
    }
    rosbridgeReachableRef.current = reachable;
  }, [status?.rosbridgeTcpMs]);

  return (
    <Box sx={{ height: "100vh", display: "flex", flexDirection: "column" }}>
      <AppBar position="static">
        <Toolbar sx={{ gap: 2 }}>
          <Typography variant="h6" sx={{ flexGrow: 1 }}>
            AUV Web GUI
          </Typography>
          <Box
            sx={{
              display: "flex",
              alignItems: "center",
              gap: 1,
              px: 1.25,
              py: 0.5,
              borderRadius: 2,
              border: "1px solid",
              borderColor: "divider",
              bgcolor: "rgba(255,255,255,0.04)",
            }}
          >
            <Typography variant="body2" sx={{ fontWeight: 700, letterSpacing: 0.6 }}>
              ROSBRIDGE
            </Typography>
            <Switch
              checked={rosbridgeOn}
              onChange={(_e, v) => {
                void (async () => {
                  setRosbridgeOn(v);
                  if (v) {
                    const ok = await ensureBackendConnected();
                    if (!ok) {
                      setRosbridgeOn(false);
                      return;
                    }
                    startRosbridgeCtrl();
                  } else {
                    stopRosbridgeCtrl();
                  }
                })();
              }}
              sx={{
                "& .MuiSwitch-switchBase": { color: "error.main" },
                "& .MuiSwitch-track": { backgroundColor: "error.main", opacity: 1 },
                "& .MuiSwitch-switchBase.Mui-checked": {
                  color:
                    status?.rosbridgeTcpMs !== null &&
                    status?.rosbridgeTcpMs !== undefined &&
                    status.rosbridgeTcpMs !== null
                      ? "success.main"
                      : "error.main"
                },
                "& .MuiSwitch-switchBase.Mui-checked + .MuiSwitch-track": {
                  backgroundColor:
                    status?.rosbridgeTcpMs !== null &&
                    status?.rosbridgeTcpMs !== undefined &&
                    status.rosbridgeTcpMs !== null
                      ? "success.main"
                      : "error.main",
                  opacity: 1
                }
              }}
            />
          </Box>
          <Chip
            size="small"
            color={connected ? "success" : "default"}
            label={connected ? `Connected: ${connectedLabel}` : "Not connected"}
            variant={connected ? "filled" : "outlined"}
          />
          <Tooltip
            title={
              status
                ? `SSH TCP: ${status.sshTcpMs ?? "—"} ms`
                : "No status yet"
            }
          >
            <Chip
              size="small"
              color={
                !status
                  ? "default"
                  : !status.ok
                  ? "error"
                  : status.sshTcpMs !== null && status.sshTcpMs < 80
                  ? "success"
                  : status.sshTcpMs !== null && status.sshTcpMs < 200
                  ? "warning"
                  : "error"
              }
              variant="outlined"
              label={
                !status
                  ? "Link: —"
                  : !status.ok
                  ? "Link: down"
                  : `Link: ${status.sshTcpMs ?? "—"} ms`
              }
            />
          </Tooltip>
        </Toolbar>
        <Tabs value={topTab} onChange={(_e, v) => setTopTab(v)} textColor="inherit">
          <Tab label="Connection" />
          <Tab label="SENSORS" />
          <Tab icon={<img src="/terminator-icon.png" alt="" style={{ width: 20, height: 20 }} />} iconPosition="start" label="TERMINALS" sx={{ ml: "auto", minHeight: 0 }} />
        </Tabs>
      </AppBar>

      {topTab === 0 && (
        <Box sx={{ p: 2 }}>
          <Paper sx={{ p: 2, display: "flex", flexDirection: "column", gap: 2, maxWidth: 900 }}>
            <Typography variant="h6">Connect to Jetson</Typography>
            <Box sx={{ display: "flex", gap: 1 }}>
              <TextField
                size="small"
                label="User"
                value={jetsonUser}
                onChange={(e) => setJetsonUser(e.target.value)}
              />
              <TextField
                size="small"
                label="IP / Host"
                value={jetsonHost}
                onChange={(e) => setJetsonHost(e.target.value)}
                fullWidth
              />
              <TextField
                size="small"
                label="Port"
                value={jetsonPort}
                onChange={(e) => setJetsonPort(e.target.value)}
                sx={{ width: 120 }}
              />
            </Box>
            <Box sx={{ display: "flex", gap: 1 }}>
              <TextField
                size="small"
                label="Password"
                type="password"
                value={jetsonPassword}
                onChange={(e) => setJetsonPassword(e.target.value)}
                fullWidth
              />
              <TextField
                size="small"
                label="Jetson AUV dir"
                value={jetsonAuvDir}
                onChange={(e) => setJetsonAuvDir(e.target.value)}
                fullWidth
              />
            </Box>
            <Box sx={{ display: "flex", gap: 1 }}>
              <Button variant="contained" disabled={!!busy} onClick={connect}>
                Connect
              </Button>
            </Box>
            <Typography variant="body2" sx={{ opacity: 0.8 }}>
              After you see “Connected”, go to the SENSORS tab.
            </Typography>
          </Paper>
        </Box>
      )}
      {topTab === 1 && (
      <Box sx={{ flex: 1, display: "flex", gap: 2, p: 2, overflow: "hidden" }}>
        {/* Left panel (Sensors controls + Topics/Nodes) */}
        <Paper sx={{ flex: 1, display: "flex", flexDirection: "column", overflow: "hidden" }}>
          <Box sx={{ p: 2 }}>
            <Box sx={{ display: "flex", gap: 1 }}>
              <Button
                variant="contained"
                color={sensorState === "activated" ? "success" : "primary"}
                disabled={!!busy}
                onClick={() => postJson("/api/sensors/activate")}
                sx={{ flex: 1 }}
              >
                {sensorState === "activated" ? "Activated" : "Activate"}
              </Button>
              <Button
                variant="contained"
                color={sensorState === "deactivated" ? "error" : "inherit"}
                disabled={!!busy}
                onClick={() => postJson("/api/sensors/deactivate")}
                sx={{ flex: 1 }}
              >
                {sensorState === "deactivated" ? "Deactivated" : "Deactivate"}
              </Button>
              <Button
                variant="outlined"
                disabled={!!busy}
                onClick={() => {
                  void refreshTopics();
                  void refreshNodes();
                }}
                sx={{ whiteSpace: "nowrap" }}
              >
                Refresh
              </Button>
            </Box>
          </Box>
          <Divider />

          <Tabs value={leftTab} onChange={(_e, v) => setLeftTab(v)}>
            <Tab label="Devices" />
            <Tab label="Micro-ROS" />
            <Tab label="ROS Topics" />
            <Tab label="ROS Nodes" />
          </Tabs>
          <Divider />

          {leftTab === 0 ? (
            <Box sx={{ p: 2, display: "flex", gap: 1 }}>
              <Button
                variant="contained"
                color="warning"
                disabled={!!busy}
                onClick={refreshUsbDevices}
              >
                List USB Devices
              </Button>
            </Box>
          ) : leftTab === 1 ? (
            <Box sx={{ p: 2, display: "flex", gap: 1 }}>
              <Button
                variant="contained"
                color="warning"
                disabled={!!busy}
                onClick={startMros}
              >
                Connect to Arduino using Micro ROS
              </Button>
              <Button variant="outlined" onClick={stopMros}>
                Stop
              </Button>
            </Box>
          ) : leftTab === 2 ? (
            <Box sx={{ p: 2, display: "flex", gap: 1 }}>
              <Button variant="contained" disabled={!!busy} onClick={refreshTopics}>
                Refresh
              </Button>
              <TextField
                size="small"
                label="Filter"
                value={topicFilter}
                onChange={(e) => setTopicFilter(e.target.value)}
                fullWidth
              />
            </Box>
          ) : (
            <Box sx={{ p: 2, display: "flex", gap: 1 }}>
              <Button variant="contained" disabled={!!busy} onClick={refreshNodes}>
                Refresh
              </Button>
            </Box>
          )}

          <Divider />

          <Box sx={{ flex: 1, overflow: "auto" }}>
            {leftTab === 0
              ? usbDevices.map((d) => (
                  <Box
                    key={d}
                    sx={{
                      px: 2,
                      py: 1
                    }}
                  >
                    <Typography variant="body2">{d}</Typography>
                  </Box>
                ))
              : leftTab === 1
              ? (
                  <Box
                    component="pre"
                    sx={{
                      m: 0,
                      p: 2,
                      overflow: "auto",
                      fontSize: 12,
                      lineHeight: 1.35
                    }}
                  >
                    {mrosText || "Press “Connect to Arduino using Micro ROS” to start the agent.\n"}
                  </Box>
                )
              : leftTab === 2
              ? filteredTopics.map((t) => (
                  <Box
                    key={t}
                    sx={{
                      px: 2,
                      py: 1
                    }}
                  >
                    <Typography variant="body2">{t}</Typography>
                  </Box>
                ))
              : nodes.map((n) => (
                  <Box
                    key={n}
                    sx={{
                      px: 2,
                      py: 1
                    }}
                  >
                    <Typography variant="body2">{n}</Typography>
                  </Box>
                ))}
          </Box>
        </Paper>

        {/* Right panel */}
        <Paper sx={{ flex: 2, display: "flex", flexDirection: "column", overflow: "hidden" }}>
          <Tabs value={rightTab} onChange={(_e, v) => setRightTab(v)}>
            <Tab label="Echo" />
            <Tab label="Plot" />
            <Tab label="Cameras" />
            <Tab label="SONARS" />
            <Tab label="Modem" />
          </Tabs>
          <Divider />

          {/* Main + resizable log split */}
          <Box
            ref={rightSplitRef}
            sx={{
              flex: 1,
              overflow: "hidden",
              display: "grid",
              gridTemplateRows: `${(1 - logFrac) * 100}% 6px ${logFrac * 100}%`
            }}
          >
            <Box sx={{ overflow: "hidden", display: "flex", flexDirection: "column" }}>
            {rightTab === 0 && (
              <>
                <Box sx={{ p: 2, display: "flex", gap: 1, alignItems: "center" }}>
                  <Autocomplete
                    options={topics}
                    value={echoTopic || null}
                    onChange={(_e, v) => setEchoTopic(v ?? "")}
                    freeSolo
                    fullWidth
                    renderInput={(params) => (
                      <TextField
                        {...params}
                        size="small"
                        label="Topic"
                        placeholder="Select a topic"
                      />
                    )}
                  />
                  <Button variant="contained" onClick={startEcho}>
                    Echo
                  </Button>
                  <Button variant="outlined" onClick={stopEcho}>
                    Stop
                  </Button>
                  <Button
                    variant="text"
                    onClick={() => setEchoText("")}
                    sx={{ whiteSpace: "nowrap" }}
                  >
                    Clear
                  </Button>
                </Box>
                <Divider />
                <Box
                  component="pre"
                  sx={{
                    flex: 1,
                    m: 0,
                    p: 2,
                    overflow: "auto",
                    fontSize: 12,
                    lineHeight: 1.35
                  }}
                >
                  {echoText}
                </Box>
              </>
            )}

          {rightTab === 1 && (
            <Box sx={{ p: 2, display: "flex", gap: 1, alignItems: "center" }}>
              <Autocomplete
                options={topics}
                value={plotTopic || null}
                onChange={(_e, v) => setPlotTopic(v ?? "")}
                freeSolo
                fullWidth
                renderInput={(params) => (
                  <TextField {...params} size="small" label="Topic (rate plot)" />
                )}
              />
              <Button variant="contained" onClick={startPlot}>
                Plot
              </Button>
              <Button variant="outlined" onClick={stopPlot}>
                Stop
              </Button>
            </Box>
          )}

          {rightTab === 2 && (
            <Box sx={{ p: 2, display: "flex", flexDirection: "column", gap: 2 }}>
              <Alert severity="info">
                Camera viewer expects a <b>CompressedImage</b> topic. Example:{" "}
                <code>/front/image_raw/compressed</code> and <code>/bottom/image_raw/compressed</code>
              </Alert>
              <Box sx={{ display: "flex", gap: 1, alignItems: "center" }}>
                <Autocomplete
                  options={topics.filter((t) => t.includes("compressed"))}
                  value={camTopic || null}
                  onChange={(_e, v) => setCamTopic(v ?? "")}
                  freeSolo
                  fullWidth
                  renderInput={(params) => (
                    <TextField {...params} size="small" label="Camera topic (CompressedImage)" />
                  )}
                />
                <Button variant="contained" onClick={startCamera}>
                  View
                </Button>
                <Button variant="outlined" onClick={stopCamera}>
                  Stop
                </Button>
              </Box>
              {camErr && <Alert severity="error">{camErr}</Alert>}
              <Box
                sx={{
                  flex: 1,
                  minHeight: 320,
                  display: "flex",
                  alignItems: "center",
                  justifyContent: "center",
                  bgcolor: "rgba(255,255,255,0.04)",
                  borderRadius: 1,
                  overflow: "hidden"
                }}
              >
                {camImgUrl ? (
                  <img
                    src={camImgUrl}
                    style={{ maxWidth: "100%", maxHeight: "100%", objectFit: "contain" }}
                  />
                ) : (
                  <Typography variant="body2" sx={{ opacity: 0.7 }}>
                    No frame yet.
                  </Typography>
                )}
              </Box>
            </Box>
          )}

          {rightTab === 3 && (
            <Box sx={{ p: 2, display: "flex", flexDirection: "column", gap: 2 }}>
              <Button
                variant="contained"
                disabled={!!busy}
                onClick={startDvlTunnel}
                sx={{
                  width: "fit-content",
                  bgcolor: "#ffeb3b",
                  color: "rgba(0,0,0,0.87)",
                  "&:hover": { bgcolor: "#fdd835" }
                }}
              >
                VIEW DVL GUI
              </Button>
              {dvlUrl ? (
                <Alert severity="success">
                  You can view the DVL GUI at{" "}
                  <Link href={dvlUrl} target="_blank" rel="noreferrer">
                    {dvlUrl}
                  </Link>
                </Alert>
              ) : (
                <Alert severity="info">
                  Press “VIEW DVL GUI” to expose the DVL UI on <b>http://localhost:8080</b>.
                </Alert>
              )}
            </Box>
          )}

          {rightTab === 4 && (
            <Box sx={{ p: 2, display: "flex", flexDirection: "column", gap: 2, overflow: "hidden" }}>
              <Box sx={{ display: "flex", gap: 1, alignItems: "center" }}>
                <TextField
                  size="small"
                  label="Send (max 8 chars)"
                  value={modemText}
                  inputProps={{ maxLength: 8 }}
                  onChange={(e) => setModemText(e.target.value)}
                  fullWidth
                />
                <Button variant="contained" color="warning" disabled={!!busy} onClick={sendModem}>
                  Send
                </Button>
              </Box>

              <Box sx={{ display: "flex", gap: 1 }}>
                <Button
                  variant="outlined"
                  disabled={!!busy}
                  onClick={() => {
                    const available = topics.filter((t) => t.includes("/modem/"));
                    ensureModemEchos(available);
                  }}
                >
                  Attach modem topics
                </Button>
                <Typography variant="body2" sx={{ opacity: 0.75, alignSelf: "center" }}>
                  Echo stays open for up to 4 modem topics (auto-adapts as new topics appear).
                </Typography>
              </Box>

              <Paper sx={{ flex: 1, display: "flex", flexDirection: "column", overflow: "hidden" }}>
                <Tabs value={modemTab} onChange={(_e, v) => setModemTab(v)} variant="scrollable">
                  {modemTopics.length ? (
                    modemTopics.map((t) => <Tab key={t} label={t} />)
                  ) : (
                    <Tab label="No modem topics yet" disabled />
                  )}
                </Tabs>
                <Divider />
                <Box
                  component="pre"
                  sx={{
                    flex: 1,
                    m: 0,
                    p: 2,
                    overflow: "auto",
                    fontSize: 12,
                    lineHeight: 1.35
                  }}
                >
                  {modemTopics[modemTab] ? modemLogs[modemTopics[modemTab]] ?? "" : ""}
                </Box>
              </Paper>
            </Box>
          )}
            </Box>

            {/* Drag handle */}
            <Box
              onMouseDown={onStartDrag}
              sx={{
                cursor: "row-resize",
                bgcolor: "rgba(255,255,255,0.08)",
                "&:hover": { bgcolor: "rgba(255,255,255,0.16)" }
              }}
            />

            {/* Bottom log panel */}
            <Box sx={{ minHeight: 160, display: "flex", flexDirection: "column", overflow: "hidden" }}>
              <Box sx={{ display: "flex", alignItems: "center" }}>
                <Tabs value={bottomTab} onChange={(_e, v) => setBottomTab(v)}>
                  <Tab label="Log" />
                </Tabs>
                <Box sx={{ flex: 1 }} />
                <Button
                  variant="text"
                  onClick={() => setLogText("")}
                  sx={{ mr: 1, whiteSpace: "nowrap" }}
                >
                  Clear log
                </Button>
              </Box>
              <Divider />
              <Box
                component="pre"
                sx={{
                  flex: 1,
                  m: 0,
                  p: 2,
                  overflow: "auto",
                  fontSize: 12,
                  lineHeight: 1.35,
                  opacity: 0.9
                }}
              >
                {logText}
              </Box>
            </Box>
          </Box>
        </Paper>
      </Box>
      )}
      {/* Terminals are always mounted so WS connections + history survive tab switches */}
      <TerminalSplitPane visible={topTab === 2} mounted={termsMounted} wsBase={wsBase} />
    </Box>
  );
}

