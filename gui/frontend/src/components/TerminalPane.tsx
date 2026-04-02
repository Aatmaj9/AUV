import { useEffect, useMemo, useRef } from "react";
import { Box } from "@mui/material";
import { Terminal } from "xterm";
import { FitAddon } from "xterm-addon-fit";
import "xterm/css/xterm.css";

type Props = {
  mode: "jetson" | "docker";
  wsBase: string;
};

export function TerminalPane({ mode, wsBase }: Props) {
  const elRef = useRef<HTMLDivElement | null>(null);
  const termRef = useRef<Terminal | null>(null);
  const fitRef = useRef<FitAddon | null>(null);
  const wsRef = useRef<WebSocket | null>(null);

  const wsUrl = useMemo(() => `${wsBase}/ws/term?mode=${encodeURIComponent(mode)}`, [mode, wsBase]);

  useEffect(() => {
    const el = elRef.current;
    if (!el) return;

    const term = new Terminal({
      cursorBlink: true,
      fontSize: 12,
      fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace',
      theme: {
        background: "#0b0f14",
      },
      scrollback: 5000,
    });
    const fit = new FitAddon();
    term.loadAddon(fit);
    term.open(el);
    term.focus();

    termRef.current = term;
    fitRef.current = fit;

    const ws = new WebSocket(wsUrl);
    wsRef.current = ws;

    const sendResize = () => {
      try {
        fit.fit();
        const cols = term.cols;
        const rows = term.rows;
        if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "resize", cols, rows }));
      } catch {}
    };

    const ro = new ResizeObserver(() => sendResize());
    ro.observe(el);

    ws.onopen = () => {
      term.writeln(`\r\n[connected] ${mode}\r\n`);
      sendResize();
    };

    ws.onmessage = (ev) => {
      try {
        const msg = JSON.parse(String(ev.data)) as any;
        if (msg?.type === "output" && typeof msg.data === "string") term.write(msg.data);
        if (msg?.type === "status" && typeof msg.message === "string") term.writeln(`\r\n[status] ${msg.message}\r\n`);
      } catch {
        term.write(String(ev.data));
      }
    };

    ws.onerror = () => term.writeln("\r\n[error] websocket error\r\n");
    ws.onclose = () => term.writeln("\r\n[closed]\r\n");

    const dispo = term.onData((data) => {
      try {
        if (ws.readyState === ws.OPEN) ws.send(JSON.stringify({ type: "input", data }));
      } catch {}
    });

    // initial fit after paint
    const t = window.setTimeout(sendResize, 50);

    return () => {
      window.clearTimeout(t);
      try {
        dispo.dispose();
      } catch {}
      try {
        ro.disconnect();
      } catch {}
      try {
        ws.close();
      } catch {}
      wsRef.current = null;
      try {
        term.dispose();
      } catch {}
      termRef.current = null;
      fitRef.current = null;
    };
  }, [mode, wsUrl]);

  return <Box ref={elRef} sx={{ flex: 1, minHeight: 0, "& .xterm": { padding: 1 } }} />;
}

