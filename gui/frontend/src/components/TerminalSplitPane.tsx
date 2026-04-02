import { useCallback, useRef, useState } from "react";
import { Box, Divider, Paper, Typography } from "@mui/material";
import { TerminalPane } from "./TerminalPane";

type CellId = "tl" | "tr" | "bl" | "br";

type Props = {
  visible: boolean;
  mounted: boolean;
  wsBase: string;
};

function useHDrag(initial: number) {
  const [pct, setPct] = useState(initial);
  const containerRef = useRef<HTMLDivElement | null>(null);

  const onPointerDown = useCallback(
    (e: React.PointerEvent) => {
      e.preventDefault();
      const container = containerRef.current;
      if (!container) return;
      const startX = e.clientX;
      const startPct = pct;
      const w = container.getBoundingClientRect().width;
      const onMove = (ev: PointerEvent) => {
        setPct(Math.min(80, Math.max(20, startPct + ((ev.clientX - startX) / w) * 100)));
      };
      const onUp = () => {
        document.removeEventListener("pointermove", onMove);
        document.removeEventListener("pointerup", onUp);
        document.body.style.cursor = "";
        document.body.style.userSelect = "";
      };
      document.body.style.cursor = "col-resize";
      document.body.style.userSelect = "none";
      document.addEventListener("pointermove", onMove);
      document.addEventListener("pointerup", onUp);
    },
    [pct],
  );

  return { pct, containerRef, onPointerDown };
}

function useVDrag(initial: number) {
  const [pct, setPct] = useState(initial);
  const containerRef = useRef<HTMLDivElement | null>(null);

  const onPointerDown = useCallback(
    (e: React.PointerEvent) => {
      e.preventDefault();
      const container = containerRef.current;
      if (!container) return;
      const startY = e.clientY;
      const startPct = pct;
      const h = container.getBoundingClientRect().height;
      const onMove = (ev: PointerEvent) => {
        setPct(Math.min(80, Math.max(20, startPct + ((ev.clientY - startY) / h) * 100)));
      };
      const onUp = () => {
        document.removeEventListener("pointermove", onMove);
        document.removeEventListener("pointerup", onUp);
        document.body.style.cursor = "";
        document.body.style.userSelect = "";
      };
      document.body.style.cursor = "row-resize";
      document.body.style.userSelect = "none";
      document.addEventListener("pointermove", onMove);
      document.addEventListener("pointerup", onUp);
    },
    [pct],
  );

  return { pct, containerRef, onPointerDown };
}

const hHandleSx = {
  flex: "0 0 8px",
  cursor: "col-resize",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
  "&:hover > div, &:active > div": { bgcolor: "primary.main" },
} as const;

const vHandleSx = {
  flex: "0 0 8px",
  cursor: "row-resize",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
  "&:hover > div, &:active > div": { bgcolor: "primary.main" },
} as const;

const hPillSx = { width: 3, height: 40, borderRadius: 2, bgcolor: "divider", transition: "background-color 0.15s" } as const;
const vPillSx = { height: 3, width: 40, borderRadius: 2, bgcolor: "divider", transition: "background-color 0.15s" } as const;

function TermCell({ label, focused, onFocus, children }: { label: string; focused: boolean; onFocus: () => void; children: React.ReactNode }) {
  return (
    <Paper
      onFocusCapture={onFocus}
      onPointerDown={onFocus}
      sx={{ flex: 1, display: "flex", flexDirection: "column", overflow: "hidden", minWidth: 0, minHeight: 0 }}
    >
      <Typography
        variant="subtitle2"
        sx={{
          px: 2,
          py: 0.5,
          fontSize: "0.75rem",
          bgcolor: focused ? "#c0392b" : "action.hover",
          color: focused ? "#fff" : "text.secondary",
          transition: "background-color 0.15s, color 0.15s",
        }}
      >
        {label}
      </Typography>
      <Divider />
      <Box sx={{ flex: 1, minHeight: 0, display: "flex" }}>{children}</Box>
    </Paper>
  );
}

export function TerminalSplitPane({ visible, mounted, wsBase }: Props) {
  const hDrag = useHDrag(50);
  const vDrag = useVDrag(50);
  const [focusedCell, setFocusedCell] = useState<CellId>("tl");

  return (
    <Box
      ref={(el: HTMLDivElement | null) => {
        hDrag.containerRef.current = el;
        vDrag.containerRef.current = el;
      }}
      sx={{
        flex: 1,
        display: visible ? "flex" : "none",
        flexDirection: "column",
        p: 1,
        gap: 0,
        overflow: "hidden",
      }}
    >
      {/* Top row */}
      <Box sx={{ flex: `0 0 calc(${vDrag.pct}% - 4px)`, display: "flex", overflow: "hidden" }}>
        <Box sx={{ flex: `0 0 calc(${hDrag.pct}% - 4px)`, display: "flex", overflow: "hidden" }}>
          <TermCell label="Jetson" focused={focusedCell === "tl"} onFocus={() => setFocusedCell("tl")}>
            {mounted && <TerminalPane mode="jetson" wsBase={wsBase} />}
          </TermCell>
        </Box>
        <Box onPointerDown={hDrag.onPointerDown} sx={hHandleSx}>
          <Box sx={hPillSx} />
        </Box>
        <Box sx={{ flex: 1, display: "flex", overflow: "hidden" }}>
          <TermCell label="Docker" focused={focusedCell === "tr"} onFocus={() => setFocusedCell("tr")}>
            {mounted && <TerminalPane mode="docker" wsBase={wsBase} />}
          </TermCell>
        </Box>
      </Box>

      {/* Horizontal divider */}
      <Box onPointerDown={vDrag.onPointerDown} sx={vHandleSx}>
        <Box sx={vPillSx} />
      </Box>

      {/* Bottom row */}
      <Box sx={{ flex: 1, display: "flex", overflow: "hidden" }}>
        <Box sx={{ flex: `0 0 calc(${hDrag.pct}% - 4px)`, display: "flex", overflow: "hidden" }}>
          <TermCell label="Jetson" focused={focusedCell === "bl"} onFocus={() => setFocusedCell("bl")}>
            {mounted && <TerminalPane mode="jetson" wsBase={wsBase} />}
          </TermCell>
        </Box>
        <Box onPointerDown={hDrag.onPointerDown} sx={hHandleSx}>
          <Box sx={hPillSx} />
        </Box>
        <Box sx={{ flex: 1, display: "flex", overflow: "hidden" }}>
          <TermCell label="Docker" focused={focusedCell === "br"} onFocus={() => setFocusedCell("br")}>
            {mounted && <TerminalPane mode="docker" wsBase={wsBase} />}
          </TermCell>
        </Box>
      </Box>
    </Box>
  );
}
