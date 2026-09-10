import React, {
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from "react";
import { createRoot } from "react-dom/client";
import {
  command,
  desktop,
  type Command,
  type MeshData,
  type Node,
  type Snapshot,
} from "./api";
import { Viewport } from "./Viewport";
import { Inspector } from "./Inspector";
import { CommandDialog, type Dialog } from "./dialogs";
import "./style.css";
const empty: Snapshot = {
  model: "Untitled model",
  epoch: 0,
  nodes: [],
  meshes: [],
  issues: [],
  selected: null,
  message: "",
  can_undo: false,
  can_redo: false,
};
const icons: Record<string, string> = {
  body: "◇",
  mesh: "▧",
  frame: "⌖",
  joint: "⊙",
  coordinate: "↗",
  site: "•",
  cable: "⌁",
  muscle: "≈",
};

function App() {
  const [snapshot, setSnapshot] = useState(empty),
    [meshes, setMeshes] = useState(new Map<string, MeshData>());
  const [selected, select] = useState<string | null>(null),
    [error, setError] = useState(""),
    [busy, setBusy] = useState(false),
    [connected, setConnected] = useState(false);
  const [dialog, setDialog] = useState<Dialog | null>(null),
    [query, setQuery] = useState(""),
    [category, setCategory] = useState("structure");
  const [mode, setMode] = useState<"translate" | "rotate">("translate"),
    [sites, setSites] = useState(false),
    [muscles, setMuscles] = useState(false),
    [frames, setFrames] = useState(false),
    [focus, setFocus] = useState(0);
  const [placing, setPlacing] = useState<string | null>(null),
    [collapsed, setCollapsed] = useState(new Set<string>());
  const lock = useRef(false),
    epoch = useRef(0);
  async function send(c: Command) {
    if (lock.current) return false;
    lock.current = true;
    setBusy(true);
    setError("");
    try {
      const s = await command(c);
      setConnected(true);
      if (s.epoch !== epoch.current) {
        epoch.current = s.epoch;
        select(null);
        setPlacing(null);
        setMeshes(new Map(s.meshes.map((m) => [m.id, m])));
      } else if (s.meshes.length)
        setMeshes((old) => {
          const next = new Map(old);
          s.meshes.forEach((m) => next.set(m.id, m));
          return next;
        });
      setSnapshot(s);
      if (s.selected) {
        select(s.selected);
        const kind = s.nodes.find((n) => n.id === s.selected)?.kind;
        setCategory(
          kind === "cable"
            ? "cables"
            : kind === "muscle"
              ? "muscles"
              : "structure",
        );
        setQuery("");
      }
      return true;
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
      return false;
    } finally {
      lock.current = false;
      setBusy(false);
    }
  }
  useEffect(() => {
    void send({ type: "snapshot" });
  }, []);
  useEffect(() => {
    document.title = `${snapshot.model} · melosim`;
  }, [snapshot.model]);
  useEffect(() => {
    const key = (e: KeyboardEvent) => {
      if ((e.target as HTMLElement).matches("input,textarea,select")) return;
      if (e.key === "Escape") {
        setDialog(null);
        setPlacing(null);
      }
      if (e.key.toLowerCase() === "f") setFocus((f) => f + 1);
      if (e.key === "1") setMode("translate");
      if (e.key === "2") setMode("rotate");
      if ((e.ctrlKey || e.metaKey) && e.key === "z") {
        e.preventDefault();
        void send({ type: e.shiftKey ? "redo" : "undo" });
      }
    };
    window.addEventListener("keydown", key);
    return () => window.removeEventListener("keydown", key);
  });
  const node = snapshot.nodes.find((n) => n.id === selected) || null;
  const bodyCount = snapshot.nodes.filter((n) => n.kind === "body").length;
  const parents = snapshot.nodes.filter((n) =>
    ["body", "frame"].includes(n.kind),
  );
  const cables = snapshot.nodes.filter((n) => n.kind === "cable");
  const children = useMemo(() => {
    const m = new Map<string | null, Node[]>();
    snapshot.nodes.forEach((n) => {
      const a = m.get(n.parent) || [];
      a.push(n);
      m.set(n.parent, a);
    });
    return m;
  }, [snapshot.nodes]);
  const matches = (n: Node): boolean =>
    n.name.toLowerCase().includes(query.toLowerCase()) ||
    (children.get(n.id) || []).some(matches);
  function row(n: Node, depth = 0): ReactNode {
    if (n.kind === "mesh" || (query && !matches(n))) return null;
    const kids = (children.get(n.id) || []).filter((c) => c.kind !== "mesh"),
      closed = collapsed.has(n.id) && !query;
    return (
      <React.Fragment key={n.id}>
        <div
          className={`tree-row ${selected === n.id ? "selected" : ""}`}
          style={{ paddingLeft: 12 + Math.min(depth, 6) * 10 }}
        >
          <button
            className="chevron"
            disabled={!kids.length || !!query}
            aria-label={`${closed ? "Expand" : "Collapse"} ${n.name}`}
            onClick={() =>
              setCollapsed((s) => {
                const next = new Set(s);
                next.has(n.id) ? next.delete(n.id) : next.add(n.id);
                return next;
              })
            }
          >
            {kids.length && !query ? (closed ? "›" : "⌄") : ""}
          </button>
          <button
            className="tree-item"
            onClick={() => select(n.id)}
            title={n.name}
          >
            <span className={`node-icon ${n.kind}`}>{icons[n.kind]}</span>
            <span>{n.name}</span>
          </button>
        </div>
        {!query && !closed && kids.map((c) => row(c, depth + 1))}
      </React.Fragment>
    );
  }
  const showDialog = (d: Dialog) => {
    setError("");
    setDialog(d);
  };
  return (
    <div className="app-shell">
      <header>
        <div className="brand">
          <span className="brand-mark">m</span>
          <strong>melosim</strong>
          <span className="badge">WORKSPACE</span>
        </div>
        <div className="document-name">
          <span className="connection-dot" data-connected={connected} />
          {snapshot.model}
        </div>
        <div className="header-actions">
          <span className="quiet">{desktop ? "Desktop" : "Local editor"}</span>
          <button
            onClick={() => showDialog("export")}
            disabled={!bodyCount || busy}
          >
            Export MJCF ↗
          </button>
        </div>
      </header>
      <nav className="toolbar">
        <div className="tool-group">
          <button
            className="primary"
            onClick={() => void send({ type: "load_arm" })}
            disabled={busy}
          >
            Load MyoArm
          </button>
          <button onClick={() => showDialog("import")} disabled={busy}>
            Open model…
          </button>
        </div>
        <div className="tool-group">
          <button
            disabled={!bodyCount || busy}
            onClick={() => showDialog("body")}
          >
            ＋ Body
          </button>
          <button
            disabled={!bodyCount || busy}
            onClick={() => showDialog("part")}
          >
            Import part
          </button>
          <button
            disabled={!bodyCount || busy}
            onClick={() => showDialog("joint")}
          >
            Joint
          </button>
          <button
            disabled={!bodyCount || busy}
            onClick={() => showDialog("cable")}
          >
            Cable
          </button>
          <button
            disabled={!cables.length || busy}
            onClick={() => showDialog("site")}
          >
            Path site
          </button>
        </div>
        <div className="tool-group end">
          <button
            title="Undo property edit (Ctrl+Z)"
            disabled={!snapshot.can_undo || busy}
            onClick={() => void send({ type: "undo" })}
          >
            ↶
          </button>
          <button
            title="Redo property edit"
            disabled={!snapshot.can_redo || busy}
            onClick={() => void send({ type: "redo" })}
          >
            ↷
          </button>
          <span className="quiet">
            {busy ? "Updating…" : `${bodyCount} bodies`}
          </span>
        </div>
      </nav>
      <aside className="hierarchy">
        <div className="panel-heading">
          MODEL EXPLORER <span>{snapshot.nodes.length}</span>
        </div>
        <input
          className="search"
          placeholder="Search the model…"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
        />
        <div className="tabs">
          {["structure", "cables", "muscles"].map((t) => (
            <button
              key={t}
              className={category === t ? "active" : ""}
              onClick={() => setCategory(t)}
            >
              {t}
            </button>
          ))}
        </div>
        <div className="tree" role="tree">
          {query
            ? snapshot.nodes
                .filter(
                  (n) =>
                    n.name.toLowerCase().includes(query.toLowerCase()) &&
                    (category === "structure"
                      ? !["cable", "muscle"].includes(n.kind)
                      : n.kind ===
                        (category === "cables" ? "cable" : "muscle")),
                )
                .map((n) => row(n))
            : category === "structure"
              ? (children.get(null) || [])
                  .filter((n) => !["cable", "muscle"].includes(n.kind))
                  .map((n) => row(n))
              : snapshot.nodes
                  .filter(
                    (n) =>
                      n.kind === (category === "cables" ? "cable" : "muscle"),
                  )
                  .map((n) => row(n))}
          {!bodyCount && (
            <p className="empty-note">
              Load an arm model to explore its bodies, joints, and attachment
              sites.
            </p>
          )}
        </div>
        <div className="panel-foot">
          {cables.length} cables ·{" "}
          {snapshot.nodes.filter((n) => n.kind === "site").length} sites
        </div>
      </aside>
      <main className={`viewport ${placing ? "placing" : ""}`}>
        <Viewport
          snapshot={snapshot}
          meshes={meshes}
          selected={selected}
          select={select}
          send={send}
          mode={mode}
          sites={sites}
          muscles={muscles}
          frames={frames}
          focus={focus}
          placing={placing}
          place={(parent, position) => {
            const existing = new Set(snapshot.nodes.map((n) => n.name));
            let number = 1;
            while (existing.has(`cable_site_${number}`)) number++;
            void send({
              type: "add_site",
              parent,
              cable: placing,
              name: `cable_site_${number}`,
              position,
            }).then((ok) => {
              if (ok) setPlacing(null);
            });
          }}
        />
        <div className="viewport-tools">
          <button
            className={mode === "translate" ? "active" : ""}
            onClick={() => setMode("translate")}
          >
            Move <kbd>1</kbd>
          </button>
          <button
            className={mode === "rotate" ? "active" : ""}
            onClick={() => setMode("rotate")}
          >
            Rotate <kbd>2</kbd>
          </button>
          <button onClick={() => setFocus((f) => f + 1)}>
            Fit <kbd>F</kbd>
          </button>
          <span className="divider" />
          <label>
            <input
              type="checkbox"
              checked={sites}
              onChange={(e) => setSites(e.target.checked)}
            />
            Sites
          </label>
          <label>
            <input
              type="checkbox"
              checked={muscles}
              onChange={(e) => setMuscles(e.target.checked)}
            />
            Muscles
          </label>
          <label>
            <input
              type="checkbox"
              checked={frames}
              onChange={(e) => setFrames(e.target.checked)}
            />
            Frames
          </label>
        </div>
        {!bodyCount && (
          <div className="welcome">
            <div className="eyebrow">MODEL · DESIGN · SIMULATE</div>
            <h1>Your model, in motion.</h1>
            <p>
              Import a musculoskeletal model, add an exoskeleton,
              <br />
              and author its cable paths in one workspace.
            </p>
            <button
              className="primary"
              disabled={busy}
              onClick={() => void send({ type: "load_arm" })}
            >
              Open the MyoArm example →
            </button>
          </div>
        )}
        {placing && (
          <div className="placement-banner">
            Click a body surface to add a cable site.{" "}
            <button onClick={() => setPlacing(null)}>Cancel</button>
          </div>
        )}
        <div className="viewport-caption">
          {node ? `${node.kind.toUpperCase()} / ${node.name}` : "PERSPECTIVE"}
          <span>Drag to orbit · Scroll to zoom · Right-drag to pan</span>
        </div>
      </main>
      <aside className="inspector">
        <div className="panel-heading">
          INSPECTOR <span>{node?.kind || "selection"}</span>
        </div>
        {node ? (
          <Inspector
            key={`${node.id}-${snapshot.epoch}`}
            node={node}
            snapshot={snapshot}
            send={send}
            busy={busy}
            select={select}
            place={() => {
              setPlacing(node.id);
              setSites(true);
            }}
          />
        ) : (
          <div className="empty-inspector">
            <span>⌖</span>
            <h3>Select an item</h3>
            <p>
              Choose a body in the viewport or an item in the explorer to edit
              its properties.
            </p>
          </div>
        )}
        <details className="validation" open={snapshot.issues.length > 0}>
          <summary>
            Model checks <span>{snapshot.issues.length || "✓"}</span>
          </summary>
          {snapshot.issues.length ? (
            snapshot.issues.map((i, k) => <p key={k}>{i}</p>)
          ) : (
            <p>No hierarchy errors detected.</p>
          )}
        </details>
      </aside>
      <footer>
        <span className={error ? "error" : ""} role="status">
          {error || snapshot.message || "Connecting to the editor…"}
        </span>
        <div>
          {error && (
            <button
              disabled={busy}
              onClick={() => void send({ type: "snapshot" })}
            >
              Reconnect
            </button>
          )}
          <span>m · rad · kg</span>
        </div>
      </footer>
      {dialog && (
        <CommandDialog
          key={dialog}
          dialog={dialog}
          parents={parents}
          cables={cables}
          nodes={snapshot.nodes}
          selected={node}
          busy={busy}
          error={error}
          close={() => setDialog(null)}
          send={async (c) => {
            const ok = await send(c);
            if (ok) setDialog(null);
            return ok;
          }}
        />
      )}
    </div>
  );
}
createRoot(document.getElementById("app")!).render(<App />);
