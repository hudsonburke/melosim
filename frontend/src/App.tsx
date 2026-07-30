import { useState, useEffect, useCallback } from "react";
import Scene from "./components/Scene";
import ControlPanel from "./components/ControlPanel";
import type { Scene as SceneData } from "./types/schema";
import "./App.css";

const API_BASE = import.meta.env.VITE_API_BASE || "";

const MODEL_EXT = /\.(xml|osim|json)$/i;

// ── Drag-and-drop helpers ─────────────────────────────

interface DroppedFile {
  file: File;
  path: string; // relative path, preserving dropped folder structure
}

async function walkEntry(entry: any, prefix: string, out: DroppedFile[]): Promise<void> {
  if (entry.isFile) {
    await new Promise<void>((res, rej) =>
      entry.file((f: File) => { out.push({ file: f, path: prefix + f.name }); res(); }, rej),
    );
  } else if (entry.isDirectory) {
    const reader = entry.createReader();
    // readEntries returns at most 100 entries per call (Chrome) — loop until empty
    let batch: any[];
    do {
      batch = await new Promise<any[]>((res, rej) => reader.readEntries(res, rej));
      for (const e of batch) await walkEntry(e, prefix + entry.name + "/", out);
    } while (batch.length > 0);
  }
}

async function collectDropped(items: DataTransferItemList): Promise<DroppedFile[]> {
  const out: DroppedFile[] = [];
  const tasks: Promise<void>[] = [];
  for (const item of Array.from(items)) {
    const entry = (item as any).webkitGetAsEntry?.();
    if (entry) tasks.push(walkEntry(entry, "", out));
    else {
      const f = item.getAsFile();
      if (f) out.push({ file: f, path: f.name });
    }
  }
  await Promise.all(tasks);
  return out;
}

export default function App() {
  const [scene, setScene] = useState<SceneData | null>(null);
  const [selectedId, setSelectedId] = useState<number | null>(null);
  const [transformMode, setTransformMode] = useState<"translate" | "rotate">("translate");
  const [showSites, setShowSites] = useState(false);
  const [showMuscles, setShowMuscles] = useState(false);
  const [loading, setLoading] = useState(true);
  const [dragging, setDragging] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [fetchMs, setFetchMs] = useState<number | null>(null);

  const fetchScene = useCallback(async () => {
    try {
      const start = performance.now();
      const res = await fetch(`${API_BASE}/scene`);
      if (!res.ok) throw new Error(`Server returned ${res.status}`);
      const data: SceneData = await res.json();
      const ms = Math.round(performance.now() - start);
      setFetchMs(ms);
      // Backwards compat: old server doesn't return muscle_paths
      if (!data.muscle_paths) data.muscle_paths = [];
      console.log(`[melosim] /scene: ${ms}ms, ${JSON.stringify(data).length} bytes`);
      console.log(`[melosim] counts: bodies=${data.bodies.length} meshes=${data.meshes.length} joints=${data.joints.length} muscles=${data.muscles.length} muscle_paths=${data.muscle_paths.length} sites=${data.sites.length}`);
      setScene(data);
      setError(null);
    } catch (e) {
      const msg = e instanceof Error ? e.message : String(e);
      console.error(`[melosim] fetchScene error: ${msg}`);
      setError(msg);
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchScene();
    const interval = setInterval(fetchScene, 5000); // 5s instead of 2s
    return () => clearInterval(interval);
  }, [fetchScene]);

  const handleDrop = async (e: React.DragEvent) => {
    e.preventDefault();
    setDragging(false);
    try {
      setLoading(true);
      const files = await collectDropped(e.dataTransfer.items);
      // Pick the shallowest model file (e.g. myoarm.xml over included XMLs)
      const model = files
        .filter((f) => MODEL_EXT.test(f.path))
        .sort((a, b) => a.path.split("/").length - b.path.split("/").length || a.path.localeCompare(b.path))[0];
      if (!model) throw new Error("No model file (.xml, .osim, .json) in drop");
      // Upload everything, preserving relative paths so MJCF includes/meshdir resolve
      const uploaded = new Map<string, string>();
      for (const f of files) {
        const url = `${API_BASE}/upload/${f.path.split("/").map(encodeURIComponent).join("/")}`;
        const res = await fetch(url, { method: "POST", body: f.file });
        if (!res.ok) throw new Error(`Upload failed: ${f.path}`);
        uploaded.set(f.path, (await res.json()).path);
      }
      const lower = model.path.toLowerCase();
      const format = lower.endsWith(".xml") ? "mjcf" : lower.endsWith(".osim") ? "osim" : "json";
      await handleImport(uploaded.get(model.path)!, format);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
      setLoading(false);
    }
  };

  const handleImport = async (path: string, format: string) => {
    try {
      setLoading(true);
      const res = await fetch(`${API_BASE}/import`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ path, format }),
      });
      if (!res.ok) {
        const err = await res.json();
        throw new Error(err.error || `Import failed: ${res.status}`);
      }
      await fetchScene();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="app-layout">
      <div
        className="viewport"
        onDragOver={(e) => { e.preventDefault(); setDragging(true); }}
        onDragLeave={() => setDragging(false)}
        onDrop={handleDrop}
      >
        {dragging && (
          <div className="loading" style={{ position: "absolute", inset: 0, zIndex: 20, pointerEvents: "none" }}>
            Drop MJCF (.xml), extracted OpenSim JSON (.json), or a model folder
          </div>
        )}
        {loading && !scene ? (
          <div className="loading">Loading model...</div>
        ) : (
          <>
            {error && <div className="error-banner">⚠ {error}</div>}
            {fetchMs !== null && (
              <div style={{ position: "absolute", top: 8, left: 8, color: "#888", fontSize: 12, zIndex: 10, fontFamily: "monospace" }}>
                {scene?.bodies.length} bodies, {scene?.meshes.length} meshes | fetch: {fetchMs}ms
              </div>
            )}
            <Scene
              scene={scene}
              onSelect={setSelectedId}
              selected={selectedId}
              showSites={showSites}
              showMuscles={showMuscles}
            />
          </>
        )}
      </div>
      <ControlPanel
        scene={scene}
        selectedId={selectedId}
        transformMode={transformMode}
        onModeChange={setTransformMode}
        showSites={showSites}
        onSiteToggle={() => setShowSites(!showSites)}
        showMuscles={showMuscles}
        onMuscleToggle={() => setShowMuscles(!showMuscles)}
        onImport={handleImport}
        onRefresh={fetchScene}
      />
    </div>
  );
}

