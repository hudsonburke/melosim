import { useState, useEffect, useCallback } from "react";
import Scene from "./components/Scene";
import ControlPanel from "./components/ControlPanel";
import type { Scene as SceneData } from "./types/schema";
import "./App.css";

const API_BASE = import.meta.env.VITE_API_BASE || "";

export default function App() {
  const [scene, setScene] = useState<SceneData | null>(null);
  const [selectedId, setSelectedId] = useState<number | null>(null);
  const [transformMode, setTransformMode] = useState<"translate" | "rotate">("translate");
  const [showSites, setShowSites] = useState(false);
  const [showMuscles, setShowMuscles] = useState(false);
  const [loading, setLoading] = useState(true);
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
      console.log(`[melosim] /scene: ${ms}ms, ${JSON.stringify(data).length} bytes`);
      console.log(`[melosim] counts: bodies=${data.bodies.length} meshes=${data.meshes.length} joints=${data.joints.length} muscles=${data.muscles.length} muscle_paths=${(data.muscle_paths ?? []).length} sites=${data.sites.length}`);
      // Backwards compat: old server doesn't return muscle_paths
      if (!data.muscle_paths) data.muscle_paths = [];
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
      <div className="viewport">
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

