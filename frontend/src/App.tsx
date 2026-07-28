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
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const fetchScene = useCallback(async () => {
    try {
      const res = await fetch(`${API_BASE}/scene`);
      if (!res.ok) throw new Error(`Server returned ${res.status}`);
      const data: SceneData = await res.json();
      setScene(data);
      setError(null);
    } catch (e) {
      const msg = e instanceof Error ? e.message : String(e);
      setError(msg);
      if (!scene) setScene(createDemoScene());
    } finally {
      setLoading(false);
    }
  }, [scene]);

  useEffect(() => {
    fetchScene();
    const interval = setInterval(fetchScene, 2000);
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
            <Scene scene={scene} onSelect={setSelectedId} selected={selectedId} showSites={showSites} />
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
        onImport={handleImport}
        onRefresh={fetchScene}
      />
    </div>
  );
}

function createDemoScene(): SceneData {
  return {
    num_entities: 6,
    bodies: [
      { id: 0, name: "ground", mass: 0, com: [0, 0, 0], parent_id: null, transform: { translation: [0, 0, 0], rotation: [1, 0, 0, 0] } },
      { id: 1, name: "pelvis", mass: 11.78, com: [0, 0, 0], parent_id: 0, transform: { translation: [0, 0, 0.85], rotation: [1, 0, 0, 0] } },
      { id: 2, name: "femur_r", mass: 9.3, com: [0, 0, -0.17], parent_id: 1, transform: { translation: [0.08, -0.1, -0.07], rotation: [1, 0, 0, 0] } },
      { id: 3, name: "tibia_r", mass: 3.71, com: [0, 0, -0.18], parent_id: 2, transform: { translation: [0, 0, -0.42], rotation: [1, 0, 0, 0] } },
      { id: 4, name: "talus_r", mass: 0.1, com: [0, 0, 0], parent_id: 3, transform: { translation: [0.02, 0, -0.4], rotation: [1, 0, 0, 0] } },
    ],
    joints: [
      { id: 5, name: "hip_r", joint_type: "hinge", body_a: 1, body_b: 2, axis: [0, 0, 1], limits: { lower: -0.52, upper: 2.09 } },
      { id: 6, name: "knee_r", joint_type: "hinge", body_a: 2, body_b: 3, axis: [0, 0, 1], limits: { lower: 0, upper: 2.09 } },
      { id: 7, name: "ankle_r", joint_type: "hinge", body_a: 3, body_b: 4, axis: [0, 0, 1], limits: { lower: -0.7, upper: 0.52 } },
    ],
    muscles: [],
    sites: [],
    meshes: [],
  };
}
