import { useState } from "react";
import type { Scene as SceneData } from "../types/schema";

interface ControlPanelProps {
  scene: SceneData | null;
  selectedId: number | null;
  transformMode: "translate" | "rotate";
  onModeChange: (mode: "translate" | "rotate") => void;
  showSites: boolean;
  onSiteToggle: () => void;
  showMuscles: boolean;
  onMuscleToggle: () => void;
  onImport: (path: string, format: string) => void;
  onRefresh: () => void;
}

export default function ControlPanel({
  scene, selectedId, transformMode, onModeChange,
  showSites, onSiteToggle, showMuscles, onMuscleToggle,
  onImport, onRefresh,
}: ControlPanelProps) {
  const [importPath, setImportPath] = useState("");
  const [importFormat, setImportFormat] = useState("mjcf");
  const [status, setStatus] = useState<string | null>(null);

  const selectedBody = scene?.bodies.find(b => b.id === selectedId);

  const handleImport = async () => {
    if (!importPath) return;
    try {
      await onImport(importPath, importFormat);
      setStatus("Import successful");
      setTimeout(() => setStatus(null), 3000);
    } catch (e) {
      setStatus(`Error: ${e instanceof Error ? e.message : e}`);
    }
  };

  return (
    <div className="control-panel">
      <h2>Melosim Editor</h2>

      {status && <div className="status">{status}</div>}

      <section>
        <h3>Import Model</h3>
        <div className="form-row">
          <input
            type="text"
            placeholder="Path to .mjcf or .osim file"
            value={importPath}
            onChange={e => setImportPath(e.target.value)}
          />
        </div>
        <div className="form-row">
          <select value={importFormat} onChange={e => setImportFormat(e.target.value)}>
            <option value="mjcf">MJCF (MuJoCo)</option>
            <option value="osim">OSIM (OpenSim)</option>
          </select>
          <button onClick={handleImport}>Import</button>
        </div>
      </section>

      <section>
        <h3>View</h3>
        <label>
          <input type="checkbox" checked={showMuscles} onChange={onMuscleToggle} />
          Show Muscles
        </label>
        <label>
          <input type="checkbox" checked={showSites} onChange={onSiteToggle} />
          Show Sites
        </label>
      </section>

      <section>
        <h3>Transform Mode</h3>
        <div className="button-group">
          <button className={transformMode === "translate" ? "active" : ""} onClick={() => onModeChange("translate")}>Translate</button>
          <button className={transformMode === "rotate" ? "active" : ""} onClick={() => onModeChange("rotate")}>Rotate</button>
        </div>
      </section>

      {selectedBody && (
        <section>
          <h3>Selected: {selectedBody.name}</h3>
          <div className="info-grid">
            <span className="label">ID:</span><span>{selectedBody.id}</span>
            <span className="label">Mass:</span><span>{selectedBody.mass.toFixed(3)} kg</span>
            <span className="label">Position:</span><span>{selectedBody.transform.translation.map(v => v.toFixed(3)).join(", ")}</span>
            <span className="label">Rotation:</span><span>{selectedBody.transform.rotation.map(v => v.toFixed(3)).join(", ")}</span>
            <span className="label">COM:</span><span>{selectedBody.com.map(v => v.toFixed(3)).join(", ")}</span>
          </div>
        </section>
      )}

      {scene && (
        <section>
          <h3>Statistics</h3>
          <div className="info-grid">
            <span className="label">Entities:</span><span>{scene.num_entities}</span>
            <span className="label">Bodies:</span><span>{scene.bodies.length}</span>
            <span className="label">Joints:</span><span>{scene.joints.length}</span>
            <span className="label">Muscles:</span><span>{scene.muscles.length}</span>
            <span className="label">Sites:</span><span>{scene.sites.length}</span>
          </div>
        </section>
      )}

      <section>
        <button onClick={onRefresh} className="refresh-btn">↻ Refresh</button>
      </section>
    </div>
  );
}
