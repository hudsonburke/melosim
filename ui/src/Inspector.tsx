import { useEffect, useState } from "react";
import type { Node, Snapshot, Command } from "./api";
export function Inspector({
  node: n,
  snapshot,
  send,
  busy,
  select,
  place,
}: {
  node: Node;
  snapshot: Snapshot;
  send: (c: Command) => Promise<boolean>;
  busy: boolean;
  select: (id: string) => void;
  place: () => void;
}) {
  const [name, setName] = useState(n.name),
    [position, setPosition] = useState(n.position),
    [q, setQ] = useState(n.value ?? 0),
    [parameters, setParameters] = useState(n.cable),
    [site, setSite] = useState("");
  useEffect(() => {
    setPosition(n.position);
    setQ(n.value ?? 0);
    setName(n.name);
    setParameters(n.cable);
  }, [n]);
  return (
    <div className="properties">
      <section>
        <div className="eyebrow">{n.kind}</div>
        <h2>{n.name}</h2>
        <label>
          Name
          <div className="inline-field">
            <input value={name} onChange={(e) => setName(e.target.value)} />
            <button
              disabled={busy || name === n.name}
              onClick={() => void send({ type: "rename", id: n.id, name })}
            >
              Save
            </button>
          </div>
        </label>
        {n.mass !== null && (
          <div className="metric">
            <span>Mass</span>
            <strong>
              {n.mass.toFixed(4)} <small>kg</small>
            </strong>
          </div>
        )}
      </section>
      {n.value !== null && n.range && (
        <section>
          <h3>
            Articulation <span>rad / m</span>
          </h3>
          <input
            aria-label="Joint position"
            type="range"
            min={n.range[0]}
            max={n.range[1]}
            step="0.001"
            value={q}
            disabled={busy || n.driven}
            onChange={(e) => setQ(+e.target.value)}
            onPointerUp={(e) =>
              void send({
                type: "set_coordinate",
                id: n.id,
                value: +e.currentTarget.value,
              })
            }
            onKeyUp={(e) =>
              void send({
                type: "set_coordinate",
                id: n.id,
                value: +e.currentTarget.value,
              })
            }
          />
          <div className="range-label">
            <span>{n.range[0].toFixed(3)}</span>
            <span>{n.range[1].toFixed(3)}</span>
          </div>
          <div className="inline-field">
            <input
              aria-label="Coordinate value"
              type="number"
              value={q}
              step="0.01"
              onChange={(e) => setQ(+e.target.value)}
            />
            <button
              disabled={busy || n.driven}
              onClick={() =>
                void send({ type: "set_coordinate", id: n.id, value: q })
              }
            >
              Apply
            </button>
          </div>
          {n.driven && <p className="hint">Driven by a coupled coordinate.</p>}
        </section>
      )}
      {n.kind === "joint" && (
        <section>
          <h3>Coordinates</h3>
          {snapshot.nodes
            .filter((c) => c.parent === n.id && c.kind === "coordinate")
            .map((c) => (
              <button key={c.id} className="wide" onClick={() => select(c.id)}>
                {c.name}
                <span>{c.value?.toFixed(3)}</span>
              </button>
            ))}
        </section>
      )}
      {n.editable && (
        <section>
          <h3>
            Local translation <span>m</span>
          </h3>
          <div className="xyz">
            {["X", "Y", "Z"].map((label, i) => (
              <label key={label}>
                {label}
                <input
                  aria-label={`${label} translation`}
                  type="number"
                  step="0.005"
                  value={position[i]}
                  onChange={(e) =>
                    setPosition(
                      (p) =>
                        p.map((v, j) =>
                          j === i ? +e.target.value : v,
                        ) as typeof position,
                    )
                  }
                />
              </label>
            ))}
          </div>
          <button
            className="wide"
            disabled={busy}
            onClick={() =>
              void send({
                type: "set_transform",
                id: n.id,
                position,
                quaternion: n.quaternion,
              })
            }
          >
            Apply position
          </button>
          <p className="hint">
            Use the viewport gizmo to move or rotate in the parent frame.
          </p>
        </section>
      )}
      {n.path.length > 0 || n.kind === "cable" ? (
        <section>
          <h3>
            Ordered path <span>{n.path.length} sites</span>
          </h3>
          {n.path.map((s, i) => (
            <div className="path-row" key={s}>
              <span>{i + 1}</span>
              <button className="path-name" onClick={() => select(s)}>
                {snapshot.nodes.find((n) => n.id === s)?.name || "Missing site"}
              </button>
              {n.kind === "cable" && (
                <>
                  <button
                    aria-label={`Move site ${i + 1} up`}
                    disabled={busy || i === 0}
                    onClick={() => {
                      const sites = [...n.path];
                      [sites[i - 1], sites[i]] = [sites[i], sites[i - 1]];
                      void send({ type: "set_path", id: n.id, sites });
                    }}
                  >
                    ↑
                  </button>
                  <button
                    aria-label={`Remove site ${i + 1} from path`}
                    disabled={busy}
                    onClick={() =>
                      void send({
                        type: "set_path",
                        id: n.id,
                        sites: n.path.filter((x) => x !== s),
                      })
                    }
                  >
                    ×
                  </button>
                </>
              )}
            </div>
          ))}
          <div className="metric">
            <span>Path length</span>
            <strong>
              {((n.length || 0) * 1000).toFixed(1)} <small>mm</small>
            </strong>
          </div>
          {n.kind === "cable" && (
            <>
              <button className="wide primary" disabled={busy} onClick={place}>
                ＋ Place site on a surface
              </button>
              <label>
                Append existing site
                <select value={site} onChange={(e) => setSite(e.target.value)}>
                  <option value="">Choose a site…</option>
                  {snapshot.nodes
                    .filter((s) => s.kind === "site" && !n.path.includes(s.id))
                    .map((s) => (
                      <option key={s.id} value={s.id}>
                        {s.name}
                      </option>
                    ))}
                </select>
              </label>
              <button
                className="wide"
                disabled={!site || busy}
                onClick={() =>
                  void send({
                    type: "set_path",
                    id: n.id,
                    sites: [...n.path, site],
                  })
                }
              >
                Append to path
              </button>
            </>
          )}
        </section>
      ) : null}
      {parameters && (
        <section>
          <h3>Cable parameters</h3>
          {[
            "Rest length (m)",
            "Stiffness (N/m)",
            "Damping (N·s/m)",
            "Maximum tension (N)",
            "Actuator force (N)",
          ].map((label, i) => (
            <label key={label}>
              {label}
              <input
                type="number"
                min="0"
                step="any"
                value={parameters[i]}
                onChange={(e) =>
                  setParameters(
                    (p) =>
                      p!.map((v, j) =>
                        j === i ? +e.target.value : v,
                      ) as typeof parameters,
                  )
                }
              />
            </label>
          ))}
          <button
            className="wide"
            disabled={busy}
            onClick={() =>
              void send({
                type: "set_cable",
                id: n.id,
                rest_length: parameters[0],
                stiffness: parameters[1],
                damping: parameters[2],
                max_tension: parameters[3],
                actuator_force: parameters[4],
              })
            }
          >
            Save parameters
          </button>
          <p className="hint">
            Stored in the model. Cable force parameters and muscle physiology
            are not yet preserved by MuJoCo export.
          </p>
        </section>
      )}
    </div>
  );
}
