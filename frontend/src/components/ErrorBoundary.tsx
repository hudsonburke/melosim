import { Component, type ReactNode } from "react";

interface Props { children: ReactNode; }
interface State { error: Error | null; }

export class ErrorBoundary extends Component<Props, State> {
  state: State = { error: null };

  static getDerivedStateFromError(error: Error): State {
    return { error };
  }

  render() {
    if (this.state.error) {
      return (
        <div style={{ padding: 24, color: "#ff4444", fontFamily: "monospace", background: "#1a1a1a", minHeight: "100vh" }}>
          <h2>React Error</h2>
          <pre style={{ whiteSpace: "pre-wrap" }}>{this.state.error.message}</pre>
          <pre style={{ color: "#888", fontSize: 12, whiteSpace: "pre-wrap" }}>{this.state.error.stack}</pre>
        </div>
      );
    }
    return this.props.children;
  }
}
