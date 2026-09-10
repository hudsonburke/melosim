use melosim::web_editor::{Command, EditorClient};
use std::io::Read;
fn main() {
    let server = tiny_http::Server::http("127.0.0.1:7421")
        .expect("cannot bind editor backend to 127.0.0.1:7421");
    let editor = EditorClient::start();
    eprintln!("melosim editor backend: http://127.0.0.1:7421");
    for mut request in server.incoming_requests() {
        // Development transport: no wildcard CORS, only JSON from the local UI.
        let origin = request
            .headers()
            .iter()
            .find(|h| h.field.equiv("Origin"))
            .map(|h| h.value.as_str());
        let allowed = origin.is_none()
            || matches!(
                origin,
                Some("http://127.0.0.1:5173" | "http://localhost:5173")
            );
        let json = request.headers().iter().any(|h| {
            h.field.equiv("Content-Type") && h.value.as_str().starts_with("application/json")
        });
        if !allowed
            || !json
            || request.method() != &tiny_http::Method::Post
            || request.url() != "/command"
        {
            let _ = request
                .respond(tiny_http::Response::from_string("Forbidden").with_status_code(403));
            continue;
        }
        let mut body = String::new();
        let result = request
            .as_reader()
            .take(1_048_577)
            .read_to_string(&mut body)
            .map_err(|e| e.to_string())
            .and_then(|_| {
                if body.len() > 1_048_576 {
                    Err("Request too large".into())
                } else {
                    Ok(())
                }
            })
            .and_then(|_| serde_json::from_str::<Command>(&body).map_err(|e| e.to_string()))
            .and_then(|c| editor.execute(c));
        let (status, body) = match result {
            Ok(s) => (200, serde_json::to_string(&s).unwrap()),
            Err(e) => (400, serde_json::json!({"error":e}).to_string()),
        };
        let _ = request.respond(
            tiny_http::Response::from_string(body)
                .with_status_code(status)
                .with_header(
                    tiny_http::Header::from_bytes("Content-Type", "application/json").unwrap(),
                ),
        );
    }
}
