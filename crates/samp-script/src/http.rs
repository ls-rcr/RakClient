//! Async HTTP for scripts (the captcha solver, Telegram reporting). `httpFetch` spawns a `reqwest`
//! request on the host's tokio runtime and stashes the result under an id; `httpPoll` lets the Lua
//! task scheduler (`newTask`/`wait`) collect it without blocking the single-threaded VM. The
//! `http.luau` module wraps the pair as `asyncHttpRequest(method, url, args, resolve, reject)`,
//! matching the stock MoonLoader shape so ported scripts change minimally.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use mlua::{Lua, Table};

/// Completed request outcomes keyed by the id `httpFetch` returned: `Ok(body)` or `Err(message)`.
/// A missing key means the request is still in flight (or the id was already collected).
type Results = Arc<Mutex<HashMap<u64, Result<String, String>>>>;

pub fn install_http(lua: &Lua) -> mlua::Result<()> {
    let globals = lua.globals();
    let results: Results = Arc::new(Mutex::new(HashMap::new()));
    let next_id = Arc::new(AtomicU64::new(1));
    let client = reqwest::Client::builder()
        .build()
        .map_err(|error| mlua::Error::runtime(format!("http client: {error}")))?;

    // httpFetch(method, url, args?) -> id. args = { headers = {name=value,..}, data = "body", timeout = ms }.
    // Returns immediately with the id; the request runs on the tokio runtime and lands in `results`.
    let r = results.clone();
    let ids = next_id.clone();
    let cl = client.clone();
    let fetch = lua.create_function(
        move |_, (method, url, args): (String, String, Option<Table>)| {
            let id = ids.fetch_add(1, Ordering::Relaxed);
            let method = reqwest::Method::from_bytes(method.to_uppercase().as_bytes())
                .unwrap_or(reqwest::Method::GET);
            let mut headers: Vec<(String, String)> = Vec::new();
            let mut body: Option<String> = None;
            let mut timeout_ms: u64 = 20_000;
            if let Some(t) = args {
                if let Ok(h) = t.get::<Table>("headers") {
                    for pair in h.pairs::<String, String>().flatten() {
                        headers.push(pair);
                    }
                }
                if let Ok(Some(d)) = t.get::<Option<String>>("data") {
                    body = Some(d);
                }
                if let Ok(Some(ms)) = t.get::<Option<u64>>("timeout") {
                    timeout_ms = ms;
                }
            }
            let results = r.clone();
            let client = cl.clone();
            match tokio::runtime::Handle::try_current() {
                Ok(handle) => {
                    handle.spawn(async move {
                        let outcome =
                            do_request(&client, method, &url, headers, body, timeout_ms).await;
                        results
                            .lock()
                            .expect("http results poisoned")
                            .insert(id, outcome);
                    });
                }
                Err(_) => {
                    results
                        .lock()
                        .expect("http results poisoned")
                        .insert(id, Err("no tokio runtime".into()));
                }
            }
            Ok(id)
        },
    )?;
    globals.set("httpFetch", fetch)?;

    // httpPoll(id) -> (status, text). status ∈ {"pending","ok","err"}; collecting a finished request
    // removes it. Poll from a newTask loop until it stops returning "pending".
    let r = results.clone();
    let poll = lua.create_function(move |_, id: u64| {
        let mut map = r.lock().expect("http results poisoned");
        Ok(match map.remove(&id) {
            None => ("pending".to_string(), String::new()),
            Some(Ok(text)) => ("ok".to_string(), text),
            Some(Err(err)) => ("err".to_string(), err),
        })
    })?;
    globals.set("httpPoll", poll)?;
    Ok(())
}

async fn do_request(
    client: &reqwest::Client,
    method: reqwest::Method,
    url: &str,
    headers: Vec<(String, String)>,
    body: Option<String>,
    timeout_ms: u64,
) -> Result<String, String> {
    let mut req = client
        .request(method, url)
        .timeout(Duration::from_millis(timeout_ms));
    for (name, value) in headers {
        req = req.header(name, value);
    }
    if let Some(body) = body {
        req = req.body(body);
    }
    let resp = req
        .send()
        .await
        .map_err(|error| format!("request: {error}"))?;
    resp.text().await.map_err(|error| format!("body: {error}"))
}
