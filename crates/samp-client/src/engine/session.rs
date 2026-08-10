//! Connection lifecycle: the join handshake, class/spawn negotiation, the pre-spawn
//! spectate window, and the disconnect/reconnect policy. This is the only group that owns
//! `spawn_requested`, `server_spectating`, `spawn_info_position`, `class_selected_at` and
//! `reconnect_attempts`.

use super::*;

impl<T: Transport> Engine<T> {
    pub(crate) async fn disconnect(&mut self) -> raknet::Result<()> {
        self.sync_timer = None;
        self.reconnect_timer = None;
        self.closed = true;
        self.transport.disconnect().await
    }

    pub(super) async fn on_connected(&mut self, body: &[u8]) {
        let (player_id, cookie) = match samp_proto::parse_connect(body) {
            Ok(parsed) => parsed,
            Err(error) => {
                tracing::warn!(%error, "failed to parse connect body");
                self.on_disconnect(DisconnectReason::Rejected);
                return;
            }
        };
        self.local_id = player_id;
        let challenge_response = cookie.0 ^ CHALLENGE_XOR;
        let gpci = self
            .config
            .gpci
            .clone()
            .unwrap_or_else(samp_proto::generate_gpci);
        let payload = {
            let join = ClientJoin {
                version: SAMP_VERSION_0_3_7,
                modded: false,
                nick: self.config.nick.as_str(),
                challenge_response,
                auth: gpci.as_str(),
                client_version: self.config.client_version.as_str(),
                // Append the trailing `challengeResponse2` (7th field) whenever a script is attached so a
                // Lua `onSendClientJoin` rewrite can keep it; vanilla servers ignore the trailing bytes.
                duplicate_challenge_response: self.bot_state.is_some(),
            };
            join.encode()
        };
        self.transition(ConnectionState::RakNetConnected);
        self.pending.push_back(ClientEvent::Connected);
        // The join goes through the `onSendRPC` chokepoint so a script can rewrite it before the wire;
        // scripts send their own post-join packets on `onConnect` (outbox flushed after this step).
        self.send_rpc(RpcId::ClientJoin as u8, payload).await;
        self.registry.dispatch_lifecycle("onConnect");
        self.transition(ConnectionState::Joining);
    }

    pub(super) async fn on_init_game(&mut self, payload: &[u8]) {
        let Some(init) = decode_or_warn::<InitGame>(payload, "init game") else {
            return;
        };
        self.local_id = init.local_player_id;
        let host_name = init.host_name;
        self.transition(ConnectionState::Joined {
            local_id: self.local_id,
            host_name: host_name.clone(),
        });
        self.pending.push_back(ClientEvent::Joined {
            local_id: self.local_id,
            host_name,
        });
        // A script's post-join packets fire on `onInitGame` (or its own Lua `wait()` task) while the
        // normal class → spawn flow proceeds immediately — validation is window-gated, not pre-spawn.
        self.registry.dispatch_lifecycle("onInitGame");
        self.flush_outbox().await;
        self.request_spawn_class().await;
    }

    /// After join, spectate and send `RequestClass`; the spawn is later driven by the class response,
    /// `RequestSpawnResponse(allow==2)`, or `TogglePlayerSpectating(1→0)`.
    pub(super) async fn request_spawn_class(&mut self) {
        self.spectating = true;
        self.transition(ConnectionState::ClassSelection);
        let payload = RequestClass {
            class: self.config.default_class,
        }
        .encode();
        self.send_rpc(RpcId::RequestClass as u8, payload).await;
    }

    /// Send `RequestSpawn` (RPC) and mark that we asked, so a non-`allow==2` response still spawns us.
    pub(super) async fn request_spawn(&mut self) {
        self.spawn_requested = true;
        let payload = RequestSpawn.encode();
        self.send_rpc(RpcId::RequestSpawn as u8, payload).await;
    }

    /// Pre-spawn tick while spectating: only the optional self-spawn fallback runs (no periodic
    /// `RequestClass`/heartbeat — anti-bot fingerprint). See docs/memory/samp-client/engine.md#send-prespawn-poll.
    pub(super) async fn send_prespawn_poll(&mut self) {
        // The self-spawn fallback (`config.self_spawn_timeout`) is opt-in and OFF by default (an
        // unauthorised RPC_Spawn trips Arizona's AC). See docs/memory/samp-client/engine.md#send-prespawn-poll.
        let Some(timeout) = self.config.self_spawn_timeout else {
            return;
        };
        if let Some(t) = self.class_selected_at {
            if Instant::now().duration_since(t) >= timeout {
                tracing::warn!(
                    "server never drove the spawn — spawning without server authorisation"
                );
                self.spectating = false;
                self.enter_spawned().await;
            }
        }
    }

    pub(super) async fn on_class_response(&mut self, payload: &[u8]) {
        let Some(response) = decode_or_warn::<RequestClassResponse>(payload, "class response")
        else {
            return;
        };
        if response.allowed {
            self.sync.position = response.spawn_position;
            self.mirror_to_state();
        }
        self.transition(ConnectionState::ClassSelected);
        self.class_selected_at.get_or_insert_with(Instant::now);
        // On a class response, request spawn only once `SetSpawnInfo` has arrived (login done); doing so
        // early earns "ОШИБКА 7721". See docs/memory/samp-client/engine.md#on-class-response.
        if self.spawn_info_position.is_some() && self.spectating {
            self.request_spawn().await;
        }
    }

    pub(super) async fn on_spawn_response(&mut self, payload: &[u8]) {
        let Some(response) = decode_or_warn::<RequestSpawnResponse>(payload, "spawn response")
        else {
            return;
        };
        // Spawn when `allow == 2`, or `allow != 0` while we explicitly requested spawn; Arizona sends
        // `allow == 2` after login. See docs/memory/samp-client/engine.md#on-spawn-response.
        let allow = response.allow;
        if allow != 2 && !(allow != 0 && self.spawn_requested) {
            tracing::debug!(allow, "server has not authorised spawn yet");
            return;
        }
        self.spectating = false;
        self.spawn_requested = false;
        self.enter_spawned().await;
    }

    /// `SetSpawnInfo` (RPC 68): remember the spawn position for `enter_spawned`, like RakSAMP Lite's
    /// `has_spawn_info` + `Net_Spawn` position copy.
    pub(super) async fn on_set_spawn_info(&mut self, payload: &[u8]) {
        let Some(info) = decode_event_or_warn::<SetSpawnInfo>(payload, "spawn info") else {
            return;
        };
        let pos = info.position;
        tracing::debug!(x = pos.x, y = pos.y, z = pos.z, "spawn info position");
        self.spawn_info_position = Some(pos);
        // Re-send Spawn (RPC 52) + adopt the position only when ALREADY Spawned (the post-skin second
        // SetSpawnInfo), without re-firing the event. See docs/memory/samp-client/engine.md#on-set-spawn-info-respawn.
        if matches!(self.state, ConnectionState::Spawned) {
            let spawn = Spawn.encode();
            self.send_rpc(RpcId::Spawn as u8, spawn).await;
            self.sync.position = pos;
            self.mirror_to_state();
            tracing::info!(
                x = pos.x,
                y = pos.y,
                z = pos.z,
                "re-spawn (RPC 52) on post-skin SetSpawnInfo"
            );
        }
    }

    /// `TogglePlayerSpectating` (RPC 124): a server `1 → 0` transition is the server-driven spawn; see
    /// docs/memory/samp-client/engine.md#on-toggle-spectating.
    pub(super) async fn on_toggle_spectating(&mut self, payload: &[u8]) {
        let Some(toggle) =
            decode_event_or_warn::<TogglePlayerSpectating>(payload, "spectating toggle")
                .map(|ev| ev.state)
        else {
            return;
        };
        let was_spectating = self.server_spectating;
        self.server_spectating = toggle;
        if toggle {
            // Arm the self-spawn fallback timer: spectate-on may be the only pre-spawn-window signal.
            // See docs/memory/samp-client/engine.md#on-toggle-spectating.
            self.class_selected_at.get_or_insert_with(Instant::now);
        }
        if was_spectating && !toggle && !matches!(self.state, ConnectionState::Spawned) {
            // A `1 → 0` toggle routes through `RequestSpawn` (RPC 129), NOT a direct `Spawn` (RPC 52);
            // the unsolicited-Spawn shortcut is the "function 52" AC kick. See docs/memory/samp-client/engine.md#on-toggle-spectating.
            self.request_spawn().await;
        }
    }

    /// Send `RPC_Spawn` and enter `Spawned` — the single convergence point for every spawn trigger; see
    /// docs/memory/samp-client/engine.md#enter-spawned.
    pub(super) async fn enter_spawned(&mut self) {
        let payload = Spawn.encode();
        self.send_rpc(RpcId::Spawn as u8, payload).await;
        // Net_Spawn copies the SetSpawnInfo position into the sync position (class-response
        // spawn_position is usually zero). See docs/memory/samp-client/engine.md#enter-spawned.
        match self.spawn_info_position {
            Some(pos) => self.sync.position = pos,
            None => {
                tracing::warn!(
                    "spawning without SetSpawnInfo — using the captured fallback position"
                );
                self.sync.position = FALLBACK_SPAWN_POSITION;
            }
        }
        self.sync.health = SPAWN_HEALTH;
        self.sync.quaternion = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        self.transition(ConnectionState::Spawned);
        self.mirror_to_state();
        let in_vehicle = self.in_vehicle();
        self.aim.arm(
            Instant::now(),
            self.sync.position,
            self.sync.quaternion,
            in_vehicle,
        );
        self.pending.push_back(ClientEvent::Spawned);
    }

    pub(super) async fn on_reconnect(&mut self) {
        self.reconnect_timer = None;
        self.closed = false;
        match self.transport.reconnect().await {
            Ok(()) => {
                self.local_id = PlayerId::default();
                self.sync = OnFootSync::default();
                self.last_sync = OnFootSync::default();
                self.last_sync_at = None;
                self.last_stats_at = None;
                self.spawn_info_position = None;
                self.class_selected_at = None;
                self.transition(ConnectionState::Connecting);
            }
            Err(error) => {
                tracing::warn!(%error, "reconnect attempt failed");
                self.on_disconnect(DisconnectReason::ConnectionLost);
            }
        }
    }

    pub(super) fn on_transport_closed(&mut self) {
        self.closed = true;
        if self.reconnect_timer.is_none() && self.state != ConnectionState::Disconnected {
            self.on_disconnect(DisconnectReason::ConnectionLost);
        }
    }

    pub(super) fn on_disconnect(&mut self, reason: DisconnectReason) {
        // A dropped connection ends any active walk; the next spawn starts clean.
        self.stop_walk();
        self.world.clear_entities();
        self.aim.reset();
        if let Some(state) = self.bot_state.as_ref() {
            self.emulation.reset(&mut state.borrow_mut());
        }
        self.transition(ConnectionState::Disconnected);
        self.pending
            .push_back(ClientEvent::Disconnected(reason.to_string()));

        // Local quit: never reconnect.
        if reason == DisconnectReason::Local {
            return;
        }

        // Terminal reason: a bad password is a real credential fault — reconnecting IS futile.
        // See docs/memory/samp-client/engine.md#on-disconnect-reconnect.
        if reason == DisconnectReason::InvalidPassword {
            tracing::warn!(reason = %reason, "terminal disconnect — not reconnecting; stopping");
            self.reconnect_timer = None;
            self.closed = true;
            return;
        }

        // A `Banned` kick is NOT terminal when we egress through a rotating proxy: SA-MP bans the PROXY
        // EXIT IP, not the account (the account is registered and unflagged — the supervisor has always
        // reused the nick on this). It's a false positive from a soured exit, so retry: reconnect re-runs
        // the full connect, and establish_proxied draws a FRESH `-session-<id>` via rotate_session — a new
        // exit IP, on which the ban does not apply. Treating it as terminal closed the client outright and
        // left the bot dead whenever the supervisor wasn't alive to relaunch it (observed live).
        if reason == DisconnectReason::Banned {
            tracing::warn!(
                "banned = proxy exit IP banned (false positive), not the account — reconnecting on a fresh proxy session"
            );
        }

        // Everything else (kick/rejection/loss/timeout) is retried with a cap; a session lasting
        // ≥STABLE_SESSION resets the counter. See docs/memory/samp-client/engine.md#on-disconnect-reconnect.
        let was_stable = self
            .connected_since
            .is_some_and(|since| since.elapsed() >= STABLE_SESSION);
        if was_stable {
            self.reconnect_attempts = 0;
        }
        self.connected_since = None;
        self.reconnect_attempts += 1;
        if self.reconnect_attempts > MAX_RECONNECT_ATTEMPTS {
            tracing::warn!(
                attempts = self.reconnect_attempts,
                reason = %reason,
                "giving up after repeated reconnects (kicked/blocked) — stopping"
            );
            self.reconnect_timer = None;
            self.closed = true;
            return;
        }
        tracing::info!(
            attempt = self.reconnect_attempts,
            max = MAX_RECONNECT_ATTEMPTS,
            reason = %reason,
            "scheduling reconnect"
        );
        self.schedule_reconnect();
    }

    pub(super) fn schedule_reconnect(&mut self) {
        if self.reconnect_timer.is_none() {
            let delay = self.config.reconnect_delay;
            self.reconnect_timer = Some(Box::pin(time::sleep(delay)));
        }
    }
}
