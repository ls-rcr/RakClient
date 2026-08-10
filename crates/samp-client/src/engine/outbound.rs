//! Outbound wire traffic: sample the advanced state and emit whatever is due. Each send owns its own
//! cadence and adaptive gating, so this is where the send-rate machinery lives.

use super::*;

impl<T: Transport> Engine<T> {
    /// Send everything scripts queued since the last drain (through the `onSendPacket` chokepoint), each
    /// with its requested reliability/channel; see docs/memory/samp-client/engine.md#flush-outbox.
    pub(super) async fn flush_outbox(&mut self) {
        for msg in self.registry.drain_outbox() {
            let msg = match msg {
                OutboundMsg::Packet {
                    data,
                    reliability,
                    channel,
                } => {
                    let id = data.first().copied().unwrap_or(0);
                    let data = match self
                        .registry
                        .dispatch_packet(Direction::Outgoing, id, &data)
                    {
                        Verdict::Drop => continue,
                        Verdict::Pass => data,
                        Verdict::Rewrite(bytes) => bytes,
                    };
                    if id == ARIZONA_CEF_PACKET_ID {
                        tracing::debug!(
                            sub_id = data.get(1).copied().unwrap_or(0),
                            channel,
                            reliability,
                            payload = %hex(&data),
                            "outbound 220",
                        );
                    }
                    OutboundMsg::Packet {
                        data,
                        reliability,
                        channel,
                    }
                }
                OutboundMsg::Rpc { id, payload } => {
                    tracing::debug!(rpc_id = id, payload = %hex(&payload), "outbound RPC");
                    OutboundMsg::Rpc { id, payload }
                }
            };
            if let Err(error) = self.send_outbound_msg(msg).await {
                tracing::warn!(%error, "failed to send script-queued message");
                self.on_disconnect(DisconnectReason::ConnectionLost);
                return;
            }
        }
    }

    /// Send one queued wire message with the reliability/channel it carries.
    pub(super) async fn send_outbound_msg(&mut self, msg: OutboundMsg) -> raknet::Result<()> {
        match msg {
            OutboundMsg::Packet {
                data,
                reliability,
                channel,
            } => {
                self.transport
                    .send(data, Reliability::from_wire(reliability), channel)
                    .await
            }
            OutboundMsg::Rpc { id, payload } => self.transport.rpc(id, payload).await,
        }
    }

    /// Send emulation-produced packets/RPCs. These are the client's own, so they bypass the script
    /// chokepoints.
    pub(super) async fn send_outbound(&mut self, msgs: Vec<OutboundMsg>) {
        for msg in msgs {
            if let Err(error) = self.send_outbound_msg(msg).await {
                tracing::warn!(%error, "failed to send emulation message");
                self.on_disconnect(DisconnectReason::ConnectionLost);
                return;
            }
        }
    }

    /// Send a spectator-sync packet (212) — the pre-spawn keepalive while spectating.
    pub(super) async fn send_spectator_sync(&mut self) {
        let packet = SpectatorSync {
            position: self.sync.position,
        }
        .to_packet();
        // Spectator sync goes UNRELIABLE (reliability 6, not sequenced) on the sync channel; see
        // docs/memory/samp-client/engine.md#send-spectator-sync.
        if let Err(error) = self
            .transport
            .send(packet, Reliability::Unreliable, SYNC_CHANNEL)
            .await
        {
            tracing::warn!(%error, "failed to send spectator sync");
            self.on_disconnect(DisconnectReason::ConnectionLost);
        }
    }

    /// One sync cycle: pre-spawn it spectates, spawned it runs the on-foot/weapons/aim sends. Any
    /// transport failure tears the connection down once.
    pub(super) async fn on_sync_tick(&mut self, force: bool) {
        let now = Instant::now();
        if self
            .last_heartbeat_at
            .is_none_or(|at| now.duration_since(at) >= HEARTBEAT_INTERVAL)
        {
            tracing::info!(
                state = ?self.state,
                position = ?self.sync.position,
                walking = self.walk.is_some(),
                "heartbeat"
            );
            self.last_heartbeat_at = Some(now);
        }
        // Arizona pre-spawn: spectate (keepalive) while awaiting the login dialog, not on-foot sync.
        if self.spectating && !matches!(self.state, ConnectionState::Spawned) {
            self.send_spectator_sync().await;
            self.send_prespawn_poll().await;
            return;
        }
        if self.run_spawned_sync(force).await.is_err() {
            self.on_disconnect(DisconnectReason::ConnectionLost);
        }
    }

    /// One spawned tick: advance the local simulation, then send whatever is due from it.
    pub(super) async fn run_spawned_sync(&mut self, force: bool) -> raknet::Result<()> {
        self.advance_local(Instant::now());
        self.send_all(force).await
    }

    /// Sample the advanced state onto the wire: each send owns its own cadence and adaptive gating.
    /// Bails on the first transport error (the caller disconnects).
    pub(super) async fn send_all(&mut self, force: bool) -> raknet::Result<()> {
        // Republish what scripts can see, on the send cadence: they poll, so a snapshot per frame is
        // exactly as fresh as the packets built from the same state.
        self.entities.publish(self.world.snapshot(Instant::now()));
        // In a vehicle (script put us there via setBotVehicle / server PutPlayerInVehicle), the server
        // expects vehicle sync (200) at the driver seat, not on-foot 207 — enables the taxi/bus/driving quests.
        if self.in_vehicle() {
            self.send_vehicle_sync(force).await?;
        } else {
            self.send_on_foot_sync(force).await?;
            self.send_unoccupied_syncs().await;
        }
        self.send_arizona_221_sync().await?;
        self.send_weapons_update().await?;
        self.send_stats_update().await?;
        self.send_aim_sync().await
    }

    /// Stream Arizona's custom on-foot position report (packet 221, sub-id 53) alongside stock 207,
    /// required by the AC; see docs/memory/samp-client/engine.md#send-arizona-221.
    pub(super) async fn send_arizona_221_sync(&mut self) -> raknet::Result<()> {
        // Report the server-assigned streamer id (221/113), falling back to our player id; the wrong id
        // is the "function 52" flag. See docs/memory/samp-client/engine.md#send-arizona-221.
        let packet = ArizonaSync221 {
            entity_id: self.arizona_streamer_id.unwrap_or(self.local_id.0),
            position: self.sync.position,
            timestamp_ms: ARIZONA_TS_BASE
                .wrapping_add(self.created_at.elapsed().as_millis() as u32),
            velocity: ArizonaSync221::REST_VELOCITY,
            heading: ArizonaSync221::REST_HEADING,
        }
        .encode();
        self.transport
            .send(packet, Reliability::UnreliableSequenced, SYNC_CHANNEL)
            .await
            .inspect_err(|error| tracing::warn!(%error, "failed to send Arizona 221 sync"))
    }

    /// Report player stats (`PACKET_STATS_UPDATE` = 205) at the [`STATS_INTERVAL`] 1 Hz cadence,
    /// UNRELIABLE on channel 0; see docs/memory/samp-client/engine.md#send-stats-update.
    pub(super) async fn send_stats_update(&mut self) -> raknet::Result<()> {
        let now = Instant::now();
        if self
            .last_stats_at
            .is_some_and(|at| now.duration_since(at) < STATS_INTERVAL)
        {
            return Ok(());
        }
        let money = self
            .bot_state
            .as_ref()
            .map_or(0, |state| state.borrow().money);
        let packet = StatsUpdate {
            money,
            drunk_level: 0,
        }
        .to_packet();
        self.transport
            .send(packet, Reliability::Unreliable, 0)
            .await
            .inspect_err(|error| tracing::warn!(%error, "failed to send stats update"))?;
        self.last_stats_at = Some(now);
        Ok(())
    }

    /// Emulation adjustments folded into the outgoing on-foot state: report the held weapon and the
    /// occasional score-ping key blip.
    pub(super) fn adjust_on_foot_from_emulation(&mut self, now: Instant) {
        if let Some(state) = self.bot_state.as_ref() {
            let (weapon, keys) =
                self.emulation
                    .adjust_on_foot(&state.borrow(), self.sync.keys, now);
            self.sync.weapon = WeaponId(weapon);
            self.sync.keys = keys;
        }
    }

    /// The on-foot packet is sent adaptively — on any state change, on a forced sync
    /// (`updateSync()`), or otherwise only at the [`IDLE_SYNC_INTERVAL`] keepalive cadence — so a
    /// stationary bot stops flooding identical packets.
    pub(super) async fn send_on_foot_sync(&mut self, force: bool) -> raknet::Result<()> {
        let now = Instant::now();
        let due = force
            || self.sync != self.last_sync
            || self
                .last_sync_at
                .is_none_or(|at| now.duration_since(at) >= IDLE_SYNC_INTERVAL);
        if !due {
            return Ok(());
        }
        let packet = self.sync.to_packet();
        // The sync packet passes through `onSendPacket` so scripts (e.g. aim-fix) can edit/drop it.
        let to_send = match self.registry.dispatch_packet(
            Direction::Outgoing,
            SyncPacketId::PlayerSync as u8,
            &packet,
        ) {
            Verdict::Drop => None,
            Verdict::Pass => Some(packet),
            Verdict::Rewrite(bytes) => Some(bytes),
        };
        if let Some(packet) = to_send {
            self.transport
                .send(packet, Reliability::UnreliableSequenced, SYNC_CHANNEL)
                .await
                .inspect_err(|error| tracing::warn!(%error, "failed to send on-foot sync"))?;
            self.last_sync = self.sync;
            self.last_sync_at = Some(now);
        }
        Ok(())
    }

    /// Stream any driverless vehicle we are the closest player to (209). A real client does this
    /// continuously and we did not, which is directly observable server-side wherever we stand — at
    /// the sawmill 24.3% of captured 209s come from the cart rows we work in. The election lives in
    /// [`VehicleTable`]; it stays silent unless we strictly win, so a tie or an unseen pose sends nothing.
    pub(super) async fn send_unoccupied_syncs(&mut self) {
        let now = Instant::now();
        let due = self.world.vehicles.due_unoccupied_sync(
            now,
            self.sync.position,
            self.world.players.iter_fresh(now),
        );
        for sync in due {
            let packet = sync.to_packet();
            if self
                .transport
                .send(packet, Reliability::UnreliableSequenced, SYNC_CHANNEL)
                .await
                .inspect_err(|error| tracing::warn!(%error, "failed to send unoccupied sync"))
                .is_err()
            {
                return;
            }
        }
    }

    /// The in-vehicle counterpart of [`send_on_foot_sync`]: while the bot is in a vehicle, emit
    /// VehicleSync (200) at the live pose (`self.sync` position/quaternion/velocity/keys) with the
    /// vehicle id + health from the shared state. Same adaptive gating (resend on change or at the
    /// idle keepalive cadence). Steering/keys are near-zero for the teleport-driven quest bot.
    pub(super) async fn send_vehicle_sync(&mut self, force: bool) -> raknet::Result<()> {
        let (vehicle_id, vehicle_health) = self
            .bot_state
            .as_ref()
            .and_then(|s| s.borrow().vehicle.map(|v| (v.id, v.health)))
            .unwrap_or((0, 1000));
        let data = VehicleSyncData {
            player_id: self.local_id.0,
            vehicle_id,
            left_right_keys: self.sync.left_right as u16,
            up_down_keys: self.sync.up_down as u16,
            keys_data: self.sync.keys,
            quaternion: self.sync.quaternion,
            position: self.sync.position,
            move_speed: self.sync.move_speed,
            // A freshly-recorded vehicle (default health 0) would read as destroyed — report full health.
            vehicle_health: if vehicle_health == 0 {
                1000
            } else {
                vehicle_health
            },
            player_health: self.sync.health,
            armor: self.sync.armour,
            ..Default::default()
        };
        let now = Instant::now();
        let due = force
            || data != self.last_vehicle_sync
            || self
                .last_vehicle_sync_at
                .is_none_or(|at| now.duration_since(at) >= IDLE_SYNC_INTERVAL);
        if !due {
            return Ok(());
        }
        let packet = data.to_packet();
        let to_send = match self.registry.dispatch_packet(
            Direction::Outgoing,
            SyncPacketId::VehicleSync as u8,
            &packet,
        ) {
            Verdict::Drop => None,
            Verdict::Pass => Some(packet),
            Verdict::Rewrite(bytes) => Some(bytes),
        };
        if let Some(packet) = to_send {
            self.transport
                .send(packet, Reliability::UnreliableSequenced, SYNC_CHANNEL)
                .await
                .inspect_err(|error| tracing::warn!(%error, "failed to send vehicle sync"))?;
            self.last_vehicle_sync = data;
            self.last_vehicle_sync_at = Some(now);
        }
        Ok(())
    }

    /// Emulation: stream the weapon inventory periodically.
    pub(super) async fn send_weapons_update(&mut self) -> raknet::Result<()> {
        let msg = match self.bot_state.as_ref() {
            Some(state) => self
                .emulation
                .due_weapons_update(&state.borrow(), Instant::now()),
            None => None,
        };
        match msg {
            Some(msg) => self
                .send_outbound_msg(msg)
                .await
                .inspect_err(|error| tracing::warn!(%error, "failed to send weapons update")),
            None => Ok(()),
        }
    }

    /// Native aim-sync: note the position (to detect movement) and send a believable aim when due.
    pub(super) async fn send_aim_sync(&mut self) -> raknet::Result<()> {
        self.aim.on_position(self.sync.position);
        let aim_packet = if self.aim.due(Instant::now()) {
            if let Some(state) = self.bot_state.as_ref() {
                self.emulation
                    .spoof_aim(self.aim.aim_mut(), &state.borrow());
            }
            Some(self.aim.encode())
        } else {
            None
        };
        self.mirror_aim_to_state();
        match aim_packet {
            Some(packet) => self
                .transport
                .send(packet, Reliability::UnreliableSequenced, SYNC_CHANNEL)
                .await
                .inspect_err(|error| tracing::warn!(%error, "failed to send aim sync")),
            None => Ok(()),
        }
    }

    pub(super) async fn send_rpc(&mut self, rpc_id: u8, payload: Vec<u8>) {
        // Registered handlers see outgoing RPCs too (the `onSend*` events): Drop cancels the send,
        // Rewrite changes the body that goes on the wire.
        let payload = match self
            .registry
            .dispatch_rpc(Direction::Outgoing, rpc_id, &payload)
        {
            Verdict::Pass => payload,
            Verdict::Drop => return,
            Verdict::Rewrite(bytes) => bytes,
        };
        if let Err(error) = self.transport.rpc(rpc_id, payload).await {
            tracing::warn!(rpc_id, %error, "failed to send rpc");
            self.on_disconnect(DisconnectReason::ConnectionLost);
        }
    }

    /// Send a chat line as the local player (`RPC_Chat`, client→server `[u8 len][text]`). `text` is
    /// raw bytes in the server's encoding (cp1251 for Arizona); callers transcode before calling.
    pub(crate) async fn send_chat(&mut self, text: &[u8]) {
        let payload = ChatOutgoing { text }.encode();
        self.send_rpc(RpcId::Chat as u8, payload).await;
    }
}
