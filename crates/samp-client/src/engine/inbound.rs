//! Inbound wire traffic: decode a packet or RPC, apply it to the world, the session or the shared
//! script state, and hand anything user-facing to the event queue. Everything here has the shape
//! `(id, payload) -> side effect`; nothing here sends.

use super::*;

impl<T: Transport> Engine<T> {
    pub(super) async fn on_rak_event(&mut self, event: RakEvent) {
        match event {
            RakEvent::Connected { body } => self.on_connected(&body).await,
            RakEvent::Rpc { id, payload } => self.on_rpc(id, &payload).await,
            RakEvent::Packet { data } => self.on_packet(&data),
            RakEvent::Disconnected(reason) => self.on_disconnect(reason),
        }
    }

    pub(super) async fn on_rpc(&mut self, id: u8, payload: &[u8]) {
        tracing::debug!(rpc_id = id, len = payload.len(), state = ?self.state, "inbound RPC");
        match id {
            RPC_WORLD_VEHICLE_ADD => self.world.vehicles.on_world_vehicle_add(payload),
            RPC_WORLD_VEHICLE_REMOVE => self.world.vehicles.on_world_vehicle_remove(payload),
            // The player pool's real lifetime. Without these a peer only exists because a sync packet
            // happened to arrive, and "streamed out" is indistinguishable from "gone quiet" — so the
            // walker keeps routing around someone the server already removed.
            PlayerStreamIn::RPC_ID => {
                if let Ok(ev) = decode_event::<PlayerStreamIn>(payload) {
                    self.world.players.stream_in(
                        ev.player_id,
                        ev.team,
                        ev.model,
                        ev.position,
                        Instant::now(),
                    );
                }
            }
            PlayerStreamOut::RPC_ID => {
                if let Ok(ev) = decode_event::<PlayerStreamOut>(payload) {
                    self.world.players.stream_out(ev.player_id);
                }
            }
            _ => {}
        }
        if let Some(decoded) = decode_inbound_rpc(id, payload) {
            tracing::debug!(target: "samp_client::engine", "  └─ {decoded}");
        }
        // Surface incoming dialogs in the console at info level (like server chat), independent of any
        // script — so every ShowDialog the server sends is visible without enabling debug logging.
        if id == RpcId::ShowDialog as u8 {
            if let Ok(d) = ShowDialog::decode(payload) {
                // Body holds the info text / list rows (options) for list/tablist dialogs; show them
                // indented under the header, one per line, so list dialogs are readable in the console.
                let body = decode_cp1251(&ShowDialog::body(payload));
                let options: String = body
                    .split('\n')
                    .filter(|row| !row.trim().is_empty())
                    .map(|row| format!("\n    • {}", row.trim_end()))
                    .collect();
                tracing::info!(
                    "dialog #{} (style {}): {:?}  [{} | {}]{}",
                    d.dialog_id,
                    d.style,
                    samp_proto::decode_cp1251(&d.title),
                    samp_proto::decode_cp1251(&d.button1),
                    samp_proto::decode_cp1251(&d.button2),
                    options,
                );
            }
        }
        // Registered handlers (scripts/observers) see the RPC first: a Drop consumes it before the
        // FSM, a Rewrite swaps the body the FSM then processes.
        let rewritten;
        let payload = match self.registry.dispatch_rpc(Direction::Incoming, id, payload) {
            Verdict::Pass => payload,
            Verdict::Drop => return,
            Verdict::Rewrite(bytes) => {
                rewritten = bytes;
                rewritten.as_slice()
            }
        };
        self.track_state(id, payload);
        self.track_dynamic_obstacles(id, payload);
        self.aim_follow(id, payload);
        self.emulation_incoming(id, payload).await;
        match (RpcId::try_from(id), &self.state) {
            (Ok(RpcId::InitGame), ConnectionState::Joining) => self.on_init_game(payload).await,
            // We re-request class every second pre-spawn, so class responses arrive in both
            // ClassSelection and ClassSelected; once we have spawn info the response requests spawn.
            (
                Ok(RpcId::RequestClass),
                ConnectionState::ClassSelection | ConnectionState::ClassSelected,
            ) => self.on_class_response(payload).await,
            // The server drives the spawn with `RequestSpawnResponse` while we're spectating
            // (ClassSelection/ClassSelected), or it answers an explicit `sampSpawnPlayer()` RequestSpawn.
            (
                Ok(RpcId::RequestSpawn),
                ConnectionState::ClassSelection | ConnectionState::ClassSelected,
            ) => self.on_spawn_response(payload).await,
            (Ok(RpcId::TogglePlayerSpectating), _) => self.on_toggle_spectating(payload).await,
            (Ok(RpcId::SetSpawnInfo), _) => self.on_set_spawn_info(payload).await,
            (Ok(RpcId::ConnectionRejected), _) => self.on_disconnect(DisconnectReason::Rejected),
            (Ok(RpcId::ClientMessage), _) => self.on_client_message(payload),
            (Ok(RpcId::Chat), _) => self.on_player_chat(payload),
            _ => {}
        }
    }

    /// Update the shared local-player state from money/interior/vehicle/camera RPCs so `getBotMoney`
    /// etc. return live values. Unknown ids and undecodable payloads are ignored.
    pub(super) fn track_state(&mut self, id: u8, payload: &[u8]) {
        use crate::state::InVehicleData;
        let Some(state) = &self.bot_state else {
            return;
        };
        let mut s = state.borrow_mut();
        match id {
            GivePlayerMoney::RPC_ID => {
                if let Ok(ev) = decode_event::<GivePlayerMoney>(payload) {
                    s.money += ev.money; // GivePlayerMoney is additive
                }
            }
            ResetPlayerMoney::RPC_ID => s.money = 0,
            // Without these the bot reports the spawn value forever while the server lowers it (Arizona's
            // hunger system), i.e. claims more health than the server assigned — the health-hack pattern.
            SetPlayerHealth::RPC_ID => {
                if let Ok(ev) = decode_event::<SetPlayerHealth>(payload) {
                    s.on_foot.health = ev.health.clamp(0.0, u8::MAX as f32) as u8;
                }
            }
            SetPlayerArmour::RPC_ID => {
                if let Ok(ev) = decode_event::<SetPlayerArmour>(payload) {
                    s.on_foot.armour = ev.armour.clamp(0.0, u8::MAX as f32) as u8;
                }
            }
            SetInterior::RPC_ID => {
                if let Ok(ev) = decode_event::<SetInterior>(payload) {
                    s.interior = ev.interior;
                }
            }
            PutPlayerInVehicle::RPC_ID => {
                if let Ok(ev) = decode_event::<PutPlayerInVehicle>(payload) {
                    s.vehicle = Some(InVehicleData {
                        id: ev.vehicle_id,
                        seat: ev.seat_id,
                        ..InVehicleData::default()
                    });
                }
            }
            RemovePlayerFromVehicle::RPC_ID => s.vehicle = None,
            SetCameraPosition::RPC_ID => {
                if let Ok(ev) = decode_event::<SetCameraPosition>(payload) {
                    s.camera_pos = ev.position;
                }
            }
            _ => {}
        }
    }

    /// Track streamed CreateObjects (RPC 44) and their moves/removals (45/47) — the ONLY visibility we
    /// have into a dynamic obstacle the static navmesh bake never saw (another player's dropped cart, a
    /// parked vehicle, ...). CreateObject's wire format has a variable-length materials/attach tail we
    /// don't need, so we decode only the fixed prefix — object id, model id (unused, kept for future
    /// filtering), position — matching the byte-exact layout `app`'s `objects` pcap-miner already uses.
    pub(super) fn track_dynamic_obstacles(&mut self, id: u8, payload: &[u8]) {
        if id == RPC_CREATE_OBJECT {
            if let Some((object_id, position)) = decode_create_object_prefix(payload) {
                self.world
                    .objects
                    .track(object_id, position, Instant::now());
            }
        } else if id == SetObjectPosition::RPC_ID {
            if let Ok(ev) = decode_event::<SetObjectPosition>(payload) {
                self.world
                    .objects
                    .track(ev.object_id, ev.position, Instant::now());
            }
        } else if id == DestroyObject::RPC_ID {
            if let Ok(ev) = decode_event::<DestroyObject>(payload) {
                self.world.objects.forget(ev.object_id);
            }
        }
    }

    /// Follow server repositions for aim-sync: `SetPlayerPos(FindZ)` while spawned, and the camera
    /// RPCs before spawn, move the bot and regenerate the aim (ported from `aim_fix_updated.lua`).
    pub(super) fn aim_follow(&mut self, id: u8, payload: &[u8]) {
        let spawned = matches!(self.state, ConnectionState::Spawned);
        let new_pos = match id {
            SetPlayerPos::RPC_ID if spawned => decode_event::<SetPlayerPos>(payload)
                .ok()
                .map(|ev| ev.position),
            SetPlayerPosFindZ::RPC_ID if spawned => decode_event::<SetPlayerPosFindZ>(payload)
                .ok()
                .map(|ev| ev.position),
            SetCameraPosition::RPC_ID if !spawned => decode_event::<SetCameraPosition>(payload)
                .ok()
                .map(|ev| ev.position),
            // `InterpolateCamera` moves us only when it carries a destination position.
            InterpolateCamera::RPC_ID if !spawned => decode_event::<InterpolateCamera>(payload)
                .ok()
                .and_then(|ev| ev.set_pos.then_some(ev.dest_pos)),
            _ => return,
        };
        let Some(new_pos) = new_pos else {
            return;
        };
        // A same-spot Z-only reposition while spawned is a slap: take the teleport, then fall back down
        // over several sync ticks — see [`SlapReaction`]/[`step_slap_reaction`].
        // WHY the interior gate: indoors, a same-X/Y jump of a few metres is a FLOOR CHANGE, which is
        // geometrically identical to a slap and must not be answered with a fall — the bot would sink
        // out of the room the server just put it in. The scripts draw the same line (flood.luau only
        // nudges outdoors, for the same reason).
        if spawned
            && self.slap.is_none()
            && self.interior() == 0
            && is_slap(self.sync.position, new_pos)
        {
            let ground_z = self.world.ground_below(self.sync.position);
            if new_pos.z > ground_z + SLAP_LANDED_EPS {
                self.sync.position = new_pos;
                self.mirror_to_state();
                self.slap = Some(SlapReaction {
                    ground_z,
                    fall_speed: 0.0,
                    last_step_at: Instant::now(),
                });
                return;
            }
        }
        self.sync.position = new_pos;
        // The body is authoritative while a walk runs, so a reposition that only touches `sync` is
        // undone on the next tick — the bot reports itself back onto its old corridor, which reads as
        // refusing the server's command. Move the body too, and re-seed the velocity baseline so the
        // teleport is not then reported as travel.
        self.motion.place(new_pos);
        if let Some(walk) = self.walk.as_mut() {
            walk.last_position = new_pos;
        }
        self.mirror_to_state();
        let in_vehicle = self.in_vehicle();
        self.aim
            .on_reposition(new_pos, self.sync.quaternion, in_vehicle);
    }

    /// Run standard-client emulation over an incoming RPC and send anything it produces.
    pub(super) async fn emulation_incoming(&mut self, id: u8, payload: &[u8]) {
        let Some(state) = self.bot_state.as_ref() else {
            return;
        };
        let msgs = self
            .emulation
            .on_incoming_rpc(&mut state.borrow_mut(), id, payload);
        self.send_outbound(msgs).await;
    }

    /// An incoming raw packet (`data[0]` = id). The client does not process incoming packets itself;
    /// the `onReceivePacket` chokepoint lets scripts observe (or, for the game, drop) them. With
    /// emulation on, a `VehicleSync` that targets the bot's own vehicle is treated as a hijack: the
    /// bot drops the vehicle and the packet.
    pub(super) fn on_packet(&mut self, data: &[u8]) {
        let Some(&id) = data.first() else {
            return;
        };
        tracing::debug!(packet_id = id, len = data.len(), "inbound packet");
        // Arizona 221/113 assigns our streamer entity id — capture it for the outbound 221/53 sync.
        if id == ARIZONA_SYNC_PACKET_ID && data.get(1) == Some(&ARIZONA_SYNC_ASSIGN_SUB) {
            if let (Some(&lo), Some(&hi)) = (data.get(3), data.get(4)) {
                let entity = u16::from_le_bytes([lo, hi]);
                if self.arizona_streamer_id != Some(entity) {
                    tracing::info!(entity, "arizona: assigned streamer entity id (221/113)");
                }
                self.arizona_streamer_id = Some(entity);
            }
        }
        if id == ARIZONA_CEF_PACKET_ID {
            tracing::debug!(
                sub_id = data.get(1).copied().unwrap_or(0),
                payload = %hex(data),
                "inbound 220",
            );
        }
        if id == SyncPacketId::PlayerSync as u8 {
            let mut r = samp_proto::BitStreamReader::new(&data[1..]);
            if let Ok(sync) = samp_proto::PlayerSyncData::decode(&mut r) {
                if sync.player_id != self.local_id.0 {
                    self.world.players.update(&sync, Instant::now());
                }
            }
        }
        // A vehicle streaming 200 has a driver; one streaming 209 already has an owner. Both veto us
        // claiming it. Inbound 209 is [id][u16 senderId][67-byte body] — 70 bytes total, verified
        // against 45,742 captured packets decoding with zero failures, so the body starts at 3.
        if id == SyncPacketId::UnoccupiedSync as u8 {
            // The 2 bytes we skip are the sender the server stamps on rebroadcast. Arizona is not
            // stock, so if it ever echoes ours back, feeding them in would make us veto ourselves and
            // silently collapse our own cadence for that vehicle from 10 Hz to once per staleness window.
            let sender = data
                .get(1..3)
                .map(|b| u16::from_le_bytes([b[0], b[1]]))
                .unwrap_or(u16::MAX);
            let mut r = samp_proto::BitStreamReader::new(&data[3.min(data.len())..]);
            if sender != self.local_id.0 {
                if let Ok(sync) = samp_proto::UnoccupiedSyncData::decode(&mut r) {
                    self.world
                        .vehicles
                        .on_unoccupied_sync_in(&sync, Instant::now());
                }
            }
        }
        if id == SyncPacketId::VehicleSync as u8 {
            let mut r = samp_proto::BitStreamReader::new(&data[1..]);
            if let Ok(sync) = VehicleSyncData::decode(&mut r) {
                let now = Instant::now();
                self.world.vehicles.on_vehicle_sync(&sync, now);
                // A player who is DRIVING streams 200, not 207, so they would age out of the peer
                // table and stop vetoing us — and the 209 election would then hand us a vehicle a
                // nearer driver already owns. Their vehicle's position is where they are.
                if sync.player_id != self.local_id.0 {
                    self.world.players.update(
                        &samp_proto::PlayerSyncData {
                            player_id: sync.player_id,
                            position: sync.position,
                            move_speed: sync.move_speed,
                            ..Default::default()
                        },
                        now,
                    );
                }
            }
            let hijack = self
                .bot_state
                .as_ref()
                .is_some_and(|state| self.emulation.is_vehicle_hijack(&state.borrow(), data));
            if hijack {
                if let Some(state) = self.bot_state.as_ref() {
                    state.borrow_mut().vehicle = None;
                }
                tracing::info!("emulation: refused a vehicle-sync hijack of our vehicle");
                return;
            }
        }
        let _ = self.registry.dispatch_packet(Direction::Incoming, id, data);
    }

    pub(super) fn on_client_message(&mut self, payload: &[u8]) {
        match ServerMessage::decode(payload) {
            Ok(msg) => self.pending.push_back(ClientEvent::ServerMessage {
                color: msg.color,
                text: decode_cp1251(&msg.text),
            }),
            Err(error) => tracing::trace!(%error, "failed to decode client message"),
        }
    }

    pub(super) fn on_player_chat(&mut self, payload: &[u8]) {
        match ChatMessage::decode(payload) {
            Ok(msg) => self.pending.push_back(ClientEvent::Chat {
                player_id: msg.player_id,
                text: decode_cp1251(&msg.text),
            }),
            Err(error) => tracing::trace!(%error, "failed to decode player chat"),
        }
    }
}
