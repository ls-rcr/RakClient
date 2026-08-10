use super::*;

use crate::world::motion::TURN_RATE;
use crate::world::PED_GROUND_OFFSET;

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

/// One wire message in the order it actually left the client. RPCs and packets go out through
/// different transport calls, so their relative order is invisible in the separate `rpcs`/`packets`
/// logs — and that order is load-bearing (see `script_rpcs_reach_the_wire_before_the_forced_sync`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Wire {
    Rpc(u8),
    Packet(u8),
}

#[derive(Default)]
struct TransportLog {
    rpcs: Vec<(u8, Vec<u8>)>,
    packets: Vec<Vec<u8>>,
    sequence: Vec<Wire>,
    disconnects: usize,
    reconnects: usize,
}

impl TransportLog {
    fn rpc_ids(&self) -> Vec<u8> {
        self.rpcs.iter().map(|(id, _)| *id).collect()
    }

    fn sync_count(&self) -> usize {
        self.on_foot().count()
    }

    fn on_foot(&self) -> impl Iterator<Item = &Vec<u8>> {
        self.packets
            .iter()
            .filter(|packet| packet.first() == Some(&(SyncPacketId::PlayerSync as u8)))
    }
}

/// Field accessors over a sent `[207][OnFootSync]` packet. Body layout (`OnFootSync::encode`):
/// left_right(2) up_down(2) keys(2) position(12) quaternion(16) health armour weapon special(4)
/// move_speed(12) surfing(12) surf_id(2) anim_id(2) anim_flags(2) = 68, after the 1-byte id.
mod on_foot {
    pub(super) fn u16_at(packet: &[u8], offset: usize) -> u16 {
        u16::from_le_bytes([packet[offset], packet[offset + 1]])
    }

    pub(super) fn f32_at(packet: &[u8], offset: usize) -> f32 {
        f32::from_le_bytes([
            packet[offset],
            packet[offset + 1],
            packet[offset + 2],
            packet[offset + 3],
        ])
    }

    pub(super) fn up_down(packet: &[u8]) -> i16 {
        u16_at(packet, 3) as i16
    }

    pub(super) fn keys(packet: &[u8]) -> u16 {
        u16_at(packet, 5)
    }

    pub(super) fn position(packet: &[u8]) -> [f32; 3] {
        [f32_at(packet, 7), f32_at(packet, 11), f32_at(packet, 15)]
    }

    pub(super) fn move_speed(packet: &[u8]) -> [f32; 3] {
        [f32_at(packet, 39), f32_at(packet, 43), f32_at(packet, 47)]
    }

    pub(super) fn animation(packet: &[u8]) -> (u16, u16) {
        (u16_at(packet, 65), u16_at(packet, 67))
    }
}

struct ScriptedTransport {
    script: VecDeque<RakEvent>,
    pend_when_empty: bool,
    log: Arc<Mutex<TransportLog>>,
}

impl ScriptedTransport {
    fn new(script: Vec<RakEvent>, pend_when_empty: bool) -> (Self, Arc<Mutex<TransportLog>>) {
        let log = Arc::new(Mutex::new(TransportLog::default()));
        let transport = Self {
            script: script.into(),
            pend_when_empty,
            log: log.clone(),
        };
        (transport, log)
    }
}

impl Transport for ScriptedTransport {
    async fn send(
        &self,
        data: Vec<u8>,
        _reliability: Reliability,
        _channel: u8,
    ) -> raknet::Result<()> {
        let mut log = self.log.lock().expect("log poisoned");
        log.sequence
            .push(Wire::Packet(data.first().copied().unwrap_or(0)));
        log.packets.push(data);
        Ok(())
    }

    async fn rpc(&self, rpc_id: u8, payload: Vec<u8>) -> raknet::Result<()> {
        let mut log = self.log.lock().expect("log poisoned");
        log.sequence.push(Wire::Rpc(rpc_id));
        log.rpcs.push((rpc_id, payload));
        Ok(())
    }

    async fn disconnect(&self) -> raknet::Result<()> {
        self.log.lock().expect("log poisoned").disconnects += 1;
        Ok(())
    }

    async fn recv(&mut self) -> Option<RakEvent> {
        if let Some(event) = self.script.pop_front() {
            Some(event)
        } else if self.pend_when_empty {
            future::pending().await
        } else {
            None
        }
    }

    async fn reconnect(&mut self) -> raknet::Result<()> {
        self.log.lock().expect("log poisoned").reconnects += 1;
        Ok(())
    }
}

fn test_config() -> ClientConfig {
    ClientConfig::builder(
        "127.0.0.1:7777".parse::<SocketAddr>().expect("addr"),
        "Tester",
    )
    .sync_interval(Duration::from_millis(100))
    .reconnect_delay(Duration::from_secs(5))
    .build()
}

/// A `CONNECTION_REQUEST_ACCEPTED` body the real [`samp_proto::parse_connect`] accepts (12 bytes:
/// `[u32 ip][u16 port][u16 playerId][u32 cookie]`). The values are irrelevant to the assertions —
/// the player id is overwritten by `InitGame` and the cookie only feeds the (unsent-here) join.
fn connect_body() -> Vec<u8> {
    vec![0u8; 12]
}

fn happy_script() -> Vec<RakEvent> {
    vec![
        RakEvent::Connected {
            body: connect_body(),
        },
        RakEvent::Rpc {
            id: RpcId::InitGame as u8,
            payload: InitGame {
                local_player_id: PlayerId(42),
                host_name: "Test Server".to_string(),
            }
            .encode(),
        },
        RakEvent::Rpc {
            id: RpcId::RequestClass as u8,
            payload: RequestClassResponse {
                allowed: true,
                spawn_position: Vector3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                ..RequestClassResponse::default()
            }
            .encode(),
        },
        RakEvent::Rpc {
            id: RpcId::RequestSpawn as u8,
            payload: RequestSpawnResponse { allow: 2 }.encode(),
        },
    ]
}

#[tokio::test]
async fn reaches_spawned_emitting_events_in_order() {
    let (transport, log) = ScriptedTransport::new(happy_script(), false);
    let mut engine = Engine::new(test_config(), transport);

    let mut milestones = Vec::new();
    while let Some(event) = engine.next_event().await {
        match event {
            ClientEvent::Connected => milestones.push("connected"),
            ClientEvent::Joined {
                local_id,
                host_name,
            } => {
                assert_eq!(local_id, PlayerId(42));
                assert_eq!(host_name, "Test Server");
                milestones.push("joined");
            }
            ClientEvent::Spawned => {
                milestones.push("spawned");
                break;
            }
            ClientEvent::Disconnected(reason) => panic!("unexpected disconnect: {reason}"),
            ClientEvent::ServerMessage { .. } | ClientEvent::Chat { .. } => {}
            ClientEvent::WalkArrived { .. } | ClientEvent::WalkFailed { .. } => {}
            ClientEvent::StateChanged(_) => {}
        }
    }

    assert_eq!(milestones, ["connected", "joined", "spawned"]);
    assert_eq!(engine.state(), &ConnectionState::Spawned);

    let log = log.lock().expect("log poisoned");
    // Server-driven spawn (RakSAMP Lite model): ClientJoin + RequestClass, then the server's
    // RequestSpawnResponse(allow==2) makes us send Spawn. We never send RequestSpawn ourselves here.
    assert_eq!(
        log.rpc_ids(),
        vec![
            RpcId::ClientJoin as u8,
            RpcId::RequestClass as u8,
            RpcId::Spawn as u8,
        ]
    );
}

#[tokio::test(start_paused = true)]
async fn disconnect_schedules_reconnect() {
    let (transport, log) = ScriptedTransport::new(
        vec![RakEvent::Disconnected(DisconnectReason::ConnectionLost)],
        false,
    );
    let mut engine = Engine::new(test_config(), transport);

    let mut disconnect_message = None;
    while let Some(event) = engine.next_event().await {
        if let ClientEvent::Disconnected(message) = event {
            disconnect_message = Some(message);
            break;
        }
    }

    assert_eq!(disconnect_message.as_deref(), Some("connection lost"));
    assert!(engine.reconnect_scheduled());
    assert_eq!(engine.state(), &ConnectionState::Disconnected);

    tokio::time::advance(test_config().reconnect_delay).await;
    let event = engine.next_event().await;
    assert!(matches!(
        event,
        Some(ClientEvent::StateChanged(ConnectionState::Connecting))
    ));
    assert_eq!(engine.state(), &ConnectionState::Connecting);
    assert!(!engine.reconnect_scheduled());
    assert_eq!(log.lock().expect("log poisoned").reconnects, 1);
}

#[tokio::test(start_paused = true)]
async fn banned_retries_on_a_fresh_proxy_exit() {
    // A ban is NOT terminal when we egress through a rotating proxy: SA-MP bans the PROXY EXIT IP, not the
    // account, so it is a false positive from a soured exit. Reconnecting re-runs the connect, which draws a
    // fresh `-session-<id>` (a new exit IP) and clears the ban — so the engine must SCHEDULE a reconnect
    // rather than close. Closing left the bot dead whenever the supervisor wasn't alive to relaunch it.
    let (transport, log) = ScriptedTransport::new(
        vec![RakEvent::Disconnected(DisconnectReason::Banned)],
        false,
    );
    let mut engine = Engine::new(test_config(), transport);

    let mut disconnect_message = None;
    while let Some(event) = engine.next_event().await {
        if let ClientEvent::Disconnected(message) = event {
            disconnect_message = Some(message);
            break;
        }
    }

    assert_eq!(disconnect_message.as_deref(), Some("banned"));
    assert!(
        engine.reconnect_scheduled(),
        "a ban is an exit-IP false positive — it must retry on a fresh proxy session"
    );

    tokio::time::advance(test_config().reconnect_delay).await;
    let event = engine.next_event().await;
    assert!(matches!(
        event,
        Some(ClientEvent::StateChanged(ConnectionState::Connecting))
    ));
    assert_eq!(engine.state(), &ConnectionState::Connecting);
    assert_eq!(log.lock().expect("log poisoned").reconnects, 1);
}

#[tokio::test(start_paused = true)]
async fn repeated_drops_stop_after_cap() {
    // Back-to-back non-terminal drops with no stable session between them (kick loop): the engine must
    // give up after the cap instead of reconnecting forever.
    let script = vec![RakEvent::Disconnected(DisconnectReason::ConnectionLost); 6];
    let (transport, _log) = ScriptedTransport::new(script, false);
    let mut engine = Engine::new(test_config(), transport);

    let mut disconnects = 0usize;
    while let Some(event) = engine.next_event().await {
        if matches!(event, ClientEvent::Disconnected(_)) {
            disconnects += 1;
        }
    }

    // MAX_RECONNECT_ATTEMPTS (5) reconnects are scheduled; the 6th drop trips the cap and closes.
    assert_eq!(disconnects, 6);
    assert!(!engine.reconnect_scheduled());
    assert_eq!(engine.state(), &ConnectionState::Disconnected);
}

#[tokio::test(start_paused = true)]
async fn connection_lost_without_event_still_reconnects() {
    let (transport, _log) = ScriptedTransport::new(Vec::new(), false);
    let mut engine = Engine::new(test_config(), transport);

    // Drain the initial `Connecting` state event, then the transport closes silently.
    let mut saw_disconnect = false;
    while let Some(event) = engine.next_event().await {
        if let ClientEvent::Disconnected(message) = event {
            assert_eq!(message, "connection lost");
            saw_disconnect = true;
            break;
        }
    }

    assert!(saw_disconnect);
    assert!(engine.reconnect_scheduled());
}

#[test]
fn track_state_updates_money_and_vehicle() {
    let (transport, _log) = ScriptedTransport::new(Vec::new(), false);
    let state =
        crate::state::LocalPlayer::shared("Bot".to_string(), "127.0.0.1:7777".parse().unwrap());
    let mut engine = Engine::new(test_config(), transport).with_bot_state(state.clone());

    // GivePlayerMoney (18) is additive; ResetPlayerMoney (20) zeroes.
    engine.track_state(GivePlayerMoney::RPC_ID, &500i32.to_le_bytes());
    engine.track_state(GivePlayerMoney::RPC_ID, &250i32.to_le_bytes());
    assert_eq!(state.borrow().money, 750);
    engine.track_state(ResetPlayerMoney::RPC_ID, &[]);
    assert_eq!(state.borrow().money, 0);

    // PutPlayerInVehicle (70): vehicleId u16 then seat u8.
    engine.track_state(PutPlayerInVehicle::RPC_ID, &[0x2A, 0x00, 0x00]);
    assert_eq!(state.borrow().vehicle_id(), 42);
    engine.track_state(RemovePlayerFromVehicle::RPC_ID, &[]);
    assert!(state.borrow().vehicle.is_none());
}

#[tokio::test]
async fn on_foot_sync_is_adaptive() {
    let (transport, log) = ScriptedTransport::new(Vec::new(), false);
    let state =
        crate::state::LocalPlayer::shared("Bot".to_string(), "127.0.0.1:7777".parse().unwrap());
    let mut engine = Engine::new(test_config(), transport).with_bot_state(state.clone());

    let sync_count = || log.lock().expect("log poisoned").sync_count();

    // First cycle always sends (nothing sent yet).
    engine.on_sync_tick(false).await;
    assert_eq!(sync_count(), 1);
    // Unchanged within the idle window → no resend.
    engine.on_sync_tick(false).await;
    assert_eq!(sync_count(), 1, "identical state should not resend");
    // A state change resends immediately.
    state.borrow_mut().on_foot.position.x = 5.0;
    engine.on_sync_tick(false).await;
    assert_eq!(sync_count(), 2, "a change should resend");
    // A forced sync always sends, even unchanged.
    engine.on_sync_tick(true).await;
    assert_eq!(sync_count(), 3, "force should always send");
}

#[tokio::test(start_paused = true)]
async fn sync_loop_sends_while_spawned() {
    let (transport, log) = ScriptedTransport::new(happy_script(), true);
    let mut engine = Engine::new(test_config(), transport);

    loop {
        match engine.next_event().await {
            Some(ClientEvent::Spawned) => break,
            Some(_) => continue,
            None => panic!("transport closed before spawn"),
        }
    }
    assert_eq!(engine.state(), &ConnectionState::Spawned);

    // Drive the FSM directly rather than from a background task — a registry-bearing engine is
    // `!Send` and cannot be `tokio::spawn`ed. With the clock paused, each `step()` awaits the
    // sync interval, which auto-advances and yields a `SyncTick`.
    let mut sync_count = 0;
    for _ in 0..16 {
        match engine.step().await {
            Step::SyncTick => engine.on_sync_tick(false).await,
            Step::Event(Some(event)) => engine.on_rak_event(event).await,
            Step::Update => engine.registry.tick(),
            _ => {}
        }
        sync_count = log.lock().expect("log poisoned").sync_count();
        if sync_count >= 1 {
            break;
        }
    }

    assert!(sync_count >= 1, "expected at least one on-foot sync packet");
    let packet = log
        .lock()
        .expect("log poisoned")
        .packets
        .iter()
        .find(|packet| packet.first() == Some(&(SyncPacketId::PlayerSync as u8)))
        .cloned()
        .expect("sync packet recorded");
    // The PlayerSync id byte followed by the real 68-byte on-foot sync body.
    assert_eq!(packet.first(), Some(&(SyncPacketId::PlayerSync as u8)));
    assert_eq!(packet.len(), samp_proto::ON_FOOT_SYNC_LEN + 1);
}

/// Byte-exact with `app`'s `objects` pcap-miner (`crates/app/src/bin/objects.rs`), which mines the same
/// RPC 44 body from real Arizona captures: `u16 objectId, i32 modelId, f32×3 position, ...(ignored)`.
#[test]
fn decode_create_object_prefix_reads_id_and_position() {
    let mut body = Vec::new();
    body.extend_from_slice(&777u16.to_le_bytes()); // object id
    body.extend_from_slice(&19300i32.to_le_bytes()); // model id (discarded)
    body.extend_from_slice(&(-503.545f32).to_le_bytes());
    body.extend_from_slice(&(-186.282f32).to_le_bytes());
    body.extend_from_slice(&77.645f32.to_le_bytes());
    body.extend_from_slice(&[0u8; 20]); // rotation + drawDistance + tail, unread

    let (id, pos) = decode_create_object_prefix(&body).expect("decodes");
    assert_eq!(id, 777);
    assert_eq!(
        pos,
        Vector3 {
            x: -503.545,
            y: -186.282,
            z: 77.645
        }
    );
}

#[test]
fn decode_create_object_prefix_rejects_short_payload() {
    assert!(decode_create_object_prefix(&[0u8; 10]).is_none());
}

fn vec3(x: f32, y: f32, z: f32) -> Vector3 {
    Vector3 { x, y, z }
}

#[test]
fn is_slap_detects_a_same_spot_z_jump() {
    let from = vec3(100.0, -50.0, 10.0);
    assert!(is_slap(from, vec3(100.0, -50.0, 25.0)));
    assert!(is_slap(from, vec3(100.02, -49.98, 3.0)));
}

#[test]
fn is_slap_ignores_a_real_teleport_or_small_terrain_step() {
    let from = vec3(100.0, -50.0, 10.0);
    // X/Y also moved: a genuine teleport, not a slap.
    assert!(!is_slap(from, vec3(500.0, -50.0, 25.0)));
    // Same spot but a sub-threshold Z change: ordinary terrain-following drift.
    assert!(!is_slap(from, vec3(100.0, -50.0, 10.5)));
}

/// `/slap` is `SetPlayerPos(z + 5)`: the engine applies that as a teleport and the ped then FREE-FALLS
/// back to the ground — and that fall is exactly what an anti-cheat probe raises a player to watch for.
/// Rising to the slapped height under gravity and hovering there instead is a flight signature, not a
/// slap reaction, so the height must be taken at once and only the fall simulated.
#[test]
fn an_upward_slap_teleports_at_once_then_falls_back_to_the_ground() {
    let mut engine = walking_driver();
    let ground = 5.0 + PED_GROUND_OFFSET;
    engine.sync_mut().position = vec3(10.0, 10.0, ground);

    engine.aim_follow(
        SetPlayerPos::RPC_ID,
        &samp_proto::events::encode_event(&SetPlayerPos {
            position: vec3(10.0, 10.0, ground + 5.0),
        }),
    );
    assert!(engine.slap_active(), "a raise should leave a fall behind");
    assert_eq!(
        engine.sync_mut().position.z,
        ground + 5.0,
        "the raise itself is a teleport and must land in the very next report"
    );

    // Step until the fall completes (bounded so a bug can't hang the test). Each iteration backdates
    // `last_step_at` to simulate a fixed-cadence tick without a real sleep.
    let mut heights = Vec::new();
    for _ in 0..500 {
        if let Some(slap) = engine.slap_mut() {
            slap.last_step_at = Instant::now() - Duration::from_millis(50);
        }
        engine.step_slap_reaction(Instant::now());
        heights.push(engine.sync_mut().position.z);
        assert!(
            engine.sync_mut().move_speed.z <= 0.0,
            "reported a climb of {} while falling",
            engine.sync_mut().move_speed.z
        );
        if !engine.slap_active() {
            break;
        }
    }
    assert!(!engine.slap_active(), "the fall should finish");
    assert!(
        heights.len() > 2,
        "landed in {} ticks, not a fall",
        heights.len()
    );
    assert!(
        heights.windows(2).all(|w| w[1] <= w[0]),
        "height must fall monotonically: {heights:?}"
    );
    assert!(
        (engine.sync_mut().position.z - ground).abs() < 1e-3,
        "landed at {}, expected the navmesh floor {ground}",
        engine.sync_mut().position.z
    );
    assert_eq!(
        engine.sync_mut().move_speed,
        Vector3::default(),
        "velocity is reported as zero once landed"
    );
}

/// A reposition that puts the bot at or below the ground has nothing to fall through: the engine just
/// applies it, so simulating a descent would report a fall the server never asked for.
#[test]
fn a_downward_slap_is_applied_at_once_with_no_fall() {
    let (transport, _log) = ScriptedTransport::new(Vec::new(), false);
    let mut engine = Engine::new(test_config(), transport);
    engine.set_state(ConnectionState::Spawned);
    engine.sync_mut().position = vec3(10.0, 10.0, 50.0);

    engine.aim_follow(
        SetPlayerPos::RPC_ID,
        &samp_proto::events::encode_event(&SetPlayerPos {
            position: vec3(10.0, 10.0, 20.0),
        }),
    );

    assert!(!engine.slap_active(), "nothing to fall through");
    assert_eq!(engine.sync_mut().position.z, 20.0);
}

fn flat_navmesh_at_z5() -> sa_nav::NavMesh {
    sa_nav::NavMesh {
        verts: vec![
            [0.0, 0.0, 5.0],
            [20.0, 0.0, 5.0],
            [20.0, 20.0, 5.0],
            [0.0, 20.0, 5.0],
        ],
        polys: vec![sa_nav::NavPoly {
            verts: vec![0, 1, 2, 3],
            neighbors: vec![-1, -1, -1, -1],
            area: 63,
        }],
        detail_verts: vec![
            [0.0, 0.0, 5.0],
            [20.0, 0.0, 5.0],
            [20.0, 20.0, 5.0],
            [0.0, 20.0, 5.0],
        ],
        detail_tris: vec![[0, 1, 2], [0, 2, 3]],
        detail_meshes: vec![[0, 2]],
    }
}

fn walking_driver() -> Engine<ScriptedTransport> {
    let (transport, _log) = ScriptedTransport::new(vec![], true);
    let mut engine = Engine::new(test_config(), transport);
    engine.set_state(ConnectionState::Spawned);
    engine.set_nav(sa_nav::NavQuery::new(flat_navmesh_at_z5()));
    engine.sync_mut().position = Vector3 {
        x: 2.0,
        y: 2.0,
        z: 5.0 + PED_GROUND_OFFSET,
    };
    engine
}

/// A ped's origin rests exactly `collisionFloor + 1.0` (GTA `CPed::ProcessEntityCollision`), and our
/// Recast floor sits ~0.33 above the true collision surface, so the body must ride at
/// `navFloor + PED_GROUND_OFFSET`. Reporting the raw floor sinks the ped ~0.67 into the ground, which
/// the remote corrects every frame and hard-snaps once the error passes 1.0 m.
#[tokio::test]
async fn the_body_rides_a_ped_height_above_the_nav_floor() {
    let mut engine = walking_driver();
    engine.start_walk(vec3(18.0, 18.0, 5.0), 1);
    tick_sync(&mut engine, 5).await;

    let z = engine.motion().sample().position.z;
    assert!(
        (z - (5.0 + PED_GROUND_OFFSET)).abs() < 1e-3,
        "body rode at {z}, expected the z=5 floor plus the ped offset"
    );
}
/// The server owns our health (Arizona's hunger system lowers it continuously). Reporting the spawn
/// value forever claims more health than the server assigned — the health-hack pattern — so
/// SetPlayerHealth/SetPlayerArmour must reach the outbound sync.
#[test]
fn server_health_and_armour_reach_the_sync() {
    let (transport, _log) = ScriptedTransport::new(Vec::new(), false);
    let state =
        crate::state::LocalPlayer::shared("Bot".to_string(), "127.0.0.1:7777".parse().unwrap());
    let mut engine = Engine::new(test_config(), transport).with_bot_state(state.clone());
    engine.sync_mut().health = SPAWN_HEALTH;

    engine.track_state(SetPlayerHealth::RPC_ID, &70.0f32.to_le_bytes());
    engine.track_state(SetPlayerArmour::RPC_ID, &35.0f32.to_le_bytes());
    engine.mirror_from_state();

    assert_eq!(engine.sync_mut().health, 70);
    assert_eq!(engine.sync_mut().armour, 35);
}

/// The gait owns only the movement modifiers. A key a script is holding (aim, fire, secondary) must
/// survive the walker's tick, or aiming and firing while moving cannot be expressed at all.
#[test]
fn walking_preserves_script_held_keys() {
    const KEY_AIM: u16 = 0x0080;
    let mut engine = walking_driver();
    engine.start_walk(
        Vector3 {
            x: 18.0,
            y: 18.0,
            z: 5.0,
        },
        2,
    );
    engine.sync_mut().keys = KEY_AIM;
    engine.motion().advance(Duration::from_millis(100));
    engine.step_walk(Instant::now());

    assert_eq!(
        engine.sync_mut().keys & KEY_AIM,
        KEY_AIM,
        "script key survived"
    );
    assert_eq!(
        engine.sync_mut().keys & KEY_SPRINT,
        KEY_SPRINT,
        "sprint gait key still applied"
    );
}

/// Sprint stop travel varies with the speed band, so the model can bottom out metres short of the
/// target. Finishing the walk from there would snap the reported position past the remote's 2 m XY
/// threshold — a teleport. The walker must keep moving instead.
#[test]
fn a_stalled_stop_short_of_the_target_keeps_walking() {
    let mut engine = walking_driver();
    let target = Vector3 {
        x: 18.0,
        y: 18.0,
        z: 5.0,
    };
    engine.start_walk(target, 2);
    engine.sync_mut().position = Vector3 {
        x: 16.0,
        y: 16.0,
        z: 5.0 + PED_GROUND_OFFSET,
    };
    engine.locomotion_mut().set_target(None);
    for _ in 0..40 {
        engine.locomotion_mut().tick(TickInput {
            dt: 0.1,
            heading_delta: 0.0,
        });
    }
    assert!(
        engine.locomotion_mut().is_stopped(),
        "model reached a full stop"
    );

    engine.motion().advance(Duration::from_millis(100));
    engine.step_walk(Instant::now());

    assert!(engine.walking(), "walk continues instead of arriving");
    assert!(
        engine
            .pending
            .iter()
            .all(|e| !matches!(e, ClientEvent::WalkArrived { .. })),
        "no arrival was reported from 2.8 m out"
    );
}

/// find_path clamps a target inside a carved keep-out to the hole edge, so the last waypoint and the
/// caller's target can be metres apart by design. Snapping the reported position to the target on
/// The idle glance must never outrank a script's facing. Anchoring once per idle stretch made every
/// later `setBotFacing` a permanent no-op on the wire — the bot kept reporting the yaw it stopped at,
/// so a script turning to face an NPC to open its dialog never actually turned.
#[test]
fn idle_wobble_follows_script_facing() {
    let (transport, _log) = ScriptedTransport::new(Vec::new(), false);
    let state =
        crate::state::LocalPlayer::shared("Bot".to_string(), "127.0.0.1:7777".parse().unwrap());
    let mut engine = Engine::new(test_config(), transport).with_bot_state(state.clone());

    engine.apply_idle_wobble(Instant::now());
    let yaw_of = |q: Quaternion| 2.0 * q.z.atan2(q.w);

    // A script turns the bot a quarter-turn while it stands still.
    let target = std::f32::consts::FRAC_PI_2;
    state.borrow_mut().on_foot.quaternion = Quaternion {
        x: 0.0,
        y: 0.0,
        z: (target * 0.5).sin(),
        w: (target * 0.5).cos(),
    };
    engine.mirror_from_state();
    engine.apply_idle_wobble(Instant::now());

    let reported = yaw_of(engine.sync_mut().quaternion);
    assert!(
        (reported - target).abs() <= IDLE_WOBBLE_MAX + 1e-3,
        "facing must track the script within the glance offset: wrote {target}, wire says {reported}"
    );
}

/// Runs a whole sprint leg to arrival. The brake radius exceeds the model's stop travel, so the walker
/// reaches a standstill short of the target and has to close the gap — the tick-rate walk/idle strobe
/// that used to produce (animation flipping 1231/1189/0xFFFF ten times a second) is a louder
/// fingerprint than the constant speed the model replaced.
#[test]
fn a_sprint_leg_closes_without_strobing() {
    let mut engine = walking_driver();
    engine.start_walk(
        Vector3 {
            x: 18.0,
            y: 18.0,
            z: 5.0,
        },
        2,
    );

    let mut anim_changes = 0usize;
    let mut last_anim = (
        engine.sync_mut().animation_id,
        engine.sync_mut().animation_flags,
    );
    for _ in 0..400 {
        let Some(walk) = engine.walk_mut() else {
            break;
        };
        // step_walk derives dt from the wall clock; back-date it so each iteration is one 100 ms tick.
        walk.last_step = Some(Instant::now() - Duration::from_millis(100));
        engine.motion().advance(Duration::from_millis(100));
        engine.step_walk(Instant::now());
        let anim = (
            engine.sync_mut().animation_id,
            engine.sync_mut().animation_flags,
        );
        if anim != last_anim {
            anim_changes += 1;
            last_anim = anim;
        }
    }

    assert!(!engine.walking(), "the leg finished");
    assert!(
        anim_changes < 20,
        "animation changed {anim_changes} times over one leg — that is a strobe, not locomotion"
    );
}

/// The brake radius has to be measured along the PATH. A corridor that detours — around a carved
/// keep-out, around an obstacle — passes close to the target with most of the walk still ahead, and
/// braking on that straight-line distance stops the gait dead: the model ramps to zero, the step
/// becomes zero, the waypoint is never reached, so the path is never exhausted and no arrival is ever
/// reported. The walker hangs with `isWalking()` stuck true and the script waits forever.
#[test]
fn braking_uses_the_remaining_path_not_the_straight_line() {
    let mut engine = walking_driver();
    let target = vec3(18.0, 18.0, 5.0);
    engine.start_walk(target, 2);
    let z = 5.0 + PED_GROUND_OFFSET;
    // A detour: the corridor runs out to (2,18) and only then back to the target, while the bot
    // happens to stand half a metre from that target in a straight line.
    let floor = z - PED_GROUND_OFFSET;
    engine.motion().place(vec3(18.0, 17.5, z));
    engine
        .motion()
        .start(vec![[2.0, 18.0, floor], [18.0, 18.0, floor]], target, None);
    engine.sync_mut().position = vec3(18.0, 17.5, z);

    let mut reached_the_far_leg = false;
    for _ in 0..300 {
        if let Some(w) = engine.walk_mut() {
            w.last_step = Some(Instant::now() - Duration::from_millis(100));
        }
        engine.motion().advance(Duration::from_millis(100));
        engine.step_walk(Instant::now());
        if engine.sync_mut().position.x < 4.0 {
            reached_the_far_leg = true;
        }
        if !engine.walking() {
            break;
        }
    }

    assert!(!engine.walking(), "walker never arrived — it stalled");
    assert!(
        reached_the_far_leg,
        "walked to {:?} without ever taking the detour",
        engine.sync_mut().position
    );
    assert!(matches!(
        engine.pending().back(),
        Some(ClientEvent::WalkArrived { .. })
    ));
}

/// Re-issuing `walkTo` mid-walk (retargeting to the next tree) must not restart the facing: the
/// fresh-walk branch snaps the heading straight to the new bearing, which for a target behind the bot
/// is a 180 deg turn and a full velocity reversal inside one 100 ms packet — an angular rate no ped
/// can produce and the reported speed says we are still sprinting through it.
#[test]
fn retargeting_mid_walk_slews_the_heading_instead_of_flipping_it() {
    let mut engine = walking_driver();
    engine.start_walk(vec3(18.0, 2.0, 5.0), 2);
    for _ in 0..10 {
        if let Some(w) = engine.walk_mut() {
            w.last_step = Some(Instant::now() - Duration::from_millis(100));
        }
        engine.motion().advance(Duration::from_millis(100));
        engine.step_walk(Instant::now());
    }
    let before = engine.motion().sample().heading;

    engine.start_walk(vec3(2.0, 2.0, 5.0), 2);
    if let Some(w) = engine.walk_mut() {
        w.last_step = Some(Instant::now() - Duration::from_millis(100));
    }
    engine.motion().advance(Duration::from_millis(100));
    engine.step_walk(Instant::now());
    let after = engine.motion().sample().heading;

    let step = (after - before).abs();
    assert!(
        step <= TURN_RATE * 0.1 + 1e-3,
        "turned {:.1} deg in one 100 ms tick, above the {:.1} deg the turn rate allows",
        step.to_degrees(),
        (TURN_RATE * 0.1).to_degrees()
    );
}

/// A real ped's Z is whatever the collision probe puts it at, so over flat ground consecutive reports
/// are bit-identical — measured at 96.755% of 1,470,848 stationary sample pairs in the capture corpus.
/// Per-tick noise on the reported Z has no counterpart in that data, and because the terrain-tracking
/// term reverts toward the waypoint in proportion to horizontal progress, it does not decay at all
/// while the bot is braked to a standstill: it becomes a free random walk that eventually crosses the
/// remote's 1 m Z snap on its own.
#[test]
fn walking_over_flat_ground_reports_a_bit_exact_z() {
    let mut engine = walking_driver();
    engine.start_walk(vec3(18.0, 18.0, 5.0), 1);
    let start = engine.sync_mut().position;

    let mut heights = Vec::new();
    for _ in 0..30 {
        if let Some(w) = engine.walk_mut() {
            w.last_step = Some(Instant::now() - Duration::from_millis(100));
        }
        engine.motion().advance(Duration::from_millis(100));
        engine.step_walk(Instant::now());
        if !engine.walking() {
            break;
        }
        heights.push(engine.sync_mut().position.z);
    }

    assert!(
        (engine.sync_mut().position.x - start.x).abs() > 1.0,
        "the bot never actually walked"
    );
    assert!(
        heights.iter().all(|&z| z == start.z),
        "flat ground produced a varying Z: {heights:?}"
    );
}

fn flat_navmesh_at_z5_sized(extent: f32) -> sa_nav::NavMesh {
    sa_nav::NavMesh {
        verts: vec![
            [0.0, 0.0, 5.0],
            [extent, 0.0, 5.0],
            [extent, extent, 5.0],
            [0.0, extent, 5.0],
        ],
        polys: vec![sa_nav::NavPoly {
            verts: vec![0, 1, 2, 3],
            neighbors: vec![-1, -1, -1, -1],
            area: 63,
        }],
        detail_verts: vec![
            [0.0, 0.0, 5.0],
            [extent, 0.0, 5.0],
            [extent, extent, 5.0],
            [0.0, extent, 5.0],
        ],
        detail_tris: vec![[0, 1, 2], [0, 2, 3]],
        detail_meshes: vec![[0, 2]],
    }
}

/// A spawned engine on a large flat mesh, for legs long enough to measure a distribution over sent
/// packets rather than over a handful of them.
fn long_walk_driver() -> Engine<ScriptedTransport> {
    let (transport, _log) = ScriptedTransport::new(vec![], true);
    let mut engine = Engine::new(test_config(), transport);
    engine.set_state(ConnectionState::Spawned);
    engine.set_nav(sa_nav::NavQuery::new(flat_navmesh_at_z5_sized(600.0)));
    engine.sync_mut().position = Vector3 {
        x: 10.0,
        y: 10.0,
        z: 5.0 + PED_GROUND_OFFSET,
    };
    engine
}

/// Run `ticks` sync cycles at a simulated 100 ms cadence, back-dating the walker clock so the wall
/// clock of a tight test loop doesn't collapse every step to dt close to zero. Stops once the walk ends.
async fn tick_sync(engine: &mut Engine<ScriptedTransport>, ticks: usize) {
    for _ in 0..ticks {
        // Stand in for the background stepper, then for the send tick it feeds.
        engine.motion().advance(Duration::from_millis(100));
        // Back-date only an ALREADY-primed clock. Seeding it on the first tick too would hide the
        // real production shape — a walk's first frame genuinely has no previous step to measure
        // against, which is exactly where a finite-difference velocity goes wrong.
        if let Some(walk) = engine.walk_mut() {
            if walk.last_step.is_some() {
                walk.last_step = Some(Instant::now() - Duration::from_millis(100));
            }
        }
        engine.on_sync_tick(false).await;
        if !engine.walking() {
            break;
        }
    }
}

/// The ALT interaction is a phase-encoded key pulse: the script holds `INTERACT_KEYS` for a few passes
/// then drops it, forcing a sync each pass, and the server only raises `OnPlayerKeyStateChange` if the
/// bit survives several consecutive on-foot frames (the sync is UnreliableSequenced over a lossy
/// proxy). If a key write lands a tick late, or the sampler reads between the on and the off write,
/// the pulse is never sent and chainsaw rent, chop and cart delivery all stop, silently.
#[tokio::test]
async fn a_pulsed_key_survives_into_consecutive_sent_frames() {
    const INTERACT_KEYS: u16 = 0x0400;
    let (transport, log) = ScriptedTransport::new(Vec::new(), false);
    let state =
        crate::state::LocalPlayer::shared("Bot".to_string(), "127.0.0.1:7777".parse().unwrap());
    let mut engine = Engine::new(test_config(), transport).with_bot_state(state.clone());
    engine.set_state(ConnectionState::Spawned);

    for pass in 0..10 {
        state.borrow_mut().on_foot.keys = if pass < 3 { INTERACT_KEYS } else { 0 };
        engine.on_sync_tick(true).await;
    }

    let log = log.lock().expect("log poisoned");
    let held: Vec<bool> = log
        .on_foot()
        .map(|packet| on_foot::keys(packet) & INTERACT_KEYS != 0)
        .collect();
    assert_eq!(held.len(), 10, "every forced sync must send a frame");
    let longest = held
        .split(|held| !held)
        .map(<[bool]>::len)
        .max()
        .unwrap_or(0);
    assert!(
        longest >= 3,
        "ALT bit survived only {longest} consecutive frames, needs 3: {held:?}"
    );
}

/// The reported velocity must stay the true derivative of the reported position over the interval the
/// remote dead-reckons — that is what separates a walk from a teleport. Two ways to break it: integrate
/// a script-written `move_speed` as if it were a physics input (double counting), or sample position
/// and velocity from different moments. Both show up here as the reported speed disagreeing with the
/// distance actually travelled between two SENT packets.
#[tokio::test]
async fn reported_speed_matches_the_distance_actually_travelled() {
    let straight = Vector3 {
        x: 400.0,
        y: 10.0,
        z: 5.0,
    };
    let hairpin = Vector3 {
        x: 12.0,
        y: 300.0,
        z: 5.0,
    };
    for (label, target) in [("straight", straight), ("hairpin", hairpin)] {
        let mut engine = long_walk_driver();
        let log = engine.transport.log.clone();
        engine.start_walk(target, 2);
        tick_sync(&mut engine, 400).await;

        let log = log.lock().expect("log poisoned");
        let frames: Vec<([f32; 3], [f32; 3])> = log
            .on_foot()
            .map(|packet| (on_foot::position(packet), on_foot::move_speed(packet)))
            .collect();
        assert!(frames.len() > 20, "{label}: only {} frames", frames.len());

        let mut discontinuities = Vec::new();
        for (i, pair) in frames.windows(2).enumerate() {
            let (from, _) = pair[0];
            let (to, speed) = pair[1];
            let travelled = ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2)).sqrt();
            // The remote snaps past 2.0 m of XY error; one 100 ms frame must never claim that much.
            assert!(
                travelled < 2.0,
                "{label} frame {i}: jumped {travelled:.3} u, past the remote's snap threshold"
            );
            let claimed = (speed[0] * speed[0] + speed[1] * speed[1]).sqrt() * 50.0 * 0.1;
            if (travelled - claimed).abs() > 0.1 * claimed.max(0.02) + 1e-3 {
                discontinuities.push((i, travelled, claimed));
            }
        }

        // Every frame, arrival included. The walker decelerates into the arrival radius and stands
        // where it stopped, so there is no frame left in which the reported position moves by something
        // other than the reported velocity.
        assert!(
            discontinuities.is_empty(),
            "{label}: {} frames where reported speed did not match travel: {discontinuities:?}",
            discontinuities.len()
        );
    }
}

/// Animation and the forward analog are SEND-cadence machines, not physics: `anim_hold` (20..60) and
/// `pedal_hold` (2..3) count model ticks, and the model was calibrated against a 10 Hz packet capture.
/// Advancing them per physics step instead of per sent packet would shorten every hold by the rate
/// ratio — a pure wire fingerprint with no functional symptom, so nothing would report it but a ban.
/// Asserting over SENT packets is the whole point; asserting per tick is what would let it through.
#[tokio::test]
async fn animation_and_analog_hold_across_sent_packets() {
    let mut engine = long_walk_driver();
    let log = engine.transport.log.clone();
    engine.start_walk(
        Vector3 {
            x: 560.0,
            y: 560.0,
            z: 5.0,
        },
        2,
    );
    tick_sync(&mut engine, 900).await;

    let log = log.lock().expect("log poisoned");
    let frames: Vec<((u16, u16), i16)> = log
        .on_foot()
        .map(|packet| (on_foot::animation(packet), on_foot::up_down(packet)))
        .collect();
    assert!(
        frames.len() >= 200,
        "need a long leg to measure a distribution, got {} frames",
        frames.len()
    );

    let anim_changes = frames.windows(2).filter(|w| w[0].0 != w[1].0).count();
    let mean_hold = frames.len() as f64 / anim_changes.max(1) as f64;
    assert!(
        mean_hold >= 15.0,
        "animation changed every {mean_hold:.1} sent packets; the model holds 20-60 model ticks"
    );

    // up_down rails at -128 while pushing forward, with a short -127..-120 tail; a rate mismatch shows
    // up as the tail taking over.
    let tail = frames.iter().filter(|(_, ud)| *ud != -128).count();
    let tail_frac = tail as f64 / frames.len() as f64;
    assert!(
        tail_frac <= 0.25,
        "forward analog off its rail {tail_frac:.3} of frames"
    );
}

/// A script's queued RPCs must reach the wire BEFORE the on-foot frame that a following `updateSync()`
/// forces. The sawmill's ALT interaction depends on it: RPC 168 names the actor being targeted, and
/// only if the server sees it first does the ALT bit in the next 207 tie to that NPC and open the
/// dialog. Today the ordering is an accident of two adjacent statements in `next_event` with no test.
#[tokio::test]
async fn script_rpcs_reach_the_wire_before_the_forced_sync() {
    const RPC_TARGET_UPDATE: u8 = 168;
    let (transport, log) = ScriptedTransport::new(Vec::new(), false);
    let state =
        crate::state::LocalPlayer::shared("Bot".to_string(), "127.0.0.1:7777".parse().unwrap());
    let registry = PacketRegistry::new();
    let outbox = registry.outbox();
    let mut engine = Engine::new(test_config(), transport)
        .with_bot_state(state.clone())
        .with_registry(registry);
    engine.set_state(ConnectionState::Spawned);

    // What a script does: queue the target RPC, set the keys, then updateSync().
    outbox.borrow_mut().push_back(OutboundMsg::Rpc {
        id: RPC_TARGET_UPDATE,
        payload: vec![0xFF, 0xFF],
    });
    state.borrow_mut().on_foot.keys = 0x0400;
    state.borrow_mut().force_sync = true;

    // The same order `next_event` runs them in.
    engine.flush_outbox().await;
    engine.poll_bot_actions().await;

    let log = log.lock().expect("log poisoned");
    let rpc_at = log
        .sequence
        .iter()
        .position(|w| *w == Wire::Rpc(RPC_TARGET_UPDATE))
        .expect("the queued RPC never reached the wire");
    let sync_at = log
        .sequence
        .iter()
        .position(|w| *w == Wire::Packet(SyncPacketId::PlayerSync as u8))
        .expect("the forced sync never reached the wire");
    assert!(
        rpc_at < sync_at,
        "RPC 168 must precede the on-foot frame carrying the ALT bit: {:?}",
        log.sequence
    );
}

/// A `walkTo` that cannot path still supersedes the walk it was issued during. Leaving the old walk
/// running would drive the bot to a target the script has already abandoned while `isWalking()` reports
/// true — which the script reads as ITS walk running, so it waits out the full timeout and its
/// `fly.to` fallback never gets the stopped walker it needs.
#[test]
fn a_failed_retarget_stops_the_walk_it_superseded() {
    let mut engine = walking_driver();
    engine.start_walk(vec3(18.0, 18.0, 5.0), 1);
    assert!(engine.walking(), "the first walk should be running");

    // Off the mesh entirely: no corridor exists.
    engine.start_walk(vec3(5000.0, 5000.0, 5.0), 1);

    assert!(
        !engine.walking(),
        "a failed retarget must not leave the superseded walk driving"
    );
    assert!(matches!(
        engine.pending().back(),
        Some(ClientEvent::WalkFailed { .. })
    ));
}

/// Indoors, a same-X/Y jump of several metres is a floor change, not a slap — geometrically identical,
/// but answering it with a free fall drops the bot straight back out of the room the server just put it
/// in. Only the outdoor world gets the fall.
#[test]
fn an_indoor_floor_change_is_not_a_slap() {
    let (transport, _log) = ScriptedTransport::new(Vec::new(), false);
    let state =
        crate::state::LocalPlayer::shared("Bot".to_string(), "127.0.0.1:7777".parse().unwrap());
    let mut engine = Engine::new(test_config(), transport).with_bot_state(state.clone());
    engine.set_state(ConnectionState::Spawned);
    engine.sync_mut().position = vec3(10.0, 10.0, 20.0);
    state.borrow_mut().interior = 5;

    engine.aim_follow(
        SetPlayerPos::RPC_ID,
        &samp_proto::events::encode_event(&SetPlayerPos {
            position: vec3(10.0, 10.0, 26.0),
        }),
    );

    assert!(
        !engine.slap_active(),
        "an interior floor change must not start a fall"
    );
    assert_eq!(
        engine.sync_mut().position.z,
        26.0,
        "the floor change is applied at once"
    );
}

/// Player ids are reassigned per session, so a peer table carried across a reconnect names players who
/// are not there — and the walker routes around every one of them.
#[tokio::test]
async fn peers_are_forgotten_on_disconnect() {
    let (transport, _log) = ScriptedTransport::new(Vec::new(), false);
    let mut engine = Engine::new(test_config(), transport);
    let now = Instant::now();
    engine.world.players.update(
        &samp_proto::PlayerSyncData {
            player_id: 7,
            position: vec3(10.0, 10.0, 3.0),
            ..Default::default()
        },
        now,
    );
    assert_eq!(engine.world.players.iter_fresh(now).count(), 1);

    engine.on_disconnect(DisconnectReason::ConnectionLost);

    assert_eq!(
        engine.world.players.iter_fresh(now).count(),
        0,
        "the peer table must not survive the session that populated it"
    );
}

/// A finished walk stands exactly where it walked to, at ped height, and says so. It does NOT snap onto
/// the caller's coordinates: the walk decelerates into the arrival radius, and closing the remainder in
/// one velocity-free jump is the discontinuity this walker exists to remove. The caller's Z is never
/// adopted either — a script aiming at a world object passes that object's origin, which is not a
/// standing height.
#[tokio::test]
async fn a_finished_walk_stands_where_it_walked_to() {
    let mut engine = walking_driver();
    let target = Vector3 {
        x: 18.0,
        y: 18.0,
        z: 6.5,
    };
    engine.start_walk(target, 1);
    tick_sync(&mut engine, 200).await;

    assert!(!engine.walking(), "the walk should have finished");
    let stood = engine.sync_mut().position;
    let gap = ((target.x - stood.x).powi(2) + (target.y - stood.y).powi(2)).sqrt();
    assert!(
        gap <= WALK_ARRIVE,
        "stopped {gap:.3} u from the target, outside the arrival radius"
    );
    assert!(
        (stood.z - (5.0 + PED_GROUND_OFFSET)).abs() < 1e-3,
        "stood at {stood:?}, expected the grounded walk Z, not the caller's 6.5"
    );

    let arrived = engine
        .pending()
        .iter()
        .find_map(|e| match e {
            ClientEvent::WalkArrived { x, y, z } => Some((*x, *y, *z)),
            _ => None,
        })
        .expect("WalkArrived was emitted");
    assert_eq!(
        arrived,
        (stood.x, stood.y, stood.z),
        "the event must report where the bot actually stands"
    );
}

/// A direct seek is the Rust-side replacement for the script mover: it reaches a point the navmesh
/// bake never covered, but by WALKING there — same body, same gait, same measured velocity — instead of
/// writing positions from Luau. It must reach an off-mesh target that `walkTo` cannot path to, and the
/// frames it emits must obey the same speed/travel invariant as any other walk.
#[tokio::test]
async fn a_direct_seek_walks_to_a_target_the_navmesh_cannot_path_to() {
    let mut engine = long_walk_driver();
    let log = engine.transport.log.clone();
    // Off the mesh entirely: a navmesh walk has no corridor to it.
    let off_mesh = vec3(-30.0, 10.0, 5.0);
    engine.start_walk(off_mesh, 1);
    assert!(!engine.walking(), "a navmesh walk cannot reach off-mesh");

    // The same target, sought directly.
    engine.start_walk(off_mesh, 11);
    assert!(engine.walking(), "a direct seek should start");
    tick_sync(&mut engine, 400).await;

    let stood = engine.sync_mut().position;
    let gap = ((off_mesh.x - stood.x).powi(2) + (off_mesh.y - stood.y).powi(2)).sqrt();
    assert!(
        gap <= WALK_ARRIVE,
        "seek stopped {gap:.2} u short of the target"
    );

    let log = log.lock().expect("log poisoned");
    let frames: Vec<([f32; 3], [f32; 3])> = log
        .on_foot()
        .map(|packet| (on_foot::position(packet), on_foot::move_speed(packet)))
        .collect();
    assert!(frames.len() > 20, "only {} frames", frames.len());
    for (i, pair) in frames.windows(2).enumerate() {
        let (from, _) = pair[0];
        let (to, speed) = pair[1];
        let travelled = ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2)).sqrt();
        assert!(travelled < 2.0, "frame {i}: jumped {travelled:.3} u");
        let claimed = (speed[0] * speed[0] + speed[1] * speed[1]).sqrt() * 50.0 * 0.1;
        assert!(
            (travelled - claimed).abs() <= 0.1 * claimed.max(0.02) + 1e-3,
            "frame {i}: travelled {travelled:.4} u but reported {claimed:.4} u"
        );
    }
}

/// Every other walker test runs ONE leg on a fresh engine, so the body's retained state across walks
/// was never observed — which is how a stale commanded speed plus a zero-length first frame put a
/// six-orders-of-magnitude velocity on the wire for the whole of every walk after the first.
/// Two legs, and a hard ceiling on what any frame may claim.
#[tokio::test]
async fn a_second_walk_never_reports_an_impossible_speed() {
    let mut engine = long_walk_driver();
    let log = engine.transport.log.clone();

    engine.start_walk(vec3(80.0, 10.0, 5.0), 2);
    tick_sync(&mut engine, 300).await;
    assert!(!engine.walking(), "first leg should finish");

    // No walkStop() in between: this is the normal completion path the scripts take.
    engine.start_walk(vec3(80.0, 90.0, 5.0), 2);
    tick_sync(&mut engine, 300).await;

    // And the harder one: a RETARGET at cruise. The gait never brakes, so the body is still commanded
    // at full speed when the new walk resets the sender's clock — the case where a finite-difference
    // velocity over a zero-length interval goes to six figures.
    engine.start_walk(vec3(500.0, 500.0, 5.0), 2);
    for _ in 0..40 {
        engine.motion().advance(Duration::from_millis(100));
        if let Some(walk) = engine.walk_mut() {
            if walk.last_step.is_some() {
                walk.last_step = Some(Instant::now() - Duration::from_millis(100));
            }
        }
        engine.on_sync_tick(false).await;
    }
    engine.start_walk(vec3(20.0, 500.0, 5.0), 2);
    tick_sync(&mut engine, 40).await;

    let log = log.lock().expect("log poisoned");
    // Sprint cruise is 0.1912 wire units per frame; allow generous headroom and still catch anything
    // that is not a plausible gait.
    let ceiling = 0.1912 * 2.0;
    for (i, packet) in log.on_foot().enumerate() {
        let v = on_foot::move_speed(packet);
        let mag = (v[0] * v[0] + v[1] * v[1]).sqrt();
        assert!(
            mag <= ceiling,
            "frame {i}: reported |move_speed| {mag} (= {} u/s), above any real gait",
            mag * 50.0
        );
    }
}
