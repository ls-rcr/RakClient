//! The connection state machine: [`Engine`] is generic over a [`Transport`] and pumped one event at a
//! time through [`Engine::next_event`]; see docs/memory/samp-client/engine.md#module.

use std::collections::VecDeque;
use std::future;
use std::pin::Pin;
use std::time::{Duration, Instant};

use crate::aim::AimSync;
use crate::client_emulation::ClientEmulation;
use crate::locomotion::{GaitKind, LocomotionModel, TickInput};
use crate::registry::PacketRegistry;
use crate::state::SharedLocalPlayer;
use crate::transport::Transport;
use crate::world::motion::{MotionHandle, PHYSICS_DT, WALK_ARRIVE};
use crate::world::{EntityView, World, PED_GROUND_OFFSET};
use crate::{ClientConfig, ClientEvent, ConnectionState};
use events::WireDecode;
use raknet::{DisconnectReason, RakEvent, Reliability};
use samp_proto::events::decode_event;
use samp_proto::events::incoming::{
    DestroyObject, GivePlayerMoney, InterpolateCamera, PlayerStreamIn, PlayerStreamOut,
    PutPlayerInVehicle, RemovePlayerFromVehicle, ResetPlayerMoney, SetCameraPosition, SetInterior,
    SetObjectPosition, SetPlayerArmour, SetPlayerHealth, SetPlayerPos, SetPlayerPosFindZ,
    SetSpawnInfo, TogglePlayerSpectating,
};
use samp_proto::{
    decode_cp1251, events, ArizonaSync221, ChatMessage, ChatOutgoing, ClientJoin, Decode,
    Direction, Encode, InitGame, OnFootSync, OutboundMsg, PlayerId, Quaternion, RequestClass,
    RequestClassResponse, RequestSpawn, RequestSpawnResponse, RpcId, ServerMessage, ShowDialog,
    Spawn, SpectatorSync, StatsUpdate, SyncPacketId, Vector3, VehicleSyncData, Verdict, WeaponId,
    CHALLENGE_XOR, SAMP_VERSION_0_3_7,
};
use tokio::time::{self, Interval, MissedTickBehavior, Sleep};

mod inbound;
mod outbound;
mod session;
mod walker;

/// How often the registry's `on_update` handlers fire while connected (drives script timers).
const UPDATE_INTERVAL: Duration = Duration::from_millis(100);

/// Keepalive resend floor for an unchanged on-foot state (≥2 Hz); see
/// docs/memory/samp-client/engine.md#idle-sync-interval.
const IDLE_SYNC_INTERVAL: Duration = Duration::from_millis(500);

/// Stats-report cadence (`PACKET_STATS_UPDATE` = 205) while spawned; see
/// docs/memory/samp-client/engine.md#stats-interval.
const STATS_INTERVAL: Duration = Duration::from_millis(1000);

/// Diagnostic heartbeat cadence: a run that dies mid-session with no disconnect/error/panic logged is
/// otherwise indistinguishable from a genuine unlogged hang. Logging proves this async tick — and thus
/// the whole tokio runtime — was still alive at this timestamp; its absence right before a log goes
/// silent narrows the hang to something blocking synchronously inside this tick (most likely a script
/// handler call that never returns) rather than the runtime itself.
const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(10);

/// Base offset for the Arizona 221/53 `timestamp_ms` so it resembles a plausible `GetTickCount` uptime
/// (the server only requires it to strictly increase within a session); the elapsed ms are added on.
const ARIZONA_TS_BASE: u32 = 200_000_000;

/// RakNet ordering channel for all player sync (on-foot 207, aim 203, spectator 212), off the RPC
/// channel 0; see docs/memory/samp-client/engine.md#sync-channel.
const SYNC_CHANNEL: u8 = 1;

/// Arizona's CEF/validation packet family: raw packet id 220, sub-id in the second byte. Logged
/// verbosely in both directions because the Arizona login flow hinges on it.
const ARIZONA_CEF_PACKET_ID: u8 = 220;

/// Arizona's custom streamer-sync packet id (221). Sub-id 53 = our outbound position report; sub-id
/// 113 = the server assigning us the streamer entity id to report under.
const ARIZONA_SYNC_PACKET_ID: u8 = 221;
const ARIZONA_SYNC_ASSIGN_SUB: u8 = 113;

const SPAWN_HEALTH: u8 = 100;

/// Fallback spawn position for [`Engine::enter_spawned`] when the server never sent `SetSpawnInfo`
/// (decoded from a real Arizona capture); see docs/memory/samp-client/engine.md#fallback-spawn-position.
const FALLBACK_SPAWN_POSITION: Vector3 = Vector3 {
    x: 1765.50,
    y: -1892.70,
    z: 13.56,
};

/// What woke the `select!` in [`Engine::step`].
enum Step {
    Event(Option<RakEvent>),
    SyncTick,
    Reconnect,
    Update,
}

/// A locomotion gait, reverse-engineered from a live GTA-SA/Arizona on-foot (207) capture; see
/// docs/memory/samp-client/engine.md#gait.
#[derive(Clone, Copy)]
struct Gait {
    /// On-foot key bits: 0 = jog (default forward run), `KEY_SPRINT` = sprint, `KEY_WALK` = walk.
    /// INVARIANT: must match `kind` — the engine derives speed from the stick magnitude and these
    /// modifiers, so sprint speed without `KEY_SPRINT` (or run speed with `KEY_WALK`) is a state no
    /// real client can produce.
    keys: u16,
    /// Which band the [`LocomotionModel`] wanders in; it owns the magnitude, so no speed lives here.
    kind: GaitKind,
}

/// SA-MP key bit for holding the sprint control (default run → sprint).
const KEY_SPRINT: u16 = 0x0008;
/// SA-MP key bit for the walk modifier (default run → slow walk).
const KEY_WALK: u16 = 0x0400;
/// The only key bits a [`Gait`] owns. Everything else in the mask belongs to whoever set it (a script
/// holding aim/fire/secondary), so the walker must not clear it — otherwise no script can express
/// aiming, firing or ducking while moving, which a real player does constantly.
const GAIT_OWNED_KEYS: u16 = KEY_SPRINT | KEY_WALK;

/// Slow walk (KEY_WALK held): 1.556 u/s; see docs/memory/samp-client/engine.md#gait-walk.
const GAIT_WALK: Gait = Gait {
    keys: KEY_WALK,
    kind: GaitKind::Walk,
};
/// Default jog (push forward, no modifier): 5.7 u/s; see docs/memory/samp-client/engine.md#gait-jog.
const GAIT_JOG: Gait = Gait {
    keys: 0,
    kind: GaitKind::Jog,
};
/// Sprint (KEY_SPRINT held): 9.56 u/s; see docs/memory/samp-client/engine.md#gait-sprint.
const GAIT_SPRINT: Gait = Gait {
    keys: KEY_SPRINT,
    kind: GaitKind::Sprint,
};

/// Mode bytes at or above this ask for a DIRECT seek — a straight line to the point, no navmesh
/// corridor. This is what `fly.luau` has been doing in script for years: interaction spots, interior
/// floors and dropped-cart positions are routinely off-mesh, so a path query returns nothing and the
/// only way there is to walk at it. Same body, same gait model, same measured velocity as a navmesh
/// walk — the only difference is where the waypoints come from.
pub const DIRECT_MODE_BASE: u8 = 10;

/// Resolve a `walkTo` mode byte (0 = walk, 1 = jog, 2 = sprint; +10 for a direct seek) to its [`Gait`].
fn gait_for_mode(mode: u8) -> Gait {
    match mode % DIRECT_MODE_BASE {
        0 => GAIT_WALK,
        2 => GAIT_SPRINT,
        _ => GAIT_JOG,
    }
}

/// Whether this mode byte asks for a direct seek rather than a navmesh path.
fn is_direct_mode(mode: u8) -> bool {
    mode >= DIRECT_MODE_BASE
}

/// CreateObject RPC id — no typed `samp_proto` event exists for it (its wire format has a variable-
/// length materials/attach tail the simple fixed-field event macro can't express); see
/// [`decode_create_object_prefix`].
const RPC_CREATE_OBJECT: u8 = 44;

/// `WorldVehicleAdd` / `WorldVehicleRemove` — the streamed-vehicle set the 209 election runs over.
const RPC_WORLD_VEHICLE_ADD: u8 = 164;
const RPC_WORLD_VEHICLE_REMOVE: u8 = 165;

/// Decode just the fixed prefix of a CreateObject (RPC 44) body — object id, model id (discarded), world
/// position — and ignore the variable-length tail (rotation, draw distance, conditional attach-offset
/// block, per-material textures) we don't need. Byte-exact with `app`'s `objects` pcap-miner
/// (`crates/app/src/bin/objects.rs`), which mines the same RPC from a capture and is verified against
/// real Arizona sessions. Returns `None` if the payload is too short to hold the prefix.
fn decode_create_object_prefix(payload: &[u8]) -> Option<(u16, Vector3)> {
    if payload.len() < 18 {
        return None;
    }
    let le_f32 = |o: usize| f32::from_le_bytes(payload[o..o + 4].try_into().unwrap());
    let object_id = u16::from_le_bytes([payload[0], payload[1]]);
    Some((
        object_id,
        Vector3 {
            x: le_f32(6),
            y: le_f32(10),
            z: le_f32(14),
        },
    ))
}

/// The sender's half of a walk: which gait was asked for and the bookkeeping needed to report it.
/// The body itself — waypoints, position, heading — lives in [`crate::world::motion`] and is stepped
/// on its own clock; this side only decides how fast to go and turns that into a packet.
struct WalkState {
    gait: Gait,
    /// Where the walk was asked to end, for the arrival log.
    target: Vector3,
    /// When this walk was last sampled, for the measured velocity.
    last_step: Option<Instant>,
    /// Position at the previous sample, so the reported velocity is a real displacement.
    last_position: Vector3,
    /// Latched once the gait has braked to a standstill with path left; the remainder is walked in.
    finishing: bool,
}

/// Idle facing wobble: while the native walker isn't driving movement (the chop/rent/deliver
/// interaction pins), the reported facing holds a small bounded offset from the yaw it had when
/// standing began, re-rolled on an occasional "glance" cadence — a real player standing still still
/// shifts their look direction a little, instead of a perfectly frozen facing. Recomputed only every
/// few seconds, not every tick, so it doesn't defeat the on-foot sync's idle-keepalive send-rate
/// reduction (a continuously-changing quaternion would mark every tick "changed" and flood sends).
struct IdleWobble {
    base_yaw: f32,
    offset: f32,
    next_glance_at: Instant,
}

/// Bound on the idle-glance yaw offset (radians) — small enough to never read as a spin.
const IDLE_WOBBLE_MAX: f32 = 0.09; // ~5°
/// Glance cadence bounds (ms) — how long one glance offset holds before the next re-roll.
const IDLE_GLANCE_MIN_MS: u64 = 1500;
const IDLE_GLANCE_SPAN_MS: u64 = 2500;

/// Horizontal distance to the final target at which the walker asks the locomotion model to brake.
/// The model then ramps down its own chain and snaps to zero, so this must exceed its stopping
/// distance (~3.3 u from sprint cruise) or the bot overshoots the target.
fn brake_distance(kind: GaitKind) -> f32 {
    match kind {
        GaitKind::Walk => 1.0,
        GaitKind::Jog => 1.5,
        GaitKind::Sprint => 3.0,
    }
}

/// The free fall left behind by a "slap" — a `SetPlayerPos(FindZ)` at the bot's own X/Y with only Z
/// changed (an admin `/slap`, or an anti-cheat probe: raise the player in place and watch whether the
/// client free-falls like a real one or hangs in the air like a bot). The reposition itself is a
/// teleport and is applied at once, exactly as the engine applies it; only the fall back down is
/// simulated, stepped once per sync tick by [`Engine::step_slap_reaction`]. See
/// [`Engine::aim_follow`]/[`is_slap`]. Ported from the community "slapfix" idiom.
struct SlapReaction {
    /// Height the fall ends at: the ground under the (unchanged) X/Y.
    // INVARIANT: strictly below the height the fall starts from — a raise is a teleport, never an
    // ascent under gravity. Letting this sit above the start levitates the bot to the slapped height
    // and leaves it hovering, which is the flight signature the reaction exists to avoid.
    ground_z: f32,
    /// Downward speed in units/sec; ramps by [`SLAP_GRAVITY`] each tick, capped at
    /// [`SLAP_TERMINAL_SPEED`].
    fall_speed: f32,
    last_step_at: Instant,
}

/// X/Y must match within this to call a reposition "the same spot" — a genuine teleport moves X/Y
/// too and is left alone (still applied instantly).
const SLAP_XY_EPS: f32 = 0.05;
/// Minimum same-spot Z delta that counts as a slap rather than ordinary `SetPlayerPosFindZ` terrain
/// snapping.
const SLAP_Z_THRESHOLD: f32 = 1.2;
/// GTA:SA free-fall acceleration in units/s²: the engine's per-frame gravity 0.008 at its 50 Hz
/// simulation rate is 0.008 × 50² = 20.
const SLAP_GRAVITY: f32 = 20.0;
/// Downward speed cap (units/sec) for the simulated fall.
const SLAP_TERMINAL_SPEED: f32 = 12.0;
/// Below this much air under the ped, a slap left nothing to fall through and the teleport is the
/// whole reaction.
const SLAP_LANDED_EPS: f32 = 0.05;

/// Is `to` a "slap" relative to `from` — same X/Y, only Z jumped? See [`SlapReaction`].
fn is_slap(from: Vector3, to: Vector3) -> bool {
    (from.x - to.x).abs() <= SLAP_XY_EPS
        && (from.y - to.y).abs() <= SLAP_XY_EPS
        && (to.z - from.z).abs() >= SLAP_Z_THRESHOLD
}

pub(crate) struct Engine<T: Transport> {
    config: ClientConfig,
    transport: T,
    state: ConnectionState,
    pending: VecDeque<ClientEvent>,
    local_id: PlayerId,
    sync: OnFootSync,
    /// The last on-foot sync actually sent, and when — drives adaptive sending (only resend on a
    /// state change, otherwise at the slow idle keepalive cadence).
    last_sync: OnFootSync,
    last_sync_at: Option<Instant>,
    /// The in-vehicle counterparts of `last_sync`/`last_sync_at`: while the script has put the bot in
    /// a vehicle, the engine emits VehicleSync (200) instead of on-foot 207, adaptively gated the same way.
    last_vehicle_sync: VehicleSyncData,
    last_vehicle_sync_at: Option<Instant>,
    /// When the periodic `PACKET_STATS_UPDATE` (205) was last sent; drives the 1 Hz stats cadence.
    last_stats_at: Option<Instant>,
    /// When the diagnostic heartbeat (proof the async runtime is still ticking, independent of any
    /// script logic) was last logged; see [`HEARTBEAT_INTERVAL`].
    last_heartbeat_at: Option<Instant>,
    /// The simulated world: the navmesh the bot walks on and every entity streamed into it.
    world: World,
    /// Read-only picture of the streamed entities, republished each send tick for scripts.
    entities: EntityView,
    /// The active native walk's SENDER side: the gait asked for and the bookkeeping to report it.
    walk: Option<WalkState>,
    /// The local ped's body, stepped by a background task on its own fixed clock; see
    /// [`crate::world::motion`].
    motion: MotionHandle,
    /// Idle-facing-wobble anchor, live only while `walk` is `None`; see [`IdleWobble`].
    idle_wobble: Option<IdleWobble>,
    /// Speed/analog/animation realism model; see [`LocomotionModel`].
    locomotion: LocomotionModel,
    /// In-progress slap reaction, if any; see [`SlapReaction`].
    slap: Option<SlapReaction>,
    sync_timer: Option<Interval>,
    reconnect_timer: Option<Pin<Box<Sleep>>>,
    closed: bool,
    /// Packet-handler registry: scripts/observers intercept RPCs here before the FSM sees them.
    registry: PacketRegistry,
    /// Fires the registry's `on_update` handlers; armed only when handlers are registered.
    update_timer: Option<Interval>,
    /// Shared local-player state mirrored to/from `sync`, exposing `getBot*`/`setBot*` to scripts.
    bot_state: Option<SharedLocalPlayer>,
    /// Native aim-sync emulation (always on — standard client behaviour).
    aim: AimSync,
    /// Standard-client emulation: ClientCheck answers, weapon inventory, score-ping, vehicle
    /// ownership (always on; acts once a shared bot state is attached).
    emulation: ClientEmulation,
    /// True between join and spawn: we spectate (sending spectator sync) until the server drives the
    /// spawn via `RequestSpawnResponse(allow==2)`, or the script calls `sampSpawnPlayer()`.
    spectating: bool,
    /// Mirrors RakSAMP Lite's `g_bSpawnRequested`: set when we send `RequestSpawn` (via
    /// `sampSpawnPlayer()`), so a non-2 `RequestSpawnResponse` allow still spawns us.
    spawn_requested: bool,
    /// Mirrors RakSAMP Lite's `g_bSpectating`: last `TogglePlayerSpectating` from the server, whose
    /// `1 → 0` transition is the Arizona spawn trigger.
    server_spectating: bool,
    /// Position from `SetSpawnInfo` (RPC 68); `enter_spawned` loads it into the sync position,
    /// mirroring RakSAMP Lite's `Net_Spawn` — without this the bot reports 0,0,0 after spawn.
    spawn_info_position: Option<Vector3>,
    /// When the pre-spawn window began — the class response or the server's spectate-on toggle,
    /// whichever lands first; drives the optional self-spawn fallback (`config.self_spawn_timeout`).
    class_selected_at: Option<Instant>,
    /// Engine-creation instant; the monotonic base for the Arizona 221/53 `timestamp_ms` field.
    created_at: Instant,
    /// Streamer entity id the server assigns via inbound `221/113` — our outbound `221/53` must report
    /// it, not our player id. See docs/memory/samp-client/engine.md#arizona-streamer-id.
    arizona_streamer_id: Option<u16>,
    /// Consecutive reconnect attempts since the last stable session. Caps a kick/reject loop so a
    /// server that keeps dropping us right back out doesn't spin the client forever.
    reconnect_attempts: u32,
    /// When we last reached `Spawned`; a session lasting ≥[`STABLE_SESSION`] before dropping resets the
    /// reconnect-attempt cap. See docs/memory/samp-client/engine.md#on-disconnect-reconnect.
    connected_since: Option<Instant>,
}

/// Give up reconnecting after this many consecutive attempts without a stable session — the drop is a
/// kick/ban/block that reconnecting won't fix, so stop instead of looping.
const MAX_RECONNECT_ATTEMPTS: u32 = 5;
/// A session must last at least this long (spawned) to count as "stable" and reset the attempt cap.
const STABLE_SESSION: Duration = Duration::from_secs(60);

impl<T: Transport> Engine<T> {
    pub(crate) fn new(config: ClientConfig, transport: T) -> Self {
        let mut pending = VecDeque::new();
        pending.push_back(ClientEvent::StateChanged(ConnectionState::Connecting));
        let seed = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0x1234_5678);
        let nav = config.navmesh.as_ref().and_then(|path| {
            match std::fs::File::open(path)
                .map_err(|e| e.to_string())
                .and_then(|f| {
                    sa_nav::NavMesh::load(&mut std::io::BufReader::new(f)).map_err(|e| e.to_string())
                }) {
                Ok(mesh) => {
                    tracing::info!(path = %path.display(), polys = mesh.polys.len(), "navmesh loaded");
                    Some(sa_nav::NavQuery::new(mesh))
                }
                Err(error) => {
                    tracing::warn!(path = %path.display(), %error, "navmesh load failed — walkTo disabled");
                    None
                }
            }
        });
        let aim = AimSync::new(seed);
        let emulation = ClientEmulation::new(seed ^ 0x5555_5555_5555_5555, Instant::now());
        Self {
            config,
            transport,
            state: ConnectionState::Connecting,
            pending,
            local_id: PlayerId::default(),
            sync: OnFootSync::default(),
            last_sync: OnFootSync::default(),
            last_sync_at: None,
            last_vehicle_sync: VehicleSyncData::default(),
            last_vehicle_sync_at: None,
            last_stats_at: None,
            last_heartbeat_at: None,
            world: World {
                nav,
                ..World::default()
            },
            entities: EntityView::default(),
            walk: None,
            motion: MotionHandle::default(),
            idle_wobble: None,
            locomotion: LocomotionModel::new(rand::random()),
            slap: None,
            sync_timer: None,
            reconnect_timer: None,
            closed: false,
            registry: PacketRegistry::new(),
            update_timer: None,
            bot_state: None,
            aim,
            emulation,
            spectating: false,
            spawn_requested: false,
            server_spectating: false,
            spawn_info_position: None,
            class_selected_at: None,
            created_at: Instant::now(),
            arizona_streamer_id: None,
            reconnect_attempts: 0,
            connected_since: None,
        }
    }

    /// Share a [`SharedLocalPlayer`] with the script engine: the engine mirrors `sync` into it and
    /// reads `setBot*` writes back out of it.
    pub(crate) fn with_bot_state(mut self, state: SharedLocalPlayer) -> Self {
        self.bot_state = Some(state);
        self
    }

    /// Attach a packet-handler registry. Arms the `on_update` timer if the registry registered any
    /// periodic handlers.
    pub(crate) fn with_registry(mut self, registry: PacketRegistry) -> Self {
        if registry.wants_update() {
            let mut interval = time::interval(UPDATE_INTERVAL);
            interval.set_missed_tick_behavior(MissedTickBehavior::Delay);
            self.update_timer = Some(interval);
        }
        self.registry = registry;
        self
    }

    /// Start stepping the body on its own clock, off the engine task.
    ///
    /// The body is `Send` and shares nothing with the script VM, so it can run on another worker while
    /// this task is blocked decoding a burst or waiting on a socket. The stepper holds only a weak
    /// reference and retires when the client drops.
    pub(crate) fn spawn_physics(&self) {
        let motion = self.motion.weak();
        tokio::spawn(async move {
            let period = Duration::from_secs_f32(PHYSICS_DT);
            let mut interval = time::interval(period);
            // Burst, not Delay: a hitch must not cost the body simulated time — the remote dead-reckons
            // straight through it, so the catch-up is the whole point. `Motion` bounds it internally.
            interval.set_missed_tick_behavior(MissedTickBehavior::Burst);
            let mut last = Instant::now();
            loop {
                interval.tick().await;
                let now = Instant::now();
                let elapsed = now.duration_since(last);
                last = now;
                if !motion.advance(elapsed) {
                    return;
                }
            }
        });
    }

    /// Shared read-only view of the streamed entities, for the script host.
    pub(crate) fn entities(&self) -> EntityView {
        self.entities.clone()
    }

    pub(crate) fn state(&self) -> &ConnectionState {
        &self.state
    }

    #[cfg(test)]
    pub(crate) fn reconnect_scheduled(&self) -> bool {
        self.reconnect_timer.is_some()
    }

    #[cfg(test)]
    fn motion(&self) -> &MotionHandle {
        &self.motion
    }

    // The test-only view of the engine's interior. Tests address the simulation through these rather
    // than through fields so that relocating the state they name — into the world, into a session — is
    // a change to this block instead of to every assertion in engine/tests.rs.
    #[cfg(test)]
    fn sync_mut(&mut self) -> &mut OnFootSync {
        &mut self.sync
    }

    #[cfg(test)]
    fn walking(&self) -> bool {
        self.walk.is_some()
    }

    #[cfg(test)]
    fn walk_mut(&mut self) -> Option<&mut WalkState> {
        self.walk.as_mut()
    }

    #[cfg(test)]
    fn slap_active(&self) -> bool {
        self.slap.is_some()
    }

    #[cfg(test)]
    fn slap_mut(&mut self) -> Option<&mut SlapReaction> {
        self.slap.as_mut()
    }

    #[cfg(test)]
    fn locomotion_mut(&mut self) -> &mut LocomotionModel {
        &mut self.locomotion
    }

    #[cfg(test)]
    fn set_nav(&mut self, nav: sa_nav::NavQuery) {
        self.world.nav = Some(nav);
    }

    #[cfg(test)]
    fn set_state(&mut self, state: ConnectionState) {
        self.state = state;
    }

    #[cfg(test)]
    fn pending(&self) -> &VecDeque<ClientEvent> {
        &self.pending
    }

    pub(crate) async fn next_event(&mut self) -> Option<ClientEvent> {
        loop {
            if let Some(event) = self.pending.pop_front() {
                return Some(event);
            }
            if self.closed && self.reconnect_timer.is_none() {
                return None;
            }
            match self.step().await {
                Step::Event(Some(event)) => self.on_rak_event(event).await,
                Step::Event(None) => self.on_transport_closed(),
                Step::SyncTick => self.on_sync_tick(false).await,
                Step::Reconnect => self.on_reconnect().await,
                Step::Update => self.registry.tick(),
            }
            self.flush_outbox().await;
            self.poll_bot_actions().await;
        }
    }

    /// Push the authoritative `sync` fields into the shared local-player state (so `getBot*` reflects
    /// what the server set, e.g. the spawn position).
    pub(super) fn mirror_to_state(&self) {
        if let Some(state) = &self.bot_state {
            let mut s = state.borrow_mut();
            s.on_foot.position = self.sync.position;
            s.on_foot.quaternion = self.sync.quaternion;
            s.on_foot.health = self.sync.health;
            s.on_foot.armour = self.sync.armour;
            s.on_foot.weapon = self.sync.weapon.0;
        }
    }

    /// Pull script `setBot*` writes back into `sync` before sending it.
    pub(super) fn mirror_from_state(&mut self) {
        if let Some(state) = &self.bot_state {
            let s = state.borrow();
            self.sync.position = s.on_foot.position;
            self.sync.quaternion = s.on_foot.quaternion;
            // Script-driven keypresses (e.g. an Arizona ALT interaction): the server derives
            // OnPlayerKeyStateChange from this field, so a script pulses `setBotKeys(mask)` then 0.
            self.sync.keys = s.on_foot.keys;
            self.sync.health = s.on_foot.health;
            self.sync.armour = s.on_foot.armour;
            self.sync.weapon = WeaponId(s.on_foot.weapon);
            self.sync.move_speed = s.on_foot.move_speed;
            self.sync.animation_id = s.on_foot.animation_id;
            self.sync.animation_flags = s.on_foot.animation_flags;
        }
    }

    /// Mirror the native aim state into the shared local-player model so `LocalPlayer.aim` reflects
    /// the camera the client is about to report.
    pub(super) fn mirror_aim_to_state(&self) {
        if let Some(state) = self.bot_state.as_ref() {
            state.borrow_mut().aim = *self.aim.aim();
        }
    }

    /// Act on `updateSync()` / `reconnect(ms)` flags scripts set on the bot state.
    pub(super) async fn poll_bot_actions(&mut self) {
        let (force_sync, reconnect_in, spawn_requested, walk_to, walk_stop) = match &self.bot_state
        {
            Some(state) => {
                let mut s = state.borrow_mut();
                (
                    std::mem::take(&mut s.force_sync),
                    s.reconnect_in_ms.take(),
                    std::mem::take(&mut s.spawn_requested),
                    s.walk_to.take(),
                    std::mem::take(&mut s.walk_stop),
                )
            }
            None => return,
        };
        if walk_stop {
            self.stop_walk();
        }
        if let Some((target, mode)) = walk_to {
            self.start_walk(target, mode);
        }
        // `sampSpawnPlayer()` — explicit manual spawn override (usually unneeded; the server drives the
        // spawn after login); sets `spawn_requested` so a non-2 allow still spawns us.
        if spawn_requested && self.spectating {
            self.request_spawn().await;
        }
        if force_sync && matches!(self.state, ConnectionState::Spawned) {
            self.on_sync_tick(true).await;
        }
        if let Some(ms) = reconnect_in {
            self.transition(ConnectionState::Disconnected);
            self.closed = true;
            let _ = self.transport.disconnect().await;
            self.reconnect_timer = Some(Box::pin(time::sleep(Duration::from_millis(ms))));
        }
    }

    async fn step(&mut self) -> Step {
        let closed = self.closed;
        let spawned = matches!(self.state, ConnectionState::Spawned);
        // Arizona pre-spawn spectating also drives the sync tick (to send spectator sync).
        let ticking = spawned || self.spectating;
        let has_reconnect = self.reconnect_timer.is_some();
        let has_sync = self.sync_timer.is_some();
        let has_update = self.update_timer.is_some();
        let Engine {
            transport,
            sync_timer,
            reconnect_timer,
            update_timer,
            ..
        } = self;
        tokio::select! {
            biased;
            _ = poll_sleep(reconnect_timer), if has_reconnect => Step::Reconnect,
            _ = poll_interval(sync_timer), if ticking && has_sync => Step::SyncTick,
            _ = poll_interval(update_timer), if has_update => Step::Update,
            event = transport.recv(), if !closed => Step::Event(event),
        }
    }

    /// Whether the bot is currently in a vehicle (from the shared local-player state).
    pub(super) fn in_vehicle(&self) -> bool {
        self.bot_state
            .as_ref()
            .is_some_and(|s| s.borrow().in_vehicle())
    }

    /// Current interior id (0 = the outdoor world), as last set by `SetInterior`.
    pub(super) fn interior(&self) -> u8 {
        self.bot_state
            .as_ref()
            .map_or(0, |state| state.borrow().interior)
    }

    pub(super) fn transition(&mut self, next: ConnectionState) {
        self.state = next.clone();
        // Reaching Spawned is a stable milestone: start the session clock so a long, healthy session
        // resets the reconnect-attempt cap when it eventually drops (see on_disconnect).
        if matches!(self.state, ConnectionState::Spawned) {
            self.connected_since = Some(Instant::now());
        }
        // Arm the sync tick while Spawned (on-foot sync) OR during the Arizona pre-spawn spectate
        // phase (spectator sync keepalive while waiting for the server to drive the spawn).
        if matches!(self.state, ConnectionState::Spawned) || self.spectating {
            if self.sync_timer.is_none() {
                let period = self.config.sync_interval.max(Duration::from_millis(1));
                let mut interval = time::interval(period);
                interval.set_missed_tick_behavior(MissedTickBehavior::Delay);
                self.sync_timer = Some(interval);
            }
        } else {
            self.sync_timer = None;
        }
        self.pending.push_back(ClientEvent::StateChanged(next));
    }
}

async fn poll_sleep(timer: &mut Option<Pin<Box<Sleep>>>) {
    match timer {
        Some(sleep) => sleep.as_mut().await,
        None => future::pending().await,
    }
}

async fn poll_interval(timer: &mut Option<Interval>) {
    match timer {
        Some(interval) => {
            interval.tick().await;
        }
        None => future::pending::<()>().await,
    }
}

/// Decode a `Decode` packet body, warning and returning `None` on failure.
fn decode_or_warn<T: Decode>(payload: &[u8], what: &'static str) -> Option<T> {
    match T::decode(payload) {
        Ok(value) => Some(value),
        Err(error) => {
            tracing::warn!(%error, "failed to decode {what}");
            None
        }
    }
}

/// Decode a `WireDecode` event body, warning and returning `None` on failure.
fn decode_event_or_warn<T: WireDecode>(payload: &[u8], what: &'static str) -> Option<T> {
    match decode_event::<T>(payload) {
        Ok(value) => Some(value),
        Err(error) => {
            tracing::warn!(%error, "failed to decode {what}");
            None
        }
    }
}

/// Render a byte slice as contiguous lowercase hex for wire-level debug logs.
fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// Decode an inbound RPC body into a human-readable struct for logging — turns the raw bytes into
/// named fields (cp1251 text rendered as strings). Covers the dialog/chat specials plus everything in
/// the typed incoming registry; returns `None` for ids with no decoder.
fn decode_inbound_rpc(id: u8, payload: &[u8]) -> Option<String> {
    use samp_proto::events::FieldValue as F;
    match RpcId::try_from(id) {
        Ok(RpcId::ShowDialog) => ShowDialog::decode(payload).ok().map(|d| {
            format!(
                "ShowDialog {{ dialog_id: {}, style: {}, title: {:?}, button1: {:?}, button2: {:?} }}",
                d.dialog_id,
                d.style,
                decode_cp1251(&d.title),
                decode_cp1251(&d.button1),
                decode_cp1251(&d.button2),
            )
        }),
        Ok(RpcId::ClientMessage) => ServerMessage::decode(payload).ok().map(|m| {
            format!(
                "ClientMessage {{ color: {:08X}, text: {:?} }}",
                m.color,
                decode_cp1251(&m.text)
            )
        }),
        Ok(RpcId::Chat) => ChatMessage::decode(payload).ok().map(|m| {
            format!(
                "Chat {{ player_id: {}, text: {:?} }}",
                m.player_id.0,
                decode_cp1251(&m.text)
            )
        }),
        _ => {
            let (name, fields) = events::decode_incoming(id, payload)?.ok()?;
            let body = fields
                .iter()
                .map(|f| match f {
                    F::Bytes(b) => format!("{:?}", decode_cp1251(b)),
                    other => format!("{other:?}"),
                })
                .collect::<Vec<_>>()
                .join(", ");
            Some(format!("{name} {{ {body} }}"))
        }
    }
}

#[cfg(test)]
mod tests;
