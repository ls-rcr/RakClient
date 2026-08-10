//! RakSAMP native bot getters/setters, backed by the shared [`SharedLocalPlayer`] the driver mirrors.
//!
//! State (position/rotation/nick/…) reads and writes go straight to the shared cell; `updateSync`
//! and `reconnect` set flags the driver polls. Money/interior/camera are populated from the relevant
//! RPCs by the driver's `track_state`.

use mlua::Lua;
use samp_client::{InVehicleData, SharedLocalPlayer};
use samp_proto::{Quaternion, Vector3};

/// Install `getBot*`/`setBot*`, `getServerAddress`, `updateSync`, `reconnect` bound to `state`.
/// Expose the streamed-entity view to scripts.
///
/// Scripts have had to rebuild these pools in Luau from raw RPCs (`shim.luau` for pickups and actors,
/// `helpers.luau` for vehicles and objects) because the host offered nothing. This is the same data the
/// client already tracks for pathing, published read-only once per send tick.
pub fn install_entity_bindings(lua: &Lua, entities: samp_client::EntityView) -> mlua::Result<()> {
    let globals = lua.globals();

    let list = |lua: &Lua, refs: &[samp_client::EntityRef]| -> mlua::Result<mlua::Table> {
        let out = lua.create_table()?;
        for (i, e) in refs.iter().enumerate() {
            let row = lua.create_table()?;
            row.set("id", e.id)?;
            row.set("x", e.position.x)?;
            row.set("y", e.position.y)?;
            row.set("z", e.position.z)?;
            if let Some(model) = e.model {
                row.set("model", model)?;
            }
            out.set(i + 1, row)?;
        }
        Ok(out)
    };

    let view = entities.clone();
    globals.set(
        "getStreamedPlayers",
        lua.create_function(move |lua, ()| list(lua, &view.read().players))?,
    )?;
    let view = entities;
    globals.set(
        "getStreamedVehicles",
        lua.create_function(move |lua, ()| list(lua, &view.read().vehicles))?,
    )?;
    Ok(())
}

pub fn install_bindings(lua: &Lua, state: SharedLocalPlayer) -> mlua::Result<()> {
    let globals = lua.globals();

    let s = state.clone();
    globals.set(
        "getBotNick",
        lua.create_function(move |_, ()| Ok(s.borrow().nick.clone()))?,
    )?;
    let s = state.clone();
    globals.set(
        "setBotNick",
        lua.create_function(move |_, nick: String| {
            s.borrow_mut().nick = nick;
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "getServerAddress",
        lua.create_function(move |_, ()| Ok(s.borrow().server_addr.to_string()))?,
    )?;

    let s = state.clone();
    globals.set(
        "getBotPosition",
        lua.create_function(move |_, ()| {
            let p = s.borrow().on_foot.position;
            Ok((p.x, p.y, p.z))
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "setBotPosition",
        lua.create_function(move |_, (x, y, z): (f32, f32, f32)| {
            s.borrow_mut().on_foot.position = Vector3 { x, y, z };
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "setBotVelocity",
        lua.create_function(move |_, (x, y, z): (f32, f32, f32)| {
            s.borrow_mut().on_foot.move_speed = Vector3 { x, y, z };
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "setBotFacing",
        // Face the horizontal direction (dx, dy) — e.g. the direction of travel. Builds the on-wire
        // quaternion about world +Z (w=cos(φ/2), z=sin(φ/2), φ=atan2(dx, dy)), the same encoding the
        // real client sends and the native walker uses. Remote players derive our facing entirely from
        // this quaternion, so a script doing setBotPosition-driven moves (the chop walk-in) can finally
        // face where it walks instead of sliding sideways. No-op for a zero vector (keeps current facing).
        lua.create_function(move |_, (dx, dy): (f32, f32)| {
            if dx != 0.0 || dy != 0.0 {
                let phi = dx.atan2(dy);
                let (sin, cos) = (phi * 0.5).sin_cos();
                s.borrow_mut().on_foot.quaternion = Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: sin,
                    w: cos,
                };
            }
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "setBotKeys",
        lua.create_function(move |_, keys: u16| {
            // Set the on-foot key bitmask the next sync reports; the server turns it into
            // OnPlayerKeyStateChange. Pulse (mask → updateSync → 0 → updateSync) to emulate a press.
            s.borrow_mut().on_foot.keys = keys;
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "getBotKeys",
        lua.create_function(move |_, ()| Ok(s.borrow().on_foot.keys))?,
    )?;
    let s = state.clone();
    globals.set(
        "setBotAnimation",
        lua.create_function(move |_, (id, flags): (u16, u16)| {
            let mut bot = s.borrow_mut();
            bot.on_foot.animation_id = id;
            bot.on_foot.animation_flags = flags;
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "getBotRotation",
        lua.create_function(move |_, ()| {
            // Yaw (Z) from the on-foot quaternion, in degrees.
            let q = s.borrow().on_foot.quaternion;
            let yaw = (2.0 * (q.w * q.z + q.x * q.y))
                .atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                .to_degrees();
            Ok(yaw)
        })?,
    )?;

    let s = state.clone();
    globals.set(
        "getBotVehicle",
        lua.create_function(move |_, ()| Ok(s.borrow().vehicle_id() as i32))?,
    )?;
    let s = state.clone();
    globals.set(
        "setBotVehicle",
        lua.create_function(move |_, (id, seat): (u16, i32)| {
            // id 0 means "on foot"; otherwise record the vehicle the script put us in.
            let mut bot = s.borrow_mut();
            bot.vehicle = (id != 0).then(|| InVehicleData {
                id,
                seat: seat as u8,
                ..InVehicleData::default()
            });
            Ok(())
        })?,
    )?;

    let s = state.clone();
    globals.set(
        "getBotWeapon",
        lua.create_function(move |_, ()| Ok(s.borrow().weapons.current as i32))?,
    )?;
    let s = state.clone();
    globals.set(
        "setBotWeapon",
        // REFUSES to arm anything. The local inventory mirrors ONLY what the server grants us via
        // GivePlayerWeapon / SetPlayerArmedWeapon / SetWeaponAmmo / ResetPlayerWeapons — a weapon we were
        // never given is a fabrication the anti-cheat catches as "Чит на оружие(15/1)". This binding used to
        // arm locally and its doc even suggested faking the rented chainsaw (id 9) for the лесопилка chop,
        // which is exactly the kick the sawmill grind hit and documented (walk.luau: "Do NOT arm the chainsaw
        // — it's a JOB TOOL; the RPC-131 pickup is the trigger"). Job interactions are driven by the pickup
        // report, never by holding the tool. Kept as a loud no-op so an existing call can't silently fake one.
        lua.create_function(move |_, (id, _ammo): (u8, Option<u16>)| {
            let _ = &s;
            tracing::warn!(
                weapon = id,
                "setBotWeapon ignored — the client never arms a weapon the server did not grant \
                 (faking one is the «Чит на оружие» kick); job actions use the RPC-131 pickup instead"
            );
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "getBotMoney",
        lua.create_function(move |_, ()| Ok(s.borrow().money))?,
    )?;
    let s = state.clone();
    globals.set(
        "getBotInterior",
        lua.create_function(move |_, ()| Ok(s.borrow().interior as i32))?,
    )?;
    let s = state.clone();
    globals.set(
        "getBotCameraPos",
        lua.create_function(move |_, ()| {
            let p = s.borrow().camera_pos;
            Ok((p.x, p.y, p.z))
        })?,
    )?;

    let s = state.clone();
    globals.set(
        "updateSync",
        lua.create_function(move |_, ()| {
            s.borrow_mut().force_sync = true;
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "sampSpawnPlayer",
        lua.create_function(move |_, ()| {
            s.borrow_mut().spawn_requested = true;
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "walkTo",
        // Native navmesh walk (driver-side): plans a path over the loaded .nav and moves with a real
        // GTA-SA gait — walk/jog/sprint speeds + forward analog + movement keys + animation, reversed
        // from a live on-foot (207) capture. Optional `mode`: "walk" | "jog" (default) | "sprint".
        // Fire-and-forget; poll `isWalking()`. Prefix the mode with "direct " to walk STRAIGHT at the
        // point instead of pathing over the navmesh — for spots the bake never covered (interaction
        // pins, interiors, dropped carts), which is why scripts grew their own position-writing mover.
        lua.create_function(move |_, (x, y, z, mode): (f32, f32, f32, Option<String>)| {
            let raw = mode.as_deref().unwrap_or("jog");
            let (direct, gait) = match raw.strip_prefix("direct") {
                Some(rest) => (true, rest.trim()),
                None => (false, raw),
            };
            let mode = match gait {
                "walk" => 0u8,
                "sprint" => 2u8,
                _ => 1u8, // jog: the default forward run
            } + if direct {
                samp_client::WALK_MODE_DIRECT
            } else {
                0
            };
            s.borrow_mut().walk_to = Some((Vector3 { x, y, z }, mode));
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "walkStop",
        lua.create_function(move |_, ()| {
            s.borrow_mut().walk_stop = true;
            Ok(())
        })?,
    )?;
    let s = state.clone();
    globals.set(
        "isWalking",
        lua.create_function(move |_, ()| Ok(s.borrow().walking))?,
    )?;
    let s = state;
    globals.set(
        "reconnect",
        lua.create_function(move |_, ms: u64| {
            s.borrow_mut().reconnect_in_ms = Some(ms);
            Ok(())
        })?,
    )?;

    // Global state manager: a cross-process key→string store every rakclient on this machine shares.
    // One file per key under RAKCLIENT_SHARED_DIR (default <temp>/rakclient-shared), written
    // temp-then-rename so a concurrent reader never sees a torn value. Luau has no io, so this is the
    // host-provided side channel scripts build ANY multi-client coordination on — leader/follower
    // rendezvous, work claims, roster presence — not a job-specific hook.
    fn shared_key_path(dir: &std::path::Path, key: &str) -> Option<std::path::PathBuf> {
        key.chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
            .then(|| dir.join(key))
    }
    let shared_dir = std::env::var("RAKCLIENT_SHARED_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::env::temp_dir().join("rakclient-shared"));
    let dir = shared_dir.clone();
    globals.set(
        "sharedSet",
        lua.create_function(move |_, (key, value): (String, String)| {
            let Some(path) = shared_key_path(&dir, &key) else {
                return Ok(false);
            };
            let write = || -> std::io::Result<()> {
                std::fs::create_dir_all(&dir)?;
                // Per-process tmp: two rakclient processes writing the SAME key (e.g. both racing to
                // claim one tree) must not share `<key>.tmp` — a shared tmp tears/clobbers and one
                // rename loses. A unique suffix keeps each writer's temp-then-rename atomic.
                let tmp = path.with_extension(format!("tmp.{}", std::process::id()));
                std::fs::write(&tmp, &value)?;
                std::fs::rename(&tmp, &path)?;
                Ok(())
            };
            Ok(write().is_ok())
        })?,
    )?;
    let dir = shared_dir.clone();
    globals.set(
        "sharedGet",
        lua.create_function(move |_, key: String| {
            Ok(shared_key_path(&dir, &key).and_then(|p| std::fs::read_to_string(p).ok()))
        })?,
    )?;
    let dir = shared_dir;
    globals.set(
        "sharedDelete",
        lua.create_function(move |_, key: String| {
            let Some(path) = shared_key_path(&dir, &key) else {
                return Ok(false);
            };
            Ok(std::fs::remove_file(path).is_ok())
        })?,
    )?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The mode STRING is the contract between Luau and the driver, and nothing else crosses it: the
    /// Rust walker tests build the mode byte by hand, so a binding that silently dropped the "direct"
    /// prefix once shipped green while every off-mesh walk failed.
    #[test]
    fn walk_to_encodes_the_gait_and_the_direct_prefix() {
        let lua = Lua::new();
        let state = samp_client::LocalPlayer::shared(
            "Tester".to_string(),
            "127.0.0.1:7777".parse().expect("addr"),
        );
        install_bindings(&lua, state.clone()).expect("bindings install");

        let mode_for = |arg: &str| {
            lua.load(format!("walkTo(1, 2, 3, {arg})"))
                .exec()
                .expect("walkTo call");
            state.borrow_mut().walk_to.take().expect("walk_to set").1
        };

        assert_eq!(mode_for("\"walk\""), 0);
        assert_eq!(mode_for("nil"), 1, "jog is the default");
        assert_eq!(mode_for("\"jog\""), 1);
        assert_eq!(mode_for("\"sprint\""), 2);

        let direct = samp_client::WALK_MODE_DIRECT;
        assert_eq!(mode_for("\"direct walk\""), direct);
        assert_eq!(mode_for("\"direct jog\""), direct + 1);
        assert_eq!(mode_for("\"direct sprint\""), direct + 2);
    }
}
