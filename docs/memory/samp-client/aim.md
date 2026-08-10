# samp-client/aim.rs

## Module (native aim-sync emulation)
<anchor: module>

Native aim-sync emulation, ported from `aim_fix_updated.lua`.

A real SA-MP client streams an aim-sync packet describing where its camera points. A bare bot
never sends one, which looks unnatural. This makes the client send a believable aim packet at
roughly 1 Hz (a random 800–1200 ms period): a camera placed ~2 units behind the bot per its
facing, jittered by a small random offset, looking back at the bot, and re-pinned to the bot's
position every tick so a walking bot's camera travels with it. When the server repositions the
bot (`SetPlayerPos`/camera RPCs) the aim is regenerated and sent promptly.

The cadence is measured, not guessed: samp.dll's sender fires every ~1000 ms on foot
(`CLocalPlayer_SendAimSync_203` @0x10005040, dispatched from `NetGame_Process` @0x10007270), and
across 1,887,286 real 203 packets the per-session 203/207 ratio is p25 0.082 / p50 0.096 / p75
0.121 — ours lands at 0.095. Sending rarely, or suppressing while moving (both of which this
module used to do), puts the ratio below the real 5th percentile: a walking bot emitted none at
all. A real client sends 203 throughout a walk, so this must never be gated on standing still.

The aim it produces is the high-level [`AimData`]; the driver mirrors it into
[`crate::state::LocalPlayer::aim`] and (with client emulation) tweaks it before [`AimSync::encode`]
packs it into the wire [`AimSyncData`].

## AimSync (usage)
<anchor: aimsync-usage>

Aim-sync state machine. Construct with [`AimSync::new`], feed it position updates and server
repositions, then each sync tick poll [`AimSync::due`] and, when due, [`AimSync::encode`].

## due
<anchor: due>

Whether an aim send is due, rescheduling the next one 800–1200 ms out. Purely timer-based — it
is deliberately NOT gated on movement (see the module note). When this returns `true`, mutate
[`Self::aim_mut`] if desired and then call [`Self::encode`].

---
