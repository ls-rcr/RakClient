--!strict
-- Bundled "fly to coords" movement API for the headless bot. Fully typed against `host.d.luau`
-- (the host-globals definition file), so it type-checks under `luau analyze --definitions=host.d.luau`.
--
-- This module used to BE the movement: a 10 Hz loop writing setBotPosition/setBotVelocity/setBotFacing
-- itself, because the native walker could only reach points the navmesh bake covered — and the spots
-- that matter (interaction pins, interiors, dropped carts) are routinely off it. That made two
-- simulations of one body, which is how the speeds could disagree.
-- The host now walks straight at an off-mesh point natively (`walkTo(..., "direct <gait>")`) with the
-- real gait and a velocity measured from actual displacement, so this module is a thin adapter over
-- it. The API is unchanged; callers did not move.
--
-- `fly.to(x, y, z, opts)` BLOCKS the calling coroutine (uses `wait()`), so call it from inside a task
-- (`newTask`). It returns true on arrival, false if `fly.stop()` interrupted it. Flying only has an
-- effect once the bot is Spawned (`updateSync` is a no-op before that); `fly.to` waits for spawn.

require("addon") -- wait()/newTask scheduler

-- The full movement-tuning shape (all fields required); `FlyOpts` is its all-optional public form.
type FlyConfig = {
    speed: number, -- units per SECOND at full ramp (time-integrated; never a per-tick step)
    tick: number, -- ms between on-foot syncs
    ramp: number, -- per-tick ramp of speed+velocity
    animId: number, -- on-foot walk animation index
    animFlags: number, -- on-foot walk animation flags
    arrive: number, -- stop when within this many units of the target
}

export type FlyOpts = {
    speed: number?,
    tick: number?,
    ramp: number?,
    animId: number?,
    animFlags: number?,
    arrive: number?,
}

-- The module's public API shape.
export type Fly = {
    distance: (x: number, y: number, z: number) -> number,
    waitSpawn: () -> (),
    stop: () -> (),
    to: (tx: number, ty: number, tz: number, opts: FlyOpts?) -> boolean,
}

local fly: Fly = {} :: Fly

-- Movement is TIME-INTEGRATED at a LEGAL GTA-SA gait speed. The old tuning (a per-tick `step = 8` with a
-- declared move_speed of 0.9) was measured against a real-client capture and was wrong on both axes:
--   * `advance = step * ramp` carried NO dt term, so every sync moved exactly 8.000 units regardless of
--     whether 32 ms or 404 ms had elapsed — 100% of our moving packets (78/78) breached the AC's ~2-unit
--     on-foot snap threshold, versus 0.413 u median / 9.9% breach for the real client. That per-packet
--     displacement is precisely what "Телепорт - Пешком(2 / 1)" measures.
--   * a declared move_speed of 0.9 implies 45 u/s under the samp `move_speed = speed / 50` invariant —
--     4.7x GAIT_SPRINT's ceiling of 0.1912 (9.56 u/s, driver.rs:122-124). Measured real-client ground
--     speed: 3.72 u/s median, 12.38 u/s p99.
-- So: integrate over ACTUAL elapsed time, cap at the sprint ceiling, and derive move_speed from the
-- distance we really covered. Travel is ~2.6x slower than the old 25 u/s — that is the intended trade.
local SPRINT_SPEED = 9.56 -- u/s — GAIT_SPRINT, the fastest legal on-foot gait (driver.rs GAIT_SPRINT)

-- The native walker stops within its own arrival radius, so no caller can be satisfied tighter than
-- this by walking alone; anything finer has to re-pin itself afterwards.
local ARRIVE_FLOOR = 1.0

local DEFAULTS: FlyConfig = {
    speed = SPRINT_SPEED, -- units per SECOND at full ramp, integrated by dt (never a per-tick jump)
    tick = 100, -- ms between on-foot syncs — matches the real client's ~93 ms median moving cadence
    ramp = 0.25, -- per-tick ramp of speed+velocity so short hops still accelerate quickly
    animId = 1189,
    animFlags = 7684,
    arrive = 1.0, -- final snap stays UNDER the ~2 u snap threshold (was 2.0, and effectively 8.0 via `step`)
}

-- A shared flag so `fly.stop()` (or a new `fly.to`) interrupts an in-flight move.
local flying = false


-- Straight-line distance from the bot to (x, y, z).
function fly.distance(x: number, y: number, z: number): number
    local bx, by, bz = getBotPosition()
    local dx, dy, dz = x - bx, y - by, z - bz
    return math.sqrt(dx * dx + dy * dy + dz * dz)
end

-- Block until the bot is spawned (position is non-zero); returns immediately if already spawned.
function fly.waitSpawn()
    while true do
        local x, y, z = getBotPosition()
        if not (x == 0 and y == 0 and z == 0) then
            return
        end
        wait(300)
    end
end

-- Interrupt any in-progress `fly.to` and clear the reported velocity/animation.
function fly.stop()
    flying = false
    walkStop()
    setBotVelocity(0, 0, 0)
    setBotAnimation(1189, 0x8004) -- idle stance, not (0,0): anim 0 never appears in real on-foot traffic
    updateSync()
end

-- Blocking move to (x, y, z). Returns true on arrival, false if interrupted by `fly.stop()`.
--
-- This used to be a second physics simulation: a 10 Hz loop writing setBotPosition/setBotVelocity/
-- setBotFacing/setBotAnimation itself, because the native walker could only reach points the navmesh
-- bake covered — and interaction pins, interiors and dropped carts are routinely off it. The host now
-- exposes a DIRECT seek (`walkTo(..., "direct <gait>")`) that walks straight at a point with the real
-- gait, so the body is simulated in one place instead of two. The signature and semantics are
-- unchanged; every caller keeps working.
function fly.to(tx: number, ty: number, tz: number, opts: FlyOpts?): boolean
    local o: FlyOpts = opts or {}
    local arrive = o.arrive or DEFAULTS.arrive

    fly.waitSpawn()
    flying = true

    -- `speed` was the script's own u/s cap; map it back onto the gait that produces it so callers that
    -- asked for a slow approach still get one.
    local speed = math.min(o.speed or DEFAULTS.speed, SPRINT_SPEED)
    local gait = "direct sprint"
    if speed <= 2.0 then
        gait = "direct walk"
    elseif speed <= 7.0 then
        gait = "direct jog"
    end

    walkTo(tx, ty, tz, gait)

    -- Give the driver a few ticks to take the walk, then hold until it finishes. `isWalking()` is the
    -- documented completion signal.
    local started = false
    for _ = 1, 6 do
        wait(100)
        if not flying then
            walkStop()
            return false
        end
        if isWalking() then
            started = true
            break
        end
    end

    if started then
        while flying and isWalking() do
            wait(100)
        end
        if not flying then
            walkStop()
            return false
        end
    end

    flying = false
    if not started then
        return false -- the walker never took it: no path, or already there and the caller can re-check
    end
    -- Judged in XY only. The walker deliberately never adopts the caller's Z (it is a world object's
    -- origin, not a standing height), so a 3-D test would fail every successful walk onto a pickup.
    local x, y = getBotPosition()
    local dx, dy = tx - x, ty - y
    return math.sqrt(dx * dx + dy * dy) <= math.max(arrive, ARRIVE_FLOOR)
end

return fly
