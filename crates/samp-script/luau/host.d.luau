--!strict
-- Luau type DEFINITIONS for the headless SAMP.Lua host's globals — a tooling-only artifact.
--
-- The `samp-script` Rust host installs a set of globals (bot getters/setters, the `bitStream`
-- userdata, the CEF/RPC senders, crypto, the cp1251 codec) plus the Lua-side scheduler/helpers that
-- `require("addon")` leaks to global scope. None of these are declarable inline, so any module that
-- touches them can only be `--!nonstrict`. This `.d.luau` declares them so modules can go `--!strict`
-- and be type-checked by luau-lsp / `luau analyze --definitions=host.d.luau`.
--
-- NOT loaded at runtime (mlua strips types and the `require` resolver never lists it); it exists
-- purely for the type checker. Keep it in sync with `src/bindings.rs`, `src/bitstream.rs`,
-- `src/crypto.rs`, `src/lib.rs`, `luau/addon.luau`, and `luau/vector3d.luau`.

export type Vector3 = { x: number, y: number, z: number }

-- The native RakNet bitstream userdata (`src/bitstream.rs`) plus the helpers `addon.luau` adds to its
-- metatable (`*8` length-prefixed strings/bools, arrays, vectors). MSB-first bits, little-endian ints.
declare class BitStream
    -- writes
    function writeBool(self, v: boolean): ()
    function writeInt8(self, v: number): ()
    function writeUInt8(self, v: number): ()
    function writeInt16(self, v: number): ()
    function writeUInt16(self, v: number): ()
    function writeInt32(self, v: number): ()
    function writeUInt32(self, v: number): ()
    function writeFloat(self, v: number): ()
    function writeString(self, v: string): ()
    function writeEncoded(self, v: string): () -- SA-MP Huffman-encoded
    -- reads
    function readBool(self): boolean
    function readInt8(self): number
    function readUInt8(self): number
    function readInt16(self): number
    function readUInt16(self): number
    function readInt32(self): number
    function readUInt32(self): number
    function readFloat(self): number
    function readString(self, len: number): string
    function readEncoded(self, maxLen: number): string
    -- cursor / size
    function setReadOffset(self, bits: number): ()
    function setWriteOffset(self, bits: number): ()
    function getNumberOfUnreadBits(self): number
    function getNumberOfUnreadBytes(self): number
    function ignoreBits(self, bits: number): ()
    function reset(self): ()
    -- send (id byte prepended by the transport)
    function sendPacket(self): ()
    function sendPacketEx(self, priority: number, reliability: number, channel: number): ()
    function sendRPC(self, id: number): ()
    -- addon.luau metatable extensions
    function readString8(self): string
    function writeString8(self, v: string): ()
    function readBool8(self): boolean
    function writeBool8(self, v: boolean): ()
    function writeArray(self, v: { number }): ()
    function writeVector3(self, v: Vector3): ()
    function readVector3(self): Vector3
end

-- `bitStream.new()` constructs an empty stream (dot call, not method).
declare bitStream: { new: () -> BitStream }

-- Local-player state (`src/bindings.rs`).
declare function getBotPosition(): (number, number, number)
declare function setBotPosition(x: number, y: number, z: number): ()
declare function setBotVelocity(x: number, y: number, z: number): ()
-- Face the horizontal direction (dx, dy) — builds the on-wire quaternion about world +Z, the same
-- encoding the real client sends. Scripts driving movement via setBotPosition must call this or the
-- reported facing stays frozen and the bot slides sideways across the map.
declare function setBotFacing(dx: number, dy: number): ()
declare function setBotAnimation(id: number, flags: number): ()
declare function getBotRotation(): number
declare function getBotNick(): string
declare function setBotNick(nick: string): ()
declare function getServerAddress(): string
declare function getBotVehicle(): number
declare function setBotVehicle(id: number, seat: number): ()
declare function getBotWeapon(): number
-- NO-OP (kept for compatibility): the client never arms a weapon the server did not grant. The inventory
-- mirrors GivePlayerWeapon/SetPlayerArmedWeapon/SetWeaponAmmo/ResetPlayerWeapons only; faking one (e.g. the
-- rented chainsaw) is the "Чит на оружие(15/1)" kick. Job actions are driven by the RPC-131 pickup report.
declare function setBotWeapon(id: number, ammo: number?): ()
declare function getBotMoney(): number
declare function getBotInterior(): number
declare function getBotCameraPos(): (number, number, number)
declare function getBotKeys(): number
declare function setBotKeys(keys: number): ()
declare function updateSync(): ()
declare function sampSpawnPlayer(): ()
-- Native navmesh walker (driver-side; needs --navmesh <file.nav>). Fire-and-forget:
-- poll isWalking() for completion. walkStop() aborts the active walk. `mode` picks the GTA-SA gait:
-- "walk" (~3.7 u/s), "jog" (default, ~5.4 u/s), or "sprint" (~9.4 u/s).
-- `mode`: "walk" | "jog" (default) | "sprint", optionally prefixed with "direct " to walk straight at
-- the point instead of pathing over the navmesh — for spots the bake never covered. A direct walk uses
-- the same body, gait and reported velocity as a navmesh one; only the route differs.
declare function walkTo(x: number, y: number, z: number, mode: string?): ()
declare function walkStop(): ()
declare function isWalking(): boolean

-- Entities streamed in around the bot, as the client already tracks them for pathing. Read-only and
-- refreshed once per send tick, so poll them like isWalking(). `model` is present only where the
-- server told us (players streamed in via RPC 32).
export type StreamedEntity = { id: number, x: number, y: number, z: number, model: number? }
declare function getStreamedPlayers(): { StreamedEntity }
declare function getStreamedVehicles(): { StreamedEntity }
declare function reconnect(ms: number): ()

-- Outgoing wire (`src/lib.rs`).
declare function sampSendChat(text: string): ()
declare function sampSendPacket(data: string): ()
declare function sampSendRpc(id: number, data: string): ()
declare function sampSendDialogResponse(id: number, button: number, listItem: number, input: string): ()

-- cp1251 ↔ UTF-8 codec + byte length (`src/lib.rs`).
declare function cp1251ToUtf8(s: string): string
declare function utf8ToCp1251(s: string): string
declare function len(s: string): number

-- Crypto + Arizona attestation resources (`src/crypto.rs`).
declare function sha256(data: string): string
declare function hmacSha256(key: string, msg: string): string
declare function arizonaCoreResource(flag: number): string?

-- The append-based chokepoint registrar (`src/lib.rs`). Handlers vary by name: RPC/packet chokepoints
-- get `(id: number, bs: BitStream)`; lifecycle/`onUpdate` get `()`. Typed loosely as a result.
declare function registerHandler(name: string, fn: (...any) -> ...any): ()

-- `print(...)` → the host `tracing` logger (target `lua`).
declare function print(...: any): ()

-- The host's custom module resolver (`src/require.rs`) serves the embedded modules by name. It can't
-- be resolved by the type checker's sourcemap, so it is typed loosely as `any` — required modules
-- (`addon`, `packet`, `arizona`, `vector3d`, `samp.events`, …) surface as `any`.
declare function require(module: string): any

-- The single-slot event table the ported `samp.events` library populates (`sampev.onX = fn`).
declare sampev: { [string]: (...any) -> ...any }

-- App-injected config (`crates/app/src/main.rs`).
declare sampResolutionW: number
declare sampResolutionH: number
declare sampNick: string

-- Scheduler + helpers leaked to global scope by `require("addon")` (`luau/addon.luau`).
export type Task = {
    id: number,
    isAlive: (self: Task) -> boolean,
    isHalted: (self: Task) -> boolean,
    halt: (self: Task) -> (),
    resume: (self: Task) -> (),
    kill: (self: Task) -> (),
}
declare function wait(ms: number): ()
declare function newTask(f: (...any) -> ...any, halted: (number | boolean)?, ...: any): Task
declare function clearTasks(): ()
declare function isBotInAnyVehicle(): boolean

-- Vector3D constructor leaked by `require("vector3d")`.
declare function Vector3D(x: number?, y: number?, z: number?): Vector3
