#!/usr/bin/env python3
"""v1ctl.py -- drive the V1 directly over miIO (Valetudo not in the loop).

For the robot experiments this talks to the robot's own ``miio_client`` on UDP
54321, i.e. the *same* path Valetudo uses, so the requests land in AppProxy and
in our LD_PRELOAD shim's handlers.  That is the point: `clean 3` exercises the
`app_segment_clean` -> `app_zoned_clean` rewrite (and the `V1SEG_DRYRUN_CLEAN`
log) without Valetudo's cached room list in the way, and it works even while the
stock (unpatched) Valetudo model is running.

The packet format is the miIO local protocol exactly as implemented by
Valetudo ``backend/lib/miio/Codec.js`` + ``MiioHeader.js`` + ``RetryWrapper.js``
(pinned checkout ``359a212e6897a31cea8df623b107fcb4a1373386``):

    header (32 B): magic 0x2131 | len BE16 (payload+32) | unknown BE32 |
                   deviceId BE32 | stamp BE32 | md5[16]
    hello        : header only, unknown=deviceId=stamp=0xffffffff, md5 = 16*0xff
    checksum     : md5(header[0:16] + token + encrypted_payload)
    crypto       : AES-128-CBC, key = md5(token), iv = md5(key + token), PKCS#7
                   the plaintext JSON may carry a trailing NUL (stripped on read)

Commands sent (shapes confirmed against the pinned Valetudo checkout):

    get_room_mapping   []                     -> [[id,"name"], ...]
    app_segment_clean  [{"segments":[1,2],    <- RoborockMapSegmentationCapability
                        "repeat":N,
                        "clean_order_mode":0|1}]
    app_zoned_clean    [[x1,y1,x2,y2,N], ...]

Usage (examples)
----------------
    # what rooms does the sidecar expose right now?
    python3 v1ctl.py --ip 192.168.1.50 --device-conf ./device.conf rooms

    # clean room 3 (one pass) -- watch the robot, or the v1appd/AppProxy log
    python3 v1ctl.py --ip 192.168.1.50 --token 0123...ef clean 3
    python3 v1ctl.py --ip 192.168.1.50 --token 0123...ef clean 1 4 --repeat 2

    # explicit rectangle, 5 cm cell frame (0..51200 mm)
    python3 v1ctl.py --ip 192.168.1.50 --token 0123...ef zone 6000 6000 8000 9000

    # escape hatch / anything not wrapped above
    python3 v1ctl.py --ip 192.168.1.50 --token 0123...ef raw get_map_v1
    python3 v1ctl.py --ip 192.168.1.50 --token 0123...ef raw app_segment_clean '[{"segments":[2],"repeat":1,"clean_order_mode":0}]'

    # no token / no AES dep, when the *patched* Valetudo is running:
    python3 v1ctl.py --via-valetudo http://192.168.1.50 rooms
    python3 v1ctl.py --via-valetudo http://192.168.1.50 clean 3

    python3 v1ctl.py selftest          # offline codec proof (round-trip + Node interop)

The **local** token is ``/mnt/data/miio/device.token``: a raw 16-byte value
(Valetudo's ``localSecret``).  ``--token-file`` reads it as bytes.  The
``key=`` value in ``/mnt/default/device.conf`` is Valetudo's *cloud* secret and
is only a fallback here (``--device-conf``); prefer the token file for local
miIO.

This is an operator tool, not part of the shim: nothing here runs on the robot.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

HEADER_LEN = 32
MAGIC = b"\x21\x31"
DEFAULT_PORT = 54321
HELLO_DEVICE_ID = 0xFFFFFFFF
MAX_INT32 = 0x7FFFFFFF
MSGID_DAY_OFFSET = 24 * 60 * 60          # Valetudo shifts msgIds by one day (in seconds)
FEB_1970_MS = 31 * 24 * 60 * 60 * 1000   # 1970-02-01T00:00:00Z


class MiioError(RuntimeError):
    """The robot answered with a JSON ``error`` object."""


# --------------------------------------------------------------------------- codec
def token_keys(token: bytes) -> tuple[bytes, bytes]:
    token_key = hashlib.md5(token).digest()
    token_iv = hashlib.md5(token_key + token).digest()
    return token_key, token_iv


def calc_msg_id(now_ms: int) -> int:
    """Valetudo MiioSocket.calculateMsgId(): 100 ids/s once the clock is set."""
    if now_ms > FEB_1970_MS:
        shifted = int(now_ms / 10 + 0.5)
        return MSGID_DAY_OFFSET + (shifted % (MAX_INT32 - MSGID_DAY_OFFSET))
    return int(now_ms / 1000 + 0.5)


def build_header(payload_len: int, device_id: int, stamp: int, unknown: int) -> bytearray:
    header = bytearray(b"\xff" * HEADER_LEN)
    header[0:2] = MAGIC
    struct.pack_into(">H", header, 2, payload_len + HEADER_LEN)
    struct.pack_into(">I", header, 4, unknown)
    struct.pack_into(">I", header, 8, device_id)
    struct.pack_into(">I", header, 12, stamp)
    return header


def hello_packet() -> bytes:
    """createMiioHeader() with defaults: everything 0xff except magic and length."""
    return bytes(build_header(0, HELLO_DEVICE_ID, HELLO_DEVICE_ID, HELLO_DEVICE_ID))


def _aes():
    try:
        from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
        from cryptography.hazmat.primitives import padding
    except ImportError:  # pragma: no cover - environment dependent
        sys.exit(
            "v1ctl: the miIO transport needs AES-128-CBC.\n"
            "  install it:  pip install cryptography\n"
            "  or drive the robot through Valetudo instead:  --via-valetudo http://<robot>"
        )
    return Cipher, algorithms, modes, padding


def aes_encrypt(token_key: bytes, token_iv: bytes, plaintext: bytes) -> bytes:
    Cipher, algorithms, modes, padding = _aes()
    padder = padding.PKCS7(128).padder()
    padded = padder.update(plaintext) + padder.finalize()
    encryptor = Cipher(algorithms.AES(token_key), modes.CBC(token_iv)).encryptor()
    return encryptor.update(padded) + encryptor.finalize()


def aes_decrypt(token_key: bytes, token_iv: bytes, ciphertext: bytes) -> bytes:
    Cipher, algorithms, modes, padding = _aes()
    decryptor = Cipher(algorithms.AES(token_key), modes.CBC(token_iv)).decryptor()
    raw = decryptor.update(ciphertext) + decryptor.finalize()
    try:
        unpadder = padding.PKCS7(128).unpadder()
        return unpadder.update(raw) + unpadder.finalize()
    except ValueError:
        # Tolerate a non-PKCS#7 / short payload the same way Codec.js tolerates junk.
        return raw.rstrip(b"\x00")


def encode_packet(token: bytes, device_id: int, stamp: int, payload: dict) -> bytes:
    token_key, token_iv = token_keys(token)
    body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    encrypted = aes_encrypt(token_key, token_iv, body)
    header = build_header(len(encrypted), device_id, stamp, 0)
    checksum = hashlib.md5(bytes(header[0:16]) + token + encrypted).digest()
    header[16:32] = checksum
    return bytes(header) + encrypted


def decode_packet(token: bytes, packet: bytes) -> dict:
    """Returns {'device_id', 'stamp', 'msg', 'token'} exactly like Codec.js."""
    if len(packet) < HEADER_LEN:
        raise ValueError("short packet (%d bytes)" % len(packet))
    header, encrypted = packet[:HEADER_LEN], packet[HEADER_LEN:]
    stamp = struct.unpack_from(">I", header, 12)[0]
    device_id = struct.unpack_from(">I", header, 8)[0]
    checksum = header[16:32]
    calculated = hashlib.md5(header[0:16] + token + encrypted).digest()

    msg = None
    new_token = None
    if checksum == calculated:
        if encrypted:
            token_key, token_iv = token_keys(token)
            plaintext = aes_decrypt(token_key, token_iv, encrypted)
            if plaintext.endswith(b"\x00"):
                plaintext = plaintext[:-1]
            try:
                msg = json.loads(plaintext.decode("utf-8"))
            except ValueError:
                msg = None          # same as Codec.js: log/ignore, do not crash
    elif not encrypted:
        # A mismatching checksum on an empty packet is how the robot offers its token.
        if checksum not in (b"\xff" * 16, b"\x00" * 16) and checksum != token:
            new_token = checksum
    return {"device_id": device_id, "stamp": stamp, "msg": msg, "token": new_token}


# --------------------------------------------------------------------------- transport
class MiioClient:
    def __init__(self, ip: str, token: bytes, port: int = DEFAULT_PORT,
                 timeout: float = 2.0, retries: int = 3, verbose: bool = False,
                 print_packet: bool = False, device_id: int | None = None):
        if len(token) != 16:
            sys.exit("v1ctl: token must be 16 bytes, got %d" % len(token))
        self.ip, self.port, self.token = ip, port, token
        self.timeout, self.retries, self.verbose = timeout, retries, verbose
        self.print_packet = print_packet
        # Valetudo sends the device.conf `did` in the header (MiioValetudoRobot.deviceId);
        # fall back to whatever the handshake reply carries.
        self.device_id = device_id if device_id is not None else HELLO_DEVICE_ID
        # The stamp is the robot's clock: seed it from the reply and advance it by local
        # elapsed time, exactly like Codec.js (Stamp.val + secondsPassed).
        self.stamp = 0
        self.stamp_monotonic = 0.0
        self.last_sent_stamp = 0
        self.handshaked = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    def _log(self, *args):
        if self.verbose:
            print(*args, file=sys.stderr)

    def _recv_until(self, deadline: float):
        while time.time() < deadline:
            try:
                packet, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                return None
            decoded = decode_packet(self.token, packet)
            if decoded["device_id"] != HELLO_DEVICE_ID and self.device_id == HELLO_DEVICE_ID:
                self.device_id = decoded["device_id"]
            if decoded["token"] is not None:
                self._log("adopting token from handshake:", decoded["token"].hex())
                self.token = decoded["token"]
            if decoded["stamp"]:
                if decoded["stamp"] > self.stamp:
                    self.stamp = decoded["stamp"]
                    self.stamp_monotonic = time.monotonic()
            self._log("<<", decoded["msg"] if decoded["msg"] is not None else {"stamp": decoded["stamp"]})
            return decoded
        return None

    def handshake(self) -> None:
        for _ in range(self.retries):
            self.sock.sendto(hello_packet(), (self.ip, self.port))
            decoded = self._recv_until(time.time() + self.timeout)
            if decoded is not None and decoded["stamp"]:
                self.handshaked = True
                self._log("handshake ok: deviceId=%d stamp=%d" % (self.device_id, self.stamp))
                return
        raise TimeoutError("no handshake reply from %s:%d (wrong token / robot offline?)"
                           % (self.ip, self.port))

    def _next_stamp(self) -> int:
        if self.stamp:
            # anchor to the robot's clock, add the local time elapsed since we saw it
            stamp = self.stamp + int(time.monotonic() - self.stamp_monotonic)
        else:
            stamp = int(time.time())
        if stamp <= self.last_sent_stamp:
            stamp = self.last_sent_stamp + 1
        self.last_sent_stamp = stamp
        return stamp

    def command(self, method: str, params, timeout: float | None = None):
        timeout = self.timeout if timeout is None else timeout
        if not self.handshaked:
            self.handshake()
        last_error = None
        # Valetudo resends the same message object on retries: a late reply to
        # the first attempt still resolves the pending id.  Generating a new id
        # per attempt could execute a slow app_segment_clean up to N times.
        msg_id = calc_msg_id(int(time.time() * 1000))
        payload = {"id": msg_id, "method": method, "params": params}
        for attempt in range(self.retries):
            packet = encode_packet(self.token, self.device_id, self._next_stamp(), payload)
            if self.print_packet:
                print(packet.hex(), file=sys.stderr)
            self._log(">>", payload)
            self.sock.sendto(packet, (self.ip, self.port))
            deadline = time.time() + timeout
            while True:
                decoded = self._recv_until(deadline)
                if decoded is None:
                    last_error = TimeoutError("%s timed out after %.1fs" % (method, timeout))
                    break
                msg = decoded["msg"]
                if not isinstance(msg, dict) or msg.get("id") != msg_id:
                    continue
                if "error" in msg:
                    raise MiioError("robot error for %s: %s" % (method, json.dumps(msg["error"])))
                return msg.get("result")
            self._log("retry %d/%d for %s" % (attempt + 1, self.retries, method))
        raise last_error if last_error else TimeoutError(method)


# --------------------------------------------------------------------------- valetudo REST
def valetudo_request(base: str, method: str, path: str, body=None, timeout: float = 5.0):
    data = json.dumps(body).encode("utf-8") if body is not None else None
    request = urllib.request.Request(base.rstrip("/") + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        raw = response.read()
    if not raw:
        return None
    text = raw.decode("utf-8", "replace")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        # e.g. Express sendStatus(200) -> the literal body "OK"
        return text


# --------------------------------------------------------------------------- commands
SEGMENT_CAP = "/api/v2/robot/capabilities/MapSegmentationCapability"


def make_client(args) -> MiioClient:
    return MiioClient(args.ip, read_token(args), args.port, args.timeout, args.retries,
                      args.verbose, args.print_packet, args.device_id)


def cmd_rooms(args) -> int:
    if args.via_valetudo:
        print(json.dumps(valetudo_request(args.via_valetudo, "GET", SEGMENT_CAP), indent=2))
        return 0
    print(json.dumps(make_client(args).command("get_room_mapping", []), indent=2))
    return 0


def cmd_clean(args) -> int:
    ids = [int(value) for value in args.ids]
    if args.via_valetudo:
        body = {"action": "start_segment_action", "segment_ids": [str(i) for i in ids],
                "iterations": args.repeat, "customOrder": bool(args.custom_order)}
        valetudo_request(args.via_valetudo, "PUT", SEGMENT_CAP, body)
        print("valetudo: start_segment_action %s iterations=%d" % (ids, args.repeat))
        return 0
    params = [{"segments": ids, "repeat": args.repeat,
               "clean_order_mode": 1 if args.custom_order else 0}]
    print(json.dumps(make_client(args).command("app_segment_clean", params)))
    return 0


def cmd_zone(args) -> int:
    params = [[args.x1, args.y1, args.x2, args.y2, args.repeat]]
    print(json.dumps(make_client(args).command("app_zoned_clean", params)))
    return 0


def cmd_raw(args) -> int:
    try:
        params = json.loads(args.params) if args.params else []
    except ValueError as error:
        sys.exit("v1ctl: invalid JSON params: %s" % error)
    print(json.dumps(make_client(args).command(args.method, params)))
    return 0


# --------------------------------------------------------------------------- token
TOKEN_RE = re.compile(r"^\s*(?P<key>[A-Za-z0-9_.]+)\s*=\s*(?P<value>\S+)")


def read_device_conf(path: str) -> dict:
    values = {}
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            match = TOKEN_RE.match(line)
            if match:
                values[match.group("key")] = match.group("value")
    return values


def device_id_from_conf(path: str) -> int | None:
    """The header device id Valetudo uses: parseInt(device.conf `did`)."""
    try:
        did = read_device_conf(path).get("did")
    except OSError:
        return None
    if did and did.isdigit():
        return int(did)
    return None


def parse_token_bytes(raw: bytes) -> bytes:
    """Valetudo's localSecret rule for a file: >=32 hex bytes -> 16 bytes,
    otherwise the first 16 raw bytes (``/mnt/data/miio/device.token``)."""
    if len(raw) >= 32 and re.fullmatch(rb"[0-9a-fA-F]{32}", raw[:32]):
        return bytes.fromhex(raw[:32].decode("ascii"))
    if len(raw) >= 16:
        return raw[:16]
    raise ValueError("token file is neither 32 hex bytes nor 16 raw bytes "
                     "(%d bytes)" % len(raw))


def parse_token_value(raw: str) -> bytes:
    """Turn a token string into the 16-byte AES key.

    Mirrors Valetudo's MiioValetudoRobot.localSecret/cloudSecret rules: a value of
    32+ hex characters is hex-decoded; anything of 16+ characters is taken as the
    raw token bytes (first 16 chars).  The V1's /mnt/default/device.conf ships the
    raw form (e.g. ``key=ExampleToken16ch``), not 32 hex.
    """
    value = raw.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        value = value[1:-1]
    if len(value) >= 32 and re.fullmatch(r"[0-9a-fA-F]{32}", value[:32]):
        return bytes.fromhex(value[:32])
    if len(value) >= 16:
        return value[:16].encode("ascii")
    raise ValueError("token is neither 32 hex chars nor 16 raw characters "
                     "(%d chars found)" % len(value))


def read_token(args) -> bytes:
    raw = None
    if args.token:
        raw = args.token
    elif args.token_file:
        # /mnt/data/miio/device.token is a raw 16-byte Buffer, not text.
        with open(args.token_file, "rb") as handle:
            return parse_token_bytes(handle.read())
    elif args.device_conf:
        with open(args.device_conf, "r", encoding="utf-8", errors="replace") as handle:
            for line in handle:
                match = TOKEN_RE.match(line)
                if match and match.group("key") == "key":
                    raw = match.group("value")
                    break
    if not raw:
        sys.exit("v1ctl: no token. Use --token <32 hex | 16 chars>, --token-file <file>, "
                 "--device-conf <device.conf> (the robot's /mnt/default/device.conf), "
                 "or --via-valetudo <url>.")
    try:
        return parse_token_value(raw)
    except ValueError as error:
        sys.exit("v1ctl: %s" % error)


# --------------------------------------------------------------------------- selftest
# An independent implementation of the same spec (Node's built-in crypto), used to
# cross-check the Python codec.  Mirrors Codec.js semantics: PKCS#7 + trailing-NUL strip.
NODE_CODEC = r"""
const crypto = require("crypto");
const [, , mode, tokenHex, payloadJson, stampArg] = process.argv;
const token = Buffer.from(tokenHex, "hex");
const tokenKey = crypto.createHash("md5").update(token).digest();
const tokenIV = crypto.createHash("md5").update(tokenKey).update(token).digest();
function header(plen, dev, stamp, unknown) {
    const h = Buffer.alloc(32, 0xff);
    h[0] = 0x21; h[1] = 0x31;
    h.writeUInt16BE(plen + 32, 2);
    h.writeUInt32BE(unknown, 4);
    h.writeUInt32BE(dev, 8);
    h.writeUInt32BE(stamp, 12);
    return h;
}
if (mode === "encode") {
    const cipher = crypto.createCipheriv("aes-128-cbc", tokenKey, tokenIV);
    const enc = Buffer.concat([cipher.update(Buffer.from(payloadJson, "utf8")), cipher.final()]);
    const h = header(enc.length, 0x12345678, Number(stampArg), 0);
    const sum = crypto.createHash("md5").update(h.subarray(0, 16)).update(token).update(enc).digest();
    sum.copy(h, 16);
    process.stdout.write(Buffer.concat([h, enc]).toString("hex"));
} else {
    const pkt = Buffer.from(payloadJson, "hex");
    const enc = pkt.subarray(32);
    const decipher = crypto.createDecipheriv("aes-128-cbc", tokenKey, tokenIV);
    let out = Buffer.concat([decipher.update(enc), decipher.final()]);
    if (out[out.length - 1] === 0) out = out.subarray(0, out.length - 1);
    process.stdout.write(out.toString("utf8"));
}
"""


# The same cross-check against the *real* Valetudo codec (its Logger is stubbed out,
# which is the only heavy require).  Used when --valetudo-src points at a checkout.
NODE_VALETUDO_CODEC = r"""
const Module = require("module");
const orig = Module._load;
Module._load = function (request) {
    if (String(request).endsWith("/Logger") || request === "../Logger") {
        return { info() {}, debug() {}, error() {}, trace() {}, warn() {} };
    }
    return orig.apply(this, arguments);
};
const [, , src, mode, tokenHex, payloadJson, stampArg] = process.argv;
const Codec = require(src.replace(/\/$/, "") + "/backend/lib/miio/Codec");
const codec = new Codec({ token: Buffer.from(tokenHex, "hex") });
if (mode === "encode") {
    codec.stamp = { val: Number(stampArg), time: Date.now(), orNew() { return this; } };
    process.stdout.write(codec.encodeOutgoingMiioPacket(JSON.parse(payloadJson), 0x12345678).toString("hex"));
} else {
    process.stdout.write(JSON.stringify(codec.decodeIncomingMiioPacket(Buffer.from(payloadJson, "hex")).msg));
}
"""


def cmd_selftest(args) -> int:
    failures = 0

    # [1] the handshake packet is the documented 32-byte hello.
    hello = hello_packet()
    checks = [
        (len(hello) == HEADER_LEN, "hello is 32 bytes"),
        (hello[0:2] == MAGIC, "hello magic 0x2131"),
        (struct.unpack_from(">H", hello, 2)[0] == HEADER_LEN, "hello length field = 32"),
        (hello[4:32] == b"\xff" * 28, "hello unknown/device/stamp/checksum = 0xff"),
    ]

    # [2] encode -> decode round-trip, and the pieces separately.
    token = bytes(range(16))
    token_key, token_iv = token_keys(token)
    checks.append((token_key == hashlib.md5(token).digest(), "key = md5(token)"))
    checks.append((token_iv == hashlib.md5(token_key + token).digest(), "iv = md5(key+token)"))
    payload = {"id": 4242, "method": "app_segment_clean",
               "params": [{"segments": [1, 3], "repeat": 2, "clean_order_mode": 0}]}
    packet = encode_packet(token, 0x12345678, 1700000000, payload)
    decoded = decode_packet(token, packet)
    checks.append((decoded["msg"] == payload, "encode->decode round-trip"))
    checks.append((decoded["device_id"] == 0x12345678, "device id round-trip"))
    checks.append((decoded["stamp"] == 1700000000, "stamp round-trip"))

    # [2b] token forms: 32 hex is hex-decoded, 16 chars are the raw bytes
    # (the V1's /mnt/default/device.conf uses the latter).
    checks.append((parse_token_value("000102030405060708090a0b0c0d0e0f") == bytes(range(16)),
                   "32-hex token -> 16 bytes"))
    checks.append((parse_token_value("ExampleToken16ch") == b"ExampleToken16ch",
                   "16-char token -> raw bytes"))
    checks.append((parse_token_bytes(b"000102030405060708090a0b0c0d0e0f") == bytes(range(16)),
                   "32-hex token file -> 16 bytes"))
    checks.append((parse_token_bytes(b"ExampleToken16ch") == b"ExampleToken16ch",
                   "16-byte token file -> raw bytes"))

    # [3] a corrupted checksum must not decode into a message.
    broken = bytearray(packet)
    broken[16] ^= 0xFF
    checks.append((decode_packet(token, bytes(broken))["msg"] is None, "bad checksum rejected"))

    # [4] msgId bounds (Valetudo's schedule).
    checks.append((0 < calc_msg_id(int(time.time() * 1000)) < MAX_INT32, "msgId in range"))
    checks.append((calc_msg_id(FEB_1970_MS + 1000) > MSGID_DAY_OFFSET, "msgId shifted when synced"))
    checks.append((calc_msg_id(FEB_1970_MS - 10 ** 9) >= 0, "msgId sane before Feb 1970"))

    for ok, label in checks:
        print("%-34s %s" % (label, "ok" if ok else "FAILED"))
        failures += 0 if ok else 1

    # [5] cross-check against an independent implementation (Node's crypto).
    if args.no_interop:
        print("%-34s %s" % ("node interop", "skipped (--no-interop)"))
    else:
        try:
            subprocess.run(["node", "--version"], check=True, capture_output=True)
            node_available = True
        except (FileNotFoundError, subprocess.CalledProcessError):
            node_available = False
        if not node_available:
            print("%-34s %s" % ("node interop", "SKIPPED (no node)"))
        else:
            with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as handle:
                handle.write(NODE_CODEC)
                node_path = handle.name
            try:
                token_hex = token.hex()
                payload_json = json.dumps(payload, separators=(",", ":"))
                node_packet = bytes.fromhex(subprocess.run(
                    ["node", node_path, "encode", token_hex, payload_json, "1700000000"],
                    check=True, capture_output=True, text=True).stdout)
                ok_encode = decode_packet(token, node_packet)["msg"] == payload
                node_text = subprocess.run(
                    ["node", node_path, "decode", token_hex, packet.hex(), ""],
                    check=True, capture_output=True, text=True).stdout
                ok_decode = json.loads(node_text) == payload
                for ok, label in ((ok_encode, "node-encoded -> python-decoded"),
                                  (ok_decode, "python-encoded -> node-decoded")):
                    print("%-34s %s" % (label, "ok" if ok else "FAILED"))
                    failures += 0 if ok else 1
            finally:
                os.unlink(node_path)

    # [6] the authoritative check: byte-compare against Valetudo's own Codec.js.
    if args.valetudo_src:
        codec_path = os.path.join(args.valetudo_src, "backend", "lib", "miio", "Codec.js")
        if not os.path.isfile(codec_path):
            print("%-34s %s" % ("valetudo Codec.js", "FAILED (not found: %s)" % codec_path))
            failures += 1
        else:
            with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as handle:
                handle.write(NODE_VALETUDO_CODEC)
                valetudo_node_path = handle.name
            try:
                token_hex = token.hex()
                payload_json = json.dumps(payload, separators=(",", ":"))
                reference = subprocess.run(
                    ["node", valetudo_node_path, args.valetudo_src, "encode",
                     token_hex, payload_json, "1700000000"],
                    check=True, capture_output=True, text=True).stdout.strip()
                decoded_by_valetudo = json.loads(subprocess.run(
                    ["node", valetudo_node_path, args.valetudo_src, "decode",
                     token_hex, packet.hex(), ""],
                    check=True, capture_output=True, text=True).stdout)
                for ok, label in ((reference == packet.hex(), "valetudo Codec.js bytes match"),
                                  (decoded_by_valetudo == payload, "valetudo Codec.js decodes ours")):
                    print("%-34s %s" % (label, "ok" if ok else "FAILED"))
                    failures += 0 if ok else 1
            finally:
                os.unlink(valetudo_node_path)

    print("selftest: %s" % ("PASS" if failures == 0 else "FAIL (%d)" % failures))
    return 0 if failures == 0 else 1


# --------------------------------------------------------------------------- cli
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Direct miIO control for the V1 (see the module docstring).",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ip", help="robot address (LAN)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="miIO port (default %d)" % DEFAULT_PORT)
    parser.add_argument("--token", help="miIO token, 32 hex chars")
    parser.add_argument("--token-file",
                        help="local token file (raw 16 bytes, e.g. "
                             "/mnt/data/miio/device.token; 32 hex also accepted)")
    parser.add_argument("--device-conf",
                        help="robot device.conf for 'did=' and key= (cloud secret; "
                             "the local token is device.token)")
    parser.add_argument("--device-id", type=int,
                        help="miIO device id (default: the `did` from --device-conf)")
    parser.add_argument("--via-valetudo", metavar="URL",
                        help="drive Valetudo's REST API instead of miIO (no token needed)")
    parser.add_argument("--timeout", type=float, default=2.0, help="per-reply timeout, seconds")
    parser.add_argument("--retries", type=int, default=3, help="send attempts")
    parser.add_argument("--verbose", action="store_true", help="log packets to stderr")
    parser.add_argument("--print-packet", action="store_true",
                        help="print the encoded packet hex to stderr (no extra effect)")

    sub = parser.add_subparsers(dest="command", required=True)

    rooms = sub.add_parser("rooms", help="get_room_mapping -> [[id,name],...]")
    rooms.set_defaults(func=cmd_rooms)

    clean = sub.add_parser("clean", help="app_segment_clean for one or more room ids")
    clean.add_argument("ids", nargs="+", help="room/segment ids (see `rooms`)")
    clean.add_argument("--repeat", type=int, default=1, help="passes (default 1)")
    clean.add_argument("--custom-order", action="store_true", help="clean_order_mode=1")
    clean.set_defaults(func=cmd_clean)

    zone = sub.add_parser("zone", help="app_zoned_clean with an explicit rectangle")
    zone.add_argument("x1", type=int)
    zone.add_argument("y1", type=int)
    zone.add_argument("x2", type=int)
    zone.add_argument("y2", type=int)
    zone.add_argument("--repeat", type=int, default=1)
    zone.set_defaults(func=cmd_zone)

    raw = sub.add_parser("raw", help="send an arbitrary method (+ JSON params)")
    raw.add_argument("method")
    raw.add_argument("params", nargs="?", help="JSON params array/object (default [])")
    raw.set_defaults(func=cmd_raw)

    selftest = sub.add_parser("selftest", help="offline codec proof")
    selftest.add_argument("--no-interop", action="store_true", help="skip the Node cross-check")
    selftest.add_argument("--valetudo-src", metavar="DIR",
                          help="Valetudo checkout; cross-check against its real Codec.js")
    selftest.set_defaults(func=cmd_selftest)

    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command != "selftest":
        if args.via_valetudo:
            if args.command not in ("rooms", "clean"):
                parser.error("--via-valetudo supports only `rooms` and `clean`")
        else:
            if not args.ip:
                parser.error("--ip is required (or use --via-valetudo)")
            if args.command == "clean" and args.repeat < 1:
                parser.error("--repeat must be >= 1")
            if args.device_id is None and args.device_conf:
                args.device_id = device_id_from_conf(args.device_conf)
    try:
        return args.func(args)
    except (MiioError, TimeoutError, ValueError) as error:
        sys.exit("v1ctl: %s" % error)
    except OSError as error:
        sys.exit("v1ctl: %s" % error)


if __name__ == "__main__":
    sys.exit(main())
