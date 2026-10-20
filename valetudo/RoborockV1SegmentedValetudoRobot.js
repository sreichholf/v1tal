/**
 * RoborockV1SegmentedValetudoRobot
 *
 * A **model definition only** for a Roborock/Rockrobo V1 that runs the on-robot
 * room-segmentation add-on (see README.md and
 * TECHNICAL.md).
 *
 * It is a robot-class definition and nothing else: it extends the stock
 * `RoborockV1ValetudoRobot` (so it keeps the V1 fan speeds, the "V1" model name and the
 * V1 attachment list) and registers exactly the *standard* capabilities that the
 * firmware now speaks. There is no `model === ...` special case, no parser change and
 * no capability-implementation change anywhere; the capability classes and the map
 * parser shipped by upstream Valetudo are used verbatim.
 *
 * Drop-in location (upstream tree):
 *   backend/lib/robots/roborock/RoborockV1SegmentedValetudoRobot.js
 *
 * Registration (the ONLY other edit; a registry entry, not special-casing):
 *   backend/lib/robots/roborock/index.js
 *       "RoborockV1SegmentedValetudoRobot": require("./RoborockV1SegmentedValetudoRobot"),
 *
 * Capability -> miio command map (all handled by upstream capability classes):
 *   RoborockMapSegmentationCapability     app_segment_clean [{segments,repeat,clean_order_mode}]
 *   RoborockMapSegmentEditCapability      merge_segment [idA,idB] / split_segment [id,x1,y1,x2,y2]
 *   RoborockMapSegmentRenameCapability    get_room_mapping [] / name_segment [{miRoomId,robotRoomId},...]
 *   RoborockMapResetCapability            reset_map []
 *   (names are re-fetched automatically on every new map by RoborockValetudoRobot.parseMap())
 *
 * Persistence is UNCONDITIONAL on this add-on: the keep-alive guard (v1guard /
 * persist_guard_tick) always protects the snapshot, with no user toggle.  There is no
 * `set_lab_status` handler, no `/mnt/data/rockrobo/lab.cfg` flag and no `lab_status`
 * read-back byte.  That is deliberate: the V1 `AppProxy` does not emit a `lab_status`
 * field in `get_status`, and the shim has no outbound reply hook to fabricate one, so
 * the S5-style `RoborockPersistentMapControlCapability` toggle could never read back as
 * enabled (review findings H2/H3).  Dropping the toggle — rather than shipping a
 * permanently-off capability — is the resolution by design.
 *
 * `RoborockMapResetCapability` is registered as the user's escape hatch: upstream sends
 * `reset_map []` and the shim answers it (`persist_map_reset`), which clears the live map
 * *and* the protected snapshot so the always-on guard cannot immediately resurrect a map
 * the user deliberately reset.  `reset_map` is the only control that expresses "stop
 * keeping this map".
 *
 * Deliberately NOT registered:
 *   RoborockMapSnapshotCapability -> the add-on implements `get_recover_maps` /
 *       `recover_map`, so it may be added as an optional extra; kept out to keep the
 *       diff minimal and because recovery is not a UI control.
 *   RoborockPersistentMapControlCapability -> removed with the toggle (see above).
 *       `set_lab_status` now passes through to the stock AppProxy unhandled.
 *   QuirksCapability (BUTTON_LEDS / MANUAL_MAP_SEGMENT_TRIGGER) -> the V1 has no on-robot
 *       re-segmentation trigger; leaving it out keeps the UI honest.
 */

const capabilities = require("./capabilities");
const MiioValetudoRobot = require("../MiioValetudoRobot");
const RoborockValetudoRobot = require("./RoborockValetudoRobot");
const RoborockV1ValetudoRobot = require("./RoborockV1ValetudoRobot");

/**
 * The distinct `model=` string this class auto-detects on.
 *
 * It is deliberately NOT the stock `rockrobo.vacuum.v1`: the robot's `/mnt/default/device.conf`
 * already reports that, and `RoborockV1ValetudoRobot` matches it too. Two classes matching the
 * same string would make `ValetudoRobotFactory.autodetectRobotImplementation()` depend on the
 * `Object.keys(Robots)` insertion order. A distinct id removes that ambiguity entirely.
 *
 * Must satisfy device.conf's key/value regex `[A-Za-z\d:.]+` (letters, digits, dots, colon).
 *
 * NOTE: this only matters for `robot.implementation: "auto"`. The supported selection is the
 * explicit config value `robot.implementation: "RoborockV1SegmentedValetudoRobot"`, which never
 * consults this handler. See BUILD.md §6.
 */
const SEGMENTED_V1_MODEL_ID = "rockrobo.vacuum.v1seg";

class RoborockV1SegmentedValetudoRobot extends RoborockV1ValetudoRobot {
    /**
     * @param {object} options
     * @param {import("../../Configuration")} options.config
     * @param {import("../../ValetudoEventStore")} options.valetudoEventStore
     */
    constructor(options) {
        // super() supplies everything V1-specific we want to keep:
        //   * fan speeds  MIN=1, LOW=38, MEDIUM=60, HIGH=75, MAX=100  (0..100, accepted by the V1)
        //   * getModelName() === "V1"
        //   * supportedAttachments: []
        // We do not override any of them.
        super(options);

        [
            // "Clean room" in the UI -> app_segment_clean [{segments:[..], repeat:N, clean_order_mode:0|1}]
            capabilities.RoborockMapSegmentationCapability,
            // merge / split in the UI -> merge_segment / split_segment
            capabilities.RoborockMapSegmentEditCapability,
            // room names -> get_room_mapping (read) / name_segment (write)
            capabilities.RoborockMapSegmentRenameCapability,
            // "reset map" -> reset_map [] (clears the live map AND the snapshot; the
            // escape hatch from the always-on keep-alive guard)
            capabilities.RoborockMapResetCapability,
        ].forEach(capability => {
            this.registerCapability(new capability({robot: this}));
        });
    }

    static IMPLEMENTATION_AUTO_DETECTION_HANDLER() {
        const deviceConf = MiioValetudoRobot.READ_DEVICE_CONF(RoborockValetudoRobot.DEVICE_CONF_PATH);

        return !!(deviceConf && deviceConf.model === SEGMENTED_V1_MODEL_ID);
    }
}

RoborockV1SegmentedValetudoRobot.MODEL_ID = SEGMENTED_V1_MODEL_ID;

module.exports = RoborockV1SegmentedValetudoRobot;
